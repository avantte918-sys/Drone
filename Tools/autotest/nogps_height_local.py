#!/usr/bin/env python3
"""Local SITL-only test for the custom mode-20 height interface (AI-assisted)."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from pymavlink import mavutil

parser = argparse.ArgumentParser()
parser.add_argument('--binary', required=True)
parser.add_argument('--scenario', choices=['handoff', 'range-loss', 'no-flow', 'gcs-loss'], default='handoff')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
artifact_root = Path.home() / 'ngh-validation'
artifact_root.mkdir(exist_ok=True)
work = Path(tempfile.mkdtemp(prefix='ngh-sitl-', dir=artifact_root))
params = work / 'test.parm'
params.write_text("""GPS1_TYPE 0
GPS2_TYPE 0
AHRS_EKF_TYPE 3
EK3_ENABLE 1
EK2_ENABLE 0
EK3_SRC1_POSXY 0
EK3_SRC1_VELXY 5
EK3_SRC1_POSZ 2
EK3_SRC1_VELZ 0
EK3_SRC1_YAW 1
RNGFND1_TYPE 100
RNGFND1_MIN 0
RNGFND1_GNDCLR 0
RNGFND1_MAX 10
RNGFND1_ORIENT 25
FLOW_TYPE 10
SIM_FLOW_ENABLE 1
FS_GCS_ENABLE 1
FS_GCS_TIMEOUT 5
FS_OPTIONS 0
DISARM_DELAY 0
LOG_DISARMED 1
""")
if args.scenario == 'no-flow':
    with params.open('a') as f:
        f.write('SIM_FLOW_ENABLE 0\nEK3_SRC1_VELXY 0\n')
out = (work / 'sitl.txt').open('w')
proc = subprocess.Popen([str(Path(args.binary).resolve()), '--model', '+', '--home', '-35.362938,149.165085,584,353', '--speedup', '1', '--defaults', str(root/'Tools/autotest/default_params/copter.parm')+','+str(params), '-w'], cwd=work, stdout=out, stderr=subprocess.STDOUT)
m = None
stop = threading.Event()
records = []
latest = {}
texts = []
def recv_until(predicate, seconds=15):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        msg = m.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        latest[msg.get_type()] = msg
        if msg.get_type() == 'STATUSTEXT':
            texts.append(msg.text)
            print(msg.text, flush=True)
        if predicate(msg):
            return msg
    raise AssertionError('timeout; latest text: '+repr(texts[-12:]))
def command(cmd, *params):
    p = list(params)+[0]*(7-len(params))
    m.mav.command_long_send(1,1,cmd,0,*p)
    ack = recv_until(lambda x: x.get_type()=='COMMAND_ACK' and x.command==cmd)
    records.append({'command':cmd, 'result':ack.result})
    return ack.result
def mode(number):
    m.mav.set_mode_send(1,1,number)
    return recv_until(lambda x:x.get_type()=='HEARTBEAT' and x.custom_mode==number)
def setparam(name,value):
    m.mav.param_set_send(1,1,name.encode(),value,mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    recv_until(lambda x:x.get_type()=='PARAM_VALUE' and x.param_id==name)
def heartbeat():
    while not stop.is_set():
        m.mav.heartbeat_send(6,8,0,0,4)
        stop.wait(1)
try:
    for _ in range(80):
        if proc.poll() is not None:
            raise RuntimeError((work/'sitl.txt').read_text())
        try:
            m=mavutil.mavlink_connection('tcp:127.0.0.1:5760',source_system=255)
            break
        except OSError:
            time.sleep(0.25)
    assert m is not None
    threading.Thread(target=heartbeat,daemon=True).start()
    recv_until(lambda x:x.get_type()=='HEARTBEAT',30)
    m.mav.request_data_stream_send(1,1,0,10,1)
    # Provide a local origin, not a GPS observation.
    m.mav.set_gps_global_origin_send(1,-353629380,1491650850,584000)
    recv_until(lambda x:x.get_type()=='EKF_STATUS_REPORT' and x.flags & 1 and x.flags & 4 and x.flags & 32,60)
    mode(20)
    assert command(31010,1.5) != 0, 'must reject while disarmed'
    assert command(31010,float('nan')) != 0
    # Wait for all normal arming checks; never force-arm.
    for _ in range(40):
        if command(400,1)==0:
            break
        recv_until(lambda x:x.get_type()=='HEARTBEAT',5)
        time.sleep(1)
    else:
        raise AssertionError('normal arming failed')
    assert command(31010,1.5)==0
    assert command(31010,1.5)==0, 'retry must be idempotent'
    assert command(31010,1.0)!=0, 'conflicting target must be rejected'
    # Legacy attitude and velocity streams must not override the height task.
    m.mav.set_attitude_target_send(0,1,1,7,[0.9238795,0.3826834,0,0],0,0,0,1)
    m.mav.set_position_target_local_ned_send(0,1,1,1,3527,0,0,0,3,0,-2,0,0,0,0,0)
    recv_until(lambda x:x.get_type()=='STATUSTEXT' and 'NGH: target reached' in x.text,35)
    attitude=recv_until(lambda x:x.get_type()=='ATTITUDE')
    assert abs(attitude.roll)<0.1 and abs(attitude.pitch)<0.1, attitude
    distance=recv_until(lambda x:x.get_type()=='DISTANCE_SENSOR')
    assert 120 <= distance.current_distance <= 190, distance
    records.append({'reached_distance_cm':distance.current_distance})
    if args.scenario == 'range-loss':
        setparam('RNGFND1_MAX',0.1)
        recv_until(lambda x:x.get_type()=='HEARTBEAT' and x.custom_mode==9,10)
    elif args.scenario == 'gcs-loss':
        stop.set()
        recv_until(lambda x:x.get_type()=='HEARTBEAT' and x.custom_mode!=20,15)
    elif args.scenario == 'no-flow':
        m.mav.set_mode_send(1,1,4)
        recv_until(lambda x:x.get_type()=='STATUSTEXT' and ('failed' in x.text.lower() or 'requires position' in x.text.lower()),10)
        hb=recv_until(lambda x:x.get_type()=='HEARTBEAT')
        assert hb.custom_mode==20
    else:
        mode(4)
        for _ in range(20):
            m.mav.set_position_target_local_ned_send(0,1,1,1,3527,0,0,0,0,0,0,0,0,0,0,0)
            recv_until(lambda x:x.get_type()=='ATTITUDE',2)
        mode(9)
    # Allow the DataFlash buffer to flush the final mode/reason before stopping SITL.
    for _ in range(4):
        final_hb=recv_until(lambda x:x.get_type()=='HEARTBEAT',5)
    records.append({'scenario':args.scenario,'passed':True,'final_mode':final_hb.custom_mode})
    print(json.dumps(records),flush=True)
finally:
    stop.set()
    if m is not None:
        m.close()
    proc.terminate()
    try:
        proc.wait(5)
    except subprocess.TimeoutExpired:
        proc.kill()
    out.close()
    (work/'results.json').write_text(json.dumps(records,indent=2))
    print('ARTIFACTS',work,flush=True)
