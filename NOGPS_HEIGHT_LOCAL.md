# 模式20高度起飞试验改造（AI辅助）

基线：ArduCopter 4.7.2 fb773567。保留既有590adad7板级功能配置。

## 接口

此本地改造替换GUIDED_NOGPS原语义，编号仍为20。仅地面进入，不支持空中重新进入。
普通GUIDED编号4的控制实现不改。

模式20、正常解锁后发送COMMAND_LONG（消息76）：command=31010（MAV_CMD_USER_1），param1=起飞上升距离（米），其余参数0。
目标系统和组件应从飞控心跳确认。command 31010是本地约定，非官方通用起飞命令，不能同时供其他自定义功能使用。
高度范围0.2至2m，向上为正，参考收到命令时EKF高度，不是原始测距绝对值。
收到COMMAND_ACK（77），command=31010，result=0才表示接受；不表示到达。
同一任务重复发送同一高度返回接受，不重新记录零点；活动任务期间不同高度拒绝。
未解锁、传感器不健康拒绝。目标超界、NaN、地面倾斜超过15度拒绝。
目标加当前测距距离必须小于测距配置上限，并预留0.2m。

## 执行

复用AltHold的takeoff.start_m/do_pilot_takeoff_ms起飞检测与油门渐增，以及AC_PosControl垂直轨迹和闭环。
最大上升0.5m/s、下降0.3m/s、加速度0.5m/s²（第一版固定值）。
保持水平与命令时航向，不做水平位置保持。
高度误差<0.10m且垂直速度<0.15m/s持续1秒后，STATUSTEXT报告NGH: target reached, holding。
目标不因不重复发送而超时。到达前20秒超时、测距/垂直状态失效、EKF高度重置或离地后再次判定落地，转LAND，复用现有降落路径。
LAND在垂直估计失效时不保证可靠定高或下降，应由遥控接管；不能将该退路视为测距失效仍可精确控制的保证。
模式20忽略SET_ATTITUDE_TARGET、SET_POSITION_TARGET_LOCAL_NED、SET_POSITION_TARGET_GLOBAL_INT。
标准NAV_TAKEOFF不支持此改造模式，必须使用31010。

## EKF与失联

这是方案A：高度反馈来自EKF，不是测距直接闭环。使用者必须配置当前EKF源组POSZ=RangeFinder，核对测距安装方向和真实量程。
检查测距健康不证明EKF融合来源；该代码没有重写EKF回退逻辑，也没有保证气压计绝不参与估计。
保留原有解锁/电池/RC/GCS保护。必须配置并验证GCS失联保护、上位机system ID及心跳接收匹配；不会自动改写用户参数。
光流不是模式20的水平控制输入；切入模式4前仍须满足普通GUIDED的位置条件。

## 交接

上位机接收到达状态后请求模式4，只有HEARTBEAT.custom_mode=4才开始发送速度目标。
切换拒绝时模式20继续保持，不自动反复切换。
STATUSTEXT可能丢失：上位机还应监测实际高度、垂直速度和模式；ACK不是到达确认。

## 修改文件

ArduCopter/mode_guided_nogps.cpp：高度命令、状态、控制与退出。
ArduCopter/mode.h：模式私有状态和接口；禁止原标准起飞、pause/resume。
ArduCopter/Copter.h：新模式访问现有Copter状态的friend声明。
ArduCopter/GCS_MAVLink_Copter.cpp：31010命令入口与模式20的旧控制消息隔离。
Tools/autotest/nogps_height_local.py：仅本地TCP SITL测试，不连接真实串口。

## 备份与回退

同级backup-before-nogps-height-20261108-133903保存修改前文件、SHA256、Git基线差异和版本。
回退只恢复上述四个ArduCopter源文件，保留原有板级配置；不要git reset --hard。
新增测试和此说明可保留用于审计。代码回退不自动回退已刷入的固件或运行参数。

## 验证状态

2026-11-08：SITL编译通过；MicoAir405v2完整构建通过（GCC 13.2.1），剩余Flash 88644字节。
四项SITL场景通过：handoff、no-flow、range-loss、gcs-loss。正常解锁检查保留，启动阶段等待IMU稳定后才成功解锁。
共同覆盖：未解锁拒绝、NaN拒绝、重复目标幂等、不同目标拒绝、旧姿态/速度消息不覆盖任务、到达检测。
SITL的向下测距模型地面为0m，因此测试用MIN=0、GNDCLR=0，仅适用于该理想模型，不能照搬实机。
最终测试到达时原始测距约1.57m；控制目标是EKF相对高度，不能承诺原始测距精确等于1.50m。
测距失效测试通过修改模拟测距有效上限使反馈无效；不是物理串口断线试验。
GCS失联测试启用FS_GCS_ENABLE=1、FS_GCS_TIMEOUT=5、FS_OPTIONS=0并由系统255发送心跳；实机必须核对对应配置。
未做实机试飞、台架验证或刷机；EKF高度重置退出和20秒未到达超时分支尚未注入故障做动态验证。
未评估不同风场、动力不足、地效、传感器噪声及安装误差；此APJ是试验版本。

构建副本位于WSL Ubuntu-24.04的/home/avant/ardupilot-nogps-height-4.7.2。
Windows源码与WSL副本四个控制源码及板级配置已逐文件校验一致（忽略CRLF差异）。
WSL make_intel_hex.py仅规范了行尾以修复python3\r错误，未改变脚本逻辑。
测试脚本/构建日志/飞行BIN/源码补丁在同级nogps-height-20261108目录。

源码回退：先运行备份目录restore_source.py预演，确认后带--apply恢复四个Copter源码文件。
脚本遇到后续修改会拒绝覆盖。原版固件备份为original-590adad7.apj，源码回退不会自动刷回该固件。

