#!/usr/bin/env bash
set -eu
cd /home/avant/ardupilot-nogps-height-4.7.2
export PYTHONPATH=/home/avant/ardupilot-build-python:/home/avant/ngh-test-python
mkdir -p /home/avant/ngh-validation
scenarios=("$@")
if [ ${#scenarios[@]} -eq 0 ]; then scenarios=(handoff no-flow range-loss gcs-loss); fi
for scenario in "${scenarios[@]}"; do
    python3 Tools/autotest/nogps_height_local.py --binary build/sitl/bin/arducopter --scenario "$scenario" > "/home/avant/ngh-validation/$scenario.log" 2>&1 || { tail -20 "/home/avant/ngh-validation/$scenario.log"; exit 1; }
    tail -2 "/home/avant/ngh-validation/$scenario.log"
done
