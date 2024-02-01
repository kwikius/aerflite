#!/bin/bash
export PATH=$PATH:$HOME/cpp/projects/jsbsim/build/src
export PATH=$PATH:$HOME/cpp/projects/aerfpilot/Tools/autotest
#export PATH=$PATH:$HOME/cpp/projects/MAVProxy/
export PATH=/usr/lib/ccache:$PATH
#sim_vehicle.sh -v ArduPlane --map
sim_vehicle.sh -v ArduPlane
#/home/andy/cpp/projects/aerfpilot/ArduPlane/ArduPlane.elf -S -I0 --home 37.619373,-122.376637,5.3,118 --model jsbsim --speedup 1 \
## --defaults /home/andy/cpp/projects/ardupilot/Tools/autotest/default_params/plane-jsbsim.parm

#mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551
#sim_vehicle.sh -j4 -L KSFO


