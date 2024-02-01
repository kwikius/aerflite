#!/bin/bash
# check 0
export QUAN_TARGET_VEHICLE=QUAN_APM_ARDUPLANE
export QUANTRACKER_ROOT_DIR=/home/andy/old_acer_laptop/projects/quantracker/
cd ArduPlane && make MIXER_DISCO=True quan
