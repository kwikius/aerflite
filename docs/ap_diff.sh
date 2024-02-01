#!/bin/bash

aerfpilot_dir='/home/andy/cpp/projects/aerfpilot/libraries/SITL/'
ardupilot_dir='/home/andy/cpp/projects/ardupilot/libraries/SITL/'

diff "$aerfpilot_dir$1" "$ardupilot_dir$1"
