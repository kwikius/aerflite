#!/bin/bash
export PATH=$PATH:/home/andy/andy_bin/gcc-arm-none-eabi-4_9-2015q3/bin
../modules/waf/waf-light configure --board=px4-v2
../modules/waf/waf-light build
