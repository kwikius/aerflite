#!/bin/bash
stm32flash -b 115200 -f -v -w /tmp/ArduPlane.build/ArduPlane.bin /dev/ttyUSB0
