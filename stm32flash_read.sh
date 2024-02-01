#!/bin/bash

if [[ -n $1 ]]; then
   stm32flash -r $1 -b 115200 /dev/ttyUSB0
else
  echo "expected a filename argument for stm32flash to read to"
fi
 
