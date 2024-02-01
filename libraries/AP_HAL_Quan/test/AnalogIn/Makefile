
include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : AnalogIn.bin
	stm32flash -b 115200 -f -v -w AnalogIn.bin /dev/ttyUSB0
