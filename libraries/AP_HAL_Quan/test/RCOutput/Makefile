include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : RCOutput.bin
	stm32flash -b 115200 -f -v -w RCOutput.bin /dev/ttyUSB0
