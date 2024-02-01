include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : RCInput.bin
	stm32flash -b 115200 -f -v -w RCInput.bin /dev/ttyUSB0
