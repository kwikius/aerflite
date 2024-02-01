include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : INS_generic.bin
	stm32flash -b 115200 -f -v -w INS_generic.bin /dev/ttyUSB0
