include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : Test1.bin
	stm32flash -b 115200 -f -v -w Test1.bin /dev/ttyUSB0
