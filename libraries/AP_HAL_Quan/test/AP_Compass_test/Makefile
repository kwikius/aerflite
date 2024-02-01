include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : AP_Compass_test.bin
	stm32flash -b 115200 -f -v -w AP_Compass_test.bin /dev/ttyUSB0
