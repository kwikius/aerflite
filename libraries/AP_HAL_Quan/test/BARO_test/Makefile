include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : BARO_test.bin
	stm32flash -b 115200 -f -v -w BARO_test.bin /dev/ttyUSB0
