include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : CompassCal.bin
	stm32flash -b 115200 -f -v -w CompassCal.bin /dev/ttyUSB0
