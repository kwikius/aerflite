include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : Airspeed.bin
	stm32flash -b 115200 -f -v -w Airspeed.bin /dev/ttyUSB0
