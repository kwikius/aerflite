include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : SDP3x.bin
	stm32flash -b 115200 -f -v -w SDP3x.bin /dev/ttyUSB0
