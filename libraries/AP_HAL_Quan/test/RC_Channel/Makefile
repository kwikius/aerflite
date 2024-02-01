include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : RC_Channel.bin
	stm32flash -b 115200 -f -v -w RC_Channel.bin /dev/ttyUSB0
