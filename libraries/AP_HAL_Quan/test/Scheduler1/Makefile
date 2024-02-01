include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : Scheduler1.bin
	stm32flash -b 115200 -f -v -w Scheduler1.bin /dev/ttyUSB0
