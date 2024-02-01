include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : Scheduler.bin
	stm32flash -b 115200 -f -v -w Scheduler.bin /dev/ttyUSB0
