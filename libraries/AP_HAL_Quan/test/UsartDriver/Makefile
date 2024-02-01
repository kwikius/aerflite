include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : UsartDriver.bin
	stm32flash -b 115200 -f -v -w UsartDriver.bin /dev/ttyUSB0

