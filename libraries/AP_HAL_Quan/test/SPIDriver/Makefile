include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : SPIDriver.bin
	stm32flash -b 115200 -f -v -w SPIDriver.bin /dev/ttyUSB0
