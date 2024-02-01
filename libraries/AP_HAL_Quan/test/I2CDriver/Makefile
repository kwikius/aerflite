include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : I2CDriver.bin
	stm32flash -b 115200 -f -v -w I2CDriver.bin /dev/ttyUSB0
