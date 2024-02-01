include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : GPIO.bin
	stm32flash -b 115200 -f -v -w GPIO.bin /dev/ttyUSB0

