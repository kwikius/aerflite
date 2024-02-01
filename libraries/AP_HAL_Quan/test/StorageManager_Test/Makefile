include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : StorageManager_Test.bin
	stm32flash -b 115200 -f -v -w StorageManager_Test.bin /dev/ttyUSB0
