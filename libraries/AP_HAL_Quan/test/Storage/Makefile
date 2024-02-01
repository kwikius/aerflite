QUAN_CUSTOM_DEFINES = QUAN_APM_DONT_START_I2C_TASK

include ../../../../mk/apm.mk

.PHONY: upload_sp
upload_sp : Storage.bin
	stm32flash -b 115200 -f -v -w Storage.bin /dev/ttyUSB0
