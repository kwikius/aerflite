QUAN_CUSTOM_DEFINES = QUAN_APM_DONT_START_I2C_TASK  QUAN_APM_DONT_START_START_IMU_TASK

include ../../../../mk/apm.mk

 .PHONY: upload_sp
upload_sp : GPS_AUTO_test.bin
	stm32flash -b 115200 -f -v -w GPS_AUTO_test.bin /dev/ttyUSB0
