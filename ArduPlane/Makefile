include ../mk/apm.mk

.PHONY: upload_sp
upload_sp : /tmp/ArduPlane.build/ArduPlane.bin
	stm32flash -b 115200 -f -v -w /tmp/ArduPlane.build/ArduPlane.bin /dev/ttyUSB0

