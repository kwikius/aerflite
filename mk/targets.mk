default: help

# convenient targets for our supported boards

sitl: all
sitl-arm: all
quan: all
empty: all

quan-upload: $(SKETCH).bin
	stm32flash -b 115200 -f -v -w $(SKETCH).bin /dev/ttyUSB0

clean:
	@rm -fr $(BUILDROOT)

