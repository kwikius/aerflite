# top level makefile to build SITL for primary vehicle targets.
# Useful for static analysis tools

##N.b must be defined in env for quan build

#export QUAN_TARGET_VEHICLE=QUAN_APM_ARDUPLANE
#export QUANTRACKER_ROOT_DIR= < path to quantracker>

export QUAN_TARGET_VEHICLE=QUAN_APM_ARDUPLANE
export QUANTRACKER_ROOT_DIR=/home/andy/old_acer_laptop/projects/quantracker/

all: sitl

sitl: TARGET=sitl
sitl: plane

clean: TARGET=clean
clean: plane

QUAN_TEST_DIR = libraries/AP_HAL_Quan/test/

quan-test-ins-clean:
	$(MAKE) -C $(QUAN_TEST_DIR)INS_generic/ clean

quan-test-ins:
	$(MAKE) -C $(QUAN_TEST_DIR)INS_generic/ quan

aerflite-test-ins:
	$(MAKE) AERFLITE=True -C $(QUAN_TEST_DIR)INS_generic/ quan

quan-test-ahrs-clean:
	$(MAKE) -C $(QUAN_TEST_DIR)AHRS/ clean

quan-test-ahrs:
	$(MAKE) -C $(QUAN_TEST_DIR)AHRS/ quan

quan-test-gps-clean:
	$(MAKE) -C $(QUAN_TEST_DIR)GPS_AUTO_test/ clean

quan-test-gps:
	$(MAKE) -C $(QUAN_TEST_DIR)GPS_AUTO_test/ quan

quan-test-airspeed-clean:
	$(MAKE) -C $(QUAN_TEST_DIR)Airspeed/ clean

quan-test-airspeed:
	$(MAKE) -C $(QUAN_TEST_DIR)Airspeed/ quan

# TODO tests to add
# done - AHRS
# done - Airspeed
#AnalogIn
#AP_Compass_test
#BARO_test
#BatteryMonitor
#CompassCal
#GPIO
#done - GPS_AUTO_test
#I2CDriver
#done - INS_generic
#maxtest
#Param
#RC_Channel
#RCInput
#RCOutput
#Scheduler
#Scheduler1
#SPIDriver
#Storage
#StorageManager_Test
#test_out.txt
#UsartDriver

.PHONY: all plane sitl clean quan-test-ins-clean quan-test-ins \
quan-test-ahrs-clean quan-test-ahrs quan-test-gps quan-test-gps-clean \
quan-test-airspeed quan-test-airspeed-clean

plane:
	$(MAKE) -C ArduPlane $(TARGET)

