EESchema Schematic File Version 2
LIBS:osd-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:contrib
LIBS:osd-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 11 20
Title ""
Date "9 jun 2016"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	6500 3450 6250 3450
Connection ~ 4450 2450
Wire Wire Line
	4450 3000 4450 2450
Connection ~ 5550 3850
Wire Wire Line
	5550 3850 8000 3850
Wire Wire Line
	8000 3850 8000 3150
Wire Wire Line
	8000 3150 6250 3150
Connection ~ 4700 3150
Wire Wire Line
	4850 3150 4700 3150
Wire Wire Line
	5550 4000 5550 3750
Wire Wire Line
	4250 2450 5550 2450
Wire Wire Line
	5550 2450 5550 2750
Wire Wire Line
	4850 3050 4700 3050
Wire Wire Line
	4700 3050 4700 3900
Connection ~ 5550 3900
Wire Wire Line
	4850 3250 4700 3250
Connection ~ 4700 3250
Wire Wire Line
	6250 3350 6500 3350
Wire Wire Line
	4450 3400 4450 3900
Connection ~ 4700 3900
Wire Wire Line
	4350 3900 5550 3900
Connection ~ 4450 3900
Text HLabel 4350 3900 0    60   BiDi ~ 0
GND
$Comp
L C-RESCUE-osd C27
U 1 1 5740E2B6
P 4450 3200
F 0 "C27" H 4500 3300 50  0000 L CNN
F 1 "0.1uF" H 4500 3100 50  0000 L CNN
F 2 "SMD_0603:SMD_0603" H 4450 3200 60  0001 C CNN
F 3 "" H 4450 3200 60  0001 C CNN
F 4 "10V" H 4450 3200 60  0001 C CNN "Description"
	1    4450 3200
	1    0    0    -1  
$EndComp
Text HLabel 6500 3450 2    60   BiDi ~ 0
I2C_SDA
Text HLabel 6500 3350 2    60   Output ~ 0
I2C_SCL
Text HLabel 4250 2450 0    60   Input ~ 0
3V3
$Comp
L GND-RESCUE-osd #PWR010
U 1 1 5740E1A4
P 5550 4000
F 0 "#PWR010" H 5550 4000 30  0001 C CNN
F 1 "GND" H 5550 3930 30  0001 C CNN
F 2 "" H 5550 4000 60  0001 C CNN
F 3 "" H 5550 4000 60  0001 C CNN
	1    5550 4000
	1    0    0    -1  
$EndComp
$Comp
L 24C512 U8
U 1 1 5740E12B
P 5550 3250
F 0 "U8" H 5700 3600 60  0000 C CNN
F 1 "M24M01" H 5750 2900 60  0000 C CNN
F 2 "SOIC8:SOIC8" H 5550 3250 60  0001 C CNN
F 3 "" H 5550 3250 60  0001 C CNN
	1    5550 3250
	1    0    0    -1  
$EndComp
$EndSCHEMATC
