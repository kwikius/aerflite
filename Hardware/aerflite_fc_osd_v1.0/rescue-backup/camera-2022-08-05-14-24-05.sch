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
Sheet 5 20
Title ""
Date "9 jun 2016"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Label 4725 3500 0    45   ~ 0
VIN+P22
Text Label 4025 3250 0    45   ~ 0
VIDEO_IN_P22
$Comp
L CAP3T C8
U 1 1 57499A5B
P 5650 3500
F 0 "C8" H 5800 3250 60  0000 C CNN
F 1 "NFM21PC104R1E3" H 5850 3750 60  0000 C CNN
F 2 "footprints:NFM21CC" H 5650 3500 60  0001 C CNN
F 3 "" H 5650 3500 60  0001 C CNN
F 4 "rf_filter/LP/25V/2A" H 5650 3500 60  0001 C CNN "Description"
	1    5650 3500
	1    0    0    -1  
$EndComp
Connection ~ 4575 4400
Wire Wire Line
	4575 3150 4575 4400
Wire Wire Line
	4150 2875 4025 2875
Wire Wire Line
	4025 2875 4025 3400
Wire Wire Line
	4025 3400 3850 3400
Wire Wire Line
	5650 3700 5650 4400
Wire Wire Line
	7300 3500 5900 3500
Wire Wire Line
	4200 3600 3850 3600
Wire Wire Line
	5400 3500 3850 3500
Wire Wire Line
	4050 3600 4050 4400
Connection ~ 4050 3600
Wire Wire Line
	4050 4400 5650 4400
Wire Wire Line
	5000 2875 5175 2875
$Comp
L TLCFILT LC3
U 1 1 5744B466
P 4625 3200
F 0 "LC3" H 4775 3300 60  0000 C CNN
F 1 "NFL18ST207X1C3" H 4750 3700 60  0000 C CNN
F 2 "SMD_0603-3T_FILT:SMD_0603-3T_LC_FILT_RND" H 4625 3200 60  0001 C CNN
F 3 "rf_filters/NFL18ST207X1C3" H 4625 3200 60  0001 C CNN
F 4 "rf filter/LP/200MHz/150mA/16V" H 4625 3200 60  0001 C CNN "Description"
	1    4625 3200
	1    0    0    -1  
$EndComp
Text HLabel 7300 3500 2    45   BiDi ~ 0
VIN+
Text HLabel 5175 2875 2    45   Output ~ 0
VIDEO_IN
Text HLabel 4200 3600 2    45   BiDi ~ 0
GND
$Comp
L CONN_3 P22
U 1 1 546F3160
P 3500 3500
F 0 "P22" V 3450 3500 50  0000 C CNN
F 1 "VIDEO_IN" V 3550 3500 40  0000 C CNN
F 2 "JST_GH3_horz:JST_GH3_horz" H 3500 3500 60  0001 C CNN
F 3 "" H 3500 3500 60  0001 C CNN
F 4 "0.1 in pitch header connector/3 contacts" H 3500 3500 60  0001 C CNN "Description"
	1    3500 3500
	-1   0    0    1   
$EndComp
$EndSCHEMATC
