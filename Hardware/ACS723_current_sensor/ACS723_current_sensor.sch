EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:acs723
LIBS:ACS723_current_sensor-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ACS723 U1
U 1 1 5B2948FD
P 5300 3300
F 0 "U1" H 5550 2950 60  0000 C CNN
F 1 "ACS723" H 5450 3300 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 5300 3300 60  0001 C CNN
F 3 "" H 5300 3300 60  0001 C CNN
	1    5300 3300
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J5
U 1 1 5B294981
P 8475 3300
F 0 "J5" H 8475 3500 50  0000 C CNN
F 1 "JST-GH-3" H 8475 3100 50  0000 C CNN
F 2 "jst_gh3_horz:JST_GH3_horz" H 8475 3300 50  0001 C CNN
F 3 "" H 8475 3300 50  0001 C CNN
	1    8475 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 3950 5300 4275
Wire Wire Line
	5300 4275 8075 4275
Wire Wire Line
	8075 4275 8075 3400
Wire Wire Line
	8075 3400 8275 3400
Wire Wire Line
	6725 3300 8275 3300
Wire Wire Line
	6725 2725 6725 3300
Wire Wire Line
	5300 2725 6725 2725
Wire Wire Line
	5300 2725 5300 2800
Wire Wire Line
	7075 3500 7075 3200
Wire Wire Line
	7075 3200 8275 3200
Wire Wire Line
	6000 3100 6225 3100
Wire Wire Line
	6225 3100 6225 2725
Connection ~ 6225 2725
$Comp
L C C1
U 1 1 5B294D41
P 7275 3875
F 0 "C1" H 7300 3975 50  0000 L CNN
F 1 "C" H 7300 3775 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 7313 3725 50  0001 C CNN
F 3 "" H 7275 3875 50  0001 C CNN
	1    7275 3875
	1    0    0    -1  
$EndComp
Wire Wire Line
	7275 3725 7275 3300
Connection ~ 7275 3300
Wire Wire Line
	7275 4025 7275 4275
Connection ~ 7275 4275
Text Notes 8900 2425 0    60   ~ 0
BATT
Text Notes 3300 2525 0    60   ~ 0
ESC
Text Label 6250 2450 0    60   ~ 0
BATT_IN
Text Label 8100 3300 0    60   ~ 0
+5V
Text Label 8100 3400 0    60   ~ 0
GND
Text Label 8000 3200 0    60   ~ 0
IOUT
$Comp
L Conn_01x01 J3
U 1 1 5B294F6B
P 8425 4500
F 0 "J3" H 8425 4600 50  0000 C CNN
F 1 "PAD" H 8425 4400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 8425 4500 50  0001 C CNN
F 3 "" H 8425 4500 50  0001 C CNN
	1    8425 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8225 4500 3875 4500
Text Label 5550 4500 0    60   ~ 0
ESC_BATT_GND
Wire Wire Line
	4800 3200 4675 3200
Wire Wire Line
	4675 3200 4675 2450
Wire Wire Line
	4675 2450 8275 2450
Wire Wire Line
	4800 3550 4500 3550
Wire Wire Line
	4500 3550 4500 2450
Wire Wire Line
	4500 2450 3925 2450
Text Label 4075 2450 0    60   ~ 0
ESC_OUT
$Comp
L Conn_01x01 J4
U 1 1 5B294FD2
P 8475 2450
F 0 "J4" H 8475 2550 50  0000 C CNN
F 1 "PAD" H 8475 2350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 8475 2450 50  0001 C CNN
F 3 "" H 8475 2450 50  0001 C CNN
	1    8475 2450
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J1
U 1 1 5B295057
P 3675 4500
F 0 "J1" H 3675 4600 50  0000 C CNN
F 1 "PAD" H 3675 4400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3675 4500 50  0001 C CNN
F 3 "" H 3675 4500 50  0001 C CNN
	1    3675 4500
	-1   0    0    1   
$EndComp
$Comp
L Conn_01x01 J2
U 1 1 5B294EC0
P 3725 2450
F 0 "J2" H 3725 2550 50  0000 C CNN
F 1 "PAD" H 3725 2350 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 3725 2450 50  0001 C CNN
F 3 "" H 3725 2450 50  0001 C CNN
	1    3725 2450
	-1   0    0    1   
$EndComp
$Comp
L PWR_FLAG #FLG01
U 1 1 5B296A26
P 6925 3225
F 0 "#FLG01" H 6925 3300 50  0001 C CNN
F 1 "PWR_FLAG" H 6925 3375 50  0000 C CNN
F 2 "" H 6925 3225 50  0001 C CNN
F 3 "" H 6925 3225 50  0001 C CNN
	1    6925 3225
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG02
U 1 1 5B296A9F
P 6875 4150
F 0 "#FLG02" H 6875 4225 50  0001 C CNN
F 1 "PWR_FLAG" H 6875 4300 50  0000 C CNN
F 2 "" H 6875 4150 50  0001 C CNN
F 3 "" H 6875 4150 50  0001 C CNN
	1    6875 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6925 3225 6925 3300
Connection ~ 6925 3300
Wire Wire Line
	6875 4150 6875 4275
Connection ~ 6875 4275
$Comp
L R R1
U 1 1 5B2A3D2A
P 6500 3500
F 0 "R1" V 6580 3500 50  0000 C CNN
F 1 "R" V 6500 3500 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6430 3500 50  0001 C CNN
F 3 "" H 6500 3500 50  0001 C CNN
	1    6500 3500
	0    1    1    0   
$EndComp
Wire Wire Line
	7075 3500 6650 3500
Wire Wire Line
	6350 3500 6000 3500
$EndSCHEMATC
