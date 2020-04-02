EESchema Schematic File Version 4
LIBS:weeSpindel-cache
EELAYER 26 0
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
Text GLabel 1300 5750 0    50   Input ~ 0
RST
Text GLabel 1300 5950 0    50   Input ~ 0
EN
$Comp
L Device:C_Small C1
U 1 1 5B0BCD0C
P 1400 5050
F 0 "C1" H 1492 5096 50  0000 L CNN
F 1 "100nF" H 1492 5005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1400 5050 50  0001 C CNN
F 3 "~" H 1400 5050 50  0001 C CNN
	1    1400 5050
	0    1    1    0   
$EndComp
Text GLabel 2300 4850 1    50   Input ~ 0
3V3
$Comp
L power:GND #PWR02
U 1 1 5B0BCD6E
P 2300 7200
F 0 "#PWR02" H 2300 6950 50  0001 C CNN
F 1 "GND" H 2305 7027 50  0000 C CNN
F 2 "" H 2300 7200 50  0001 C CNN
F 3 "" H 2300 7200 50  0001 C CNN
	1    2300 7200
	1    0    0    -1  
$EndComp
Text GLabel 1300 6050 0    50   Input ~ 0
GPIO16
Text GLabel 7150 1300 0    50   Input ~ 0
EN
$Comp
L Device:R R6
U 1 1 5B0C7A88
P 7500 1300
F 0 "R6" V 7293 1300 50  0000 C CNN
F 1 "10K" V 7384 1300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7430 1300 50  0001 C CNN
F 3 "~" H 7500 1300 50  0001 C CNN
	1    7500 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	7150 1300 7250 1300
Text GLabel 6950 2500 0    50   Input ~ 0
RST
Text GLabel 7750 950  1    50   Input ~ 0
3V3
$Comp
L Device:R R7
U 1 1 5B0C938A
P 7500 1700
F 0 "R7" V 7293 1700 50  0000 C CNN
F 1 "10K" V 7384 1700 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7430 1700 50  0001 C CNN
F 3 "~" H 7500 1700 50  0001 C CNN
	1    7500 1700
	0    1    1    0   
$EndComp
Text GLabel 7150 1700 0    50   Input ~ 0
GPIO0
Text GLabel 3350 6350 2    50   Input ~ 0
GPIO15
Wire Wire Line
	7300 3350 7500 3350
$Comp
L Device:R R8
U 1 1 5B0FC92D
P 7650 3350
F 0 "R8" V 7750 3350 50  0000 C CNN
F 1 "10K" V 7800 3350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7580 3350 50  0001 C CNN
F 3 "~" H 7650 3350 50  0001 C CNN
	1    7650 3350
	0    1    1    0   
$EndComp
Text GLabel 7300 3350 0    50   Input ~ 0
GPIO15
Wire Wire Line
	7650 1700 7750 1700
Wire Wire Line
	7650 1300 7750 1300
Wire Wire Line
	3200 6350 3350 6350
Wire Wire Line
	2300 6950 2300 7000
Wire Wire Line
	900  7000 1300 7000
Connection ~ 2300 7000
Wire Wire Line
	2300 7000 2300 7200
Wire Wire Line
	1300 5950 1400 5950
Wire Wire Line
	1300 6050 1400 6050
Wire Wire Line
	1300 5750 1400 5750
Wire Wire Line
	1500 5050 2300 5050
Wire Wire Line
	2300 4850 2300 5050
Connection ~ 2300 5050
Wire Wire Line
	2300 5050 2300 5150
Wire Wire Line
	1300 5050 900  5050
Wire Wire Line
	900  5050 900  7000
$Comp
L ESP8266:ESP-12 U1
U 1 1 5B0BCB07
P 2300 6050
F 0 "U1" H 2300 7328 50  0000 C CNN
F 1 "ESP-12E" H 2300 7237 50  0000 C CNN
F 2 "CPB:ESP-12Emin" H 2300 6300 50  0001 C CNN
F 3 "http://wiki.ai-thinker.com/_media/esp8266/docs/aithinker_esp_12f_datasheet_en.pdf" H 1950 6350 50  0001 C CNN
	1    2300 6050
	1    0    0    -1  
$EndComp
Text GLabel 1300 6250 0    50   Input ~ 0
DQ
Wire Wire Line
	6950 2500 7150 2500
Text Notes 6100 2350 0    50   ~ 0
RST: newer ESP-12F has\ninternal 12k pullup
$Comp
L Device:R R5
U 1 1 5CA494A3
P 7400 2500
F 0 "R5" V 7500 2500 50  0000 C CNN
F 1 "47K" V 7550 2500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7330 2500 50  0001 C CNN
F 3 "~" H 7400 2500 50  0001 C CNN
	1    7400 2500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5CAD35A6
P 8300 3650
F 0 "#PWR07" H 8300 3400 50  0001 C CNN
F 1 "GND" H 8305 3477 50  0000 C CNN
F 2 "" H 8300 3650 50  0001 C CNN
F 3 "" H 8300 3650 50  0001 C CNN
	1    8300 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 3350 8300 3350
Connection ~ 8300 3350
Wire Wire Line
	8300 3350 8300 3650
Wire Wire Line
	7750 950  7750 1300
Connection ~ 7750 1300
Wire Wire Line
	7750 1300 7750 1700
$Comp
L Device:R_Small R4
U 1 1 5CC24126
P 7150 2800
F 0 "R4" V 7350 2800 50  0000 C CNN
F 1 "1K" V 7250 2800 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7150 2800 50  0001 C CNN
F 3 "~" H 7150 2800 50  0001 C CNN
	1    7150 2800
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5CC4D333
P 8050 1400
F 0 "C7" H 8142 1446 50  0000 L CNN
F 1 "100nF" H 8142 1355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8050 1400 50  0001 C CNN
F 3 "~" H 8050 1400 50  0001 C CNN
	1    8050 1400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7950 1400 7250 1400
Wire Wire Line
	7250 1400 7250 1300
Connection ~ 7250 1300
Wire Wire Line
	7250 1300 7350 1300
Wire Wire Line
	8150 1400 8300 1400
$Comp
L Device:L L1
U 1 1 5CD61B46
P 1850 2100
F 0 "L1" V 2040 2100 50  0000 C CNN
F 1 "10uH" V 1949 2100 50  0000 C CNN
F 2 "Inductor_SMD:L_Coilcraft_XxL4040" H 1850 2100 50  0001 C CNN
F 3 "~" H 1850 2100 50  0001 C CNN
	1    1850 2100
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C2
U 1 1 5CD61C94
P 1650 2850
F 0 "C2" H 1738 2896 50  0000 L CNN
F 1 "10uF" H 1738 2805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1650 2850 50  0001 C CNN
F 3 "~" H 1650 2850 50  0001 C CNN
	1    1650 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5CD61CDE
P 3300 2550
F 0 "C3" H 3388 2596 50  0000 L CNN
F 1 "10uF" H 3388 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3300 2550 50  0001 C CNN
F 3 "~" H 3300 2550 50  0001 C CNN
	1    3300 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5CD6223B
P 2600 3200
F 0 "#PWR03" H 2600 2950 50  0001 C CNN
F 1 "GND" H 2605 3027 50  0000 C CNN
F 2 "" H 2600 3200 50  0001 C CNN
F 3 "" H 2600 3200 50  0001 C CNN
	1    2600 3200
	1    0    0    -1  
$EndComp
Connection ~ 2600 3050
Wire Wire Line
	2600 3050 2600 3200
Wire Wire Line
	1650 2100 1700 2100
Connection ~ 1650 2100
Text GLabel 2200 800  0    50   Input ~ 0
VBAT
Text GLabel 1050 2100 0    50   Input ~ 0
VBAT
Text GLabel 4400 2100 2    50   Output ~ 0
3V3
Text GLabel 1300 5850 0    50   Input ~ 0
ADC
Wire Wire Line
	1300 5850 1400 5850
Text GLabel 5950 900  2    50   Output ~ 0
ADC
$Comp
L Device:R R1
U 1 1 5CE089C6
P 5100 900
F 0 "R1" V 4893 900 50  0000 C CNN
F 1 "1M" V 4984 900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5030 900 50  0001 C CNN
F 3 "~" H 5100 900 50  0001 C CNN
	1    5100 900 
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5CE08A65
P 5400 1250
F 0 "R2" H 5330 1204 50  0000 R CNN
F 1 "330K" H 5330 1295 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5330 1250 50  0001 C CNN
F 3 "~" H 5400 1250 50  0001 C CNN
	1    5400 1250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5CE08AFD
P 5400 1500
F 0 "#PWR04" H 5400 1250 50  0001 C CNN
F 1 "GND" H 5405 1327 50  0000 C CNN
F 2 "" H 5400 1500 50  0001 C CNN
F 3 "" H 5400 1500 50  0001 C CNN
	1    5400 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 900  4950 900 
Wire Wire Line
	5250 900  5400 900 
Wire Wire Line
	5400 900  5400 1100
Connection ~ 5400 900 
Wire Wire Line
	5400 1400 5400 1450
Text Notes 4750 1150 0    50   ~ 0
divider:\n4.03v -> 1.0v
Text GLabel 4750 900  0    50   Input ~ 0
VBAT
$Comp
L Device:C_Small C6
U 1 1 5CE67CB7
P 5750 1250
F 0 "C6" H 5842 1296 50  0000 L CNN
F 1 "100nF" H 5842 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5750 1250 50  0001 C CNN
F 3 "~" H 5750 1250 50  0001 C CNN
	1    5750 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 900  5750 900 
Wire Wire Line
	5750 1150 5750 900 
Connection ~ 5750 900 
Wire Wire Line
	5750 900  5950 900 
Wire Wire Line
	5750 1350 5750 1450
Wire Wire Line
	5750 1450 5400 1450
Connection ~ 5400 1450
Wire Wire Line
	5400 1450 5400 1500
$Comp
L Device:C C4
U 1 1 5D095DCD
P 3700 2550
F 0 "C4" H 3788 2596 50  0000 L CNN
F 1 "10uF" H 3788 2505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3700 2550 50  0001 C CNN
F 3 "~" H 3700 2550 50  0001 C CNN
	1    3700 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 3050 1650 3000
Text Notes 3350 3450 0    50   ~ 0
Duplicate 10uF caps\nto reduce ESD, as per\ndatasheet. Larger bulk\ncapacitor for 1xAA.
Wire Wire Line
	2600 2550 2600 3050
Wire Wire Line
	2000 2100 2100 2100
Wire Wire Line
	1650 3050 2600 3050
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5D26FFDF
P 2600 900
F 0 "J1" H 2680 892 50  0000 L CNN
F 1 "POWER" H 2680 801 50  0000 L CNN
F 2 "CPB:Socket_Strip_Straight_1x02_Oval_Pitch2.54mm" H 2600 900 50  0001 C CNN
F 3 "~" H 2600 900 50  0001 C CNN
	1    2600 900 
	1    0    0    1   
$EndComp
Wire Wire Line
	3700 2400 3700 2100
Wire Wire Line
	3150 2100 3300 2100
Connection ~ 3700 2100
Wire Wire Line
	3700 2100 4100 2100
Wire Wire Line
	3300 2400 3300 2100
Connection ~ 3300 2100
Wire Wire Line
	3300 2100 3700 2100
Wire Wire Line
	2600 3050 3300 3050
Wire Wire Line
	3700 3050 3700 2700
Connection ~ 3700 3050
Wire Wire Line
	3700 3050 4100 3050
Wire Wire Line
	3300 3050 3300 2700
Connection ~ 3300 3050
Wire Wire Line
	3300 3050 3700 3050
$Comp
L power:GND #PWR01
U 1 1 5DBBA683
P 2050 900
F 0 "#PWR01" H 2050 650 50  0001 C CNN
F 1 "GND" H 2055 727 50  0000 C CNN
F 2 "" H 2050 900 50  0001 C CNN
F 3 "" H 2050 900 50  0001 C CNN
	1    2050 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 1700 7350 1700
Text GLabel 6950 3050 0    50   Input ~ 0
GPIO16
Text GLabel 3350 6150 2    50   Input ~ 0
GPIO0
Wire Wire Line
	3200 6150 3350 6150
NoConn ~ 1400 6150
NoConn ~ 3200 5750
NoConn ~ 3200 5850
NoConn ~ 3200 6250
$Comp
L Sensor_Temperature:DS18B20 U3
U 1 1 5E7C7D45
P 5900 5150
F 0 "U3" H 5670 5196 50  0000 R CNN
F 1 "DS18B20" H 5670 5105 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline_Wide" H 4900 4900 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf" H 5750 5400 50  0001 C CNN
	1    5900 5150
	1    0    0    -1  
$EndComp
Text GLabel 6450 5150 2    50   Input ~ 0
DQ
$Comp
L power:GND #PWR05
U 1 1 5E7C7E1D
P 5900 5550
F 0 "#PWR05" H 5900 5300 50  0001 C CNN
F 1 "GND" H 5905 5377 50  0000 C CNN
F 2 "" H 5900 5550 50  0001 C CNN
F 3 "" H 5900 5550 50  0001 C CNN
	1    5900 5550
	1    0    0    -1  
$EndComp
Text GLabel 5900 4700 1    50   Input ~ 0
3V3
Wire Wire Line
	5900 4700 5900 4800
Wire Wire Line
	6200 5150 6300 5150
Wire Wire Line
	5900 5450 5900 5550
$Comp
L Device:R_Small R3
U 1 1 5E7CEF88
P 6300 4900
F 0 "R3" V 6500 4900 50  0000 C CNN
F 1 "4K7" V 6400 4900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6300 4900 50  0001 C CNN
F 3 "~" H 6300 4900 50  0001 C CNN
	1    6300 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 4800 6300 4800
Connection ~ 5900 4800
Wire Wire Line
	5900 4800 5900 4850
Wire Wire Line
	6300 5000 6300 5150
Connection ~ 6300 5150
Wire Wire Line
	6300 5150 6450 5150
Connection ~ 7750 1700
Connection ~ 4100 2100
Wire Wire Line
	1650 2100 1650 2700
Wire Wire Line
	8300 1400 8300 3350
Wire Wire Line
	2200 800  2400 800 
Wire Wire Line
	2400 900  2050 900 
Wire Wire Line
	4100 2100 4400 2100
Wire Wire Line
	6950 3050 7150 3050
Wire Wire Line
	7150 3050 7150 2900
Wire Wire Line
	7150 2700 7150 2500
Wire Wire Line
	7750 2500 7550 2500
Wire Wire Line
	7750 1700 7750 2500
Wire Wire Line
	7250 2500 7150 2500
Connection ~ 7150 2500
Text GLabel 3350 6050 2    50   BiDi ~ 0
SDA
Text GLabel 3350 5950 2    50   BiDi ~ 0
SCL
Wire Wire Line
	3200 5950 3350 5950
Wire Wire Line
	3350 6050 3200 6050
Text GLabel 8000 4750 0    50   Input ~ 0
3V3
Text GLabel 8000 4950 0    50   BiDi ~ 0
SCL
Text GLabel 8000 5050 0    50   BiDi ~ 0
SDA
$Comp
L power:GND #PWR06
U 1 1 5E852412
P 7650 4850
F 0 "#PWR06" H 7650 4600 50  0001 C CNN
F 1 "GND" H 7655 4677 50  0000 C CNN
F 2 "" H 7650 4850 50  0001 C CNN
F 3 "" H 7650 4850 50  0001 C CNN
	1    7650 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 4750 8250 4750
Wire Wire Line
	8250 4850 7650 4850
Wire Wire Line
	8000 4950 8250 4950
Wire Wire Line
	8250 5050 8000 5050
$Comp
L Connector_Generic:Conn_01x08 J2
U 1 1 5E85991F
P 8450 5050
F 0 "J2" H 8529 5042 50  0000 L CNN
F 1 "MPU-6050" H 8529 4951 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 8450 5050 50  0001 C CNN
F 3 "~" H 8450 5050 50  0001 C CNN
	1    8450 5050
	1    0    0    -1  
$EndComp
Text GLabel 8000 5450 0    50   Input ~ 0
GPIO15
Wire Wire Line
	8000 5450 8250 5450
NoConn ~ 8250 5150
NoConn ~ 8250 5250
NoConn ~ 8250 5350
Text Notes 7700 5900 0    50   ~ 0
Assume the MPU-6050\nmodule has appropriate\nI2C pullups builtin, which\nseems to be typical.
Wire Wire Line
	1050 2100 1650 2100
$Comp
L Device:CP1_Small C5
U 1 1 5E8676BA
P 4100 2550
F 0 "C5" H 4191 2596 50  0000 L CNN
F 1 "100uF" H 4191 2505 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3528-12_Kemet-T_Pad1.50x2.35mm_HandSolder" H 4100 2550 50  0001 C CNN
F 3 "~" H 4100 2550 50  0001 C CNN
	1    4100 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 2100 4100 2450
Wire Wire Line
	4100 2650 4100 3050
$Comp
L Holtek:HT7733SA U2
U 1 1 5E86B919
P 2600 2100
F 0 "U2" H 2625 2365 50  0000 C CNN
F 1 "HT7733SA" H 2625 2274 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 2800 1800 50  0001 C CNN
F 3 "" H 2600 2100 50  0001 C CNN
	1    2600 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1700 3300 2100
Wire Wire Line
	2050 1700 2050 2250
Wire Wire Line
	2050 2250 2100 2250
Wire Wire Line
	2050 1700 3300 1700
Wire Wire Line
	1300 6250 1400 6250
Wire Wire Line
	1300 6350 1400 6350
$Comp
L Device:Jumper JP1
U 1 1 5E859075
P 1300 6650
F 0 "JP1" V 1346 6562 50  0000 R CNN
F 1 "Cal" V 1255 6562 50  0000 R CNN
F 2 "CPB:Socket_Strip_Straight_1x02_Oval_Pitch2.54mm" H 1300 6650 50  0001 C CNN
F 3 "~" H 1300 6650 50  0001 C CNN
	1    1300 6650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1300 6950 1300 7000
Connection ~ 1300 7000
Wire Wire Line
	1300 7000 2300 7000
$EndSCHEMATC
