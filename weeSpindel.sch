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
$Comp
L Device:C_Small C1
U 1 1 5B0BCD0C
P 4050 4950
F 0 "C1" H 4142 4996 50  0000 L CNN
F 1 "100nF" H 4142 4905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4050 4950 50  0001 C CNN
F 3 "~" H 4050 4950 50  0001 C CNN
	1    4050 4950
	0    1    1    0   
$EndComp
Text GLabel 3700 4750 1    50   Input ~ 0
3V3
$Comp
L power:GND #PWR02
U 1 1 5B0BCD6E
P 3700 7100
F 0 "#PWR02" H 3700 6850 50  0001 C CNN
F 1 "GND" H 3705 6927 50  0000 C CNN
F 2 "" H 3700 7100 50  0001 C CNN
F 3 "" H 3700 7100 50  0001 C CNN
	1    3700 7100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5B0C7A88
P 2350 5300
F 0 "R6" V 2143 5300 50  0000 C CNN
F 1 "10K" V 2234 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2280 5300 50  0001 C CNN
F 3 "~" H 2350 5300 50  0001 C CNN
	1    2350 5300
	-1   0    0    1   
$EndComp
Wire Wire Line
	4600 6250 4700 6250
Wire Wire Line
	3700 6850 3700 6900
Connection ~ 3700 6900
Wire Wire Line
	3700 4750 3700 4850
Connection ~ 3700 4950
Wire Wire Line
	3700 4950 3700 5050
$Comp
L ESP8266:ESP-12 U1
U 1 1 5B0BCB07
P 3700 5950
F 0 "U1" H 3550 6600 50  0000 C CNN
F 1 "ESP-12E" H 3300 6600 50  0000 C CNN
F 2 "CPB:ESP-12Emin" H 3700 6200 50  0001 C CNN
F 3 "http://wiki.ai-thinker.com/_media/esp8266/docs/aithinker_esp_12f_datasheet_en.pdf" H 3350 6250 50  0001 C CNN
	1    3700 5950
	1    0    0    -1  
$EndComp
Text Notes 850  5450 0    50   ~ 0
RST: newer ESP-12F has\ninternal 12k pullup. 47K\nshould allow versions\nwith and without to boot.
$Comp
L Device:R R5
U 1 1 5CA494A3
P 2100 5300
F 0 "R5" V 2200 5300 50  0000 C CNN
F 1 "47K" V 2250 5300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2030 5300 50  0001 C CNN
F 3 "~" H 2100 5300 50  0001 C CNN
	1    2100 5300
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5CC4D333
P 2200 6800
F 0 "C7" H 2292 6846 50  0000 L CNN
F 1 "2.2uF" H 2292 6755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2200 6800 50  0001 C CNN
F 3 "~" H 2200 6800 50  0001 C CNN
	1    2200 6800
	-1   0    0    1   
$EndComp
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
Text GLabel 4400 2100 2    50   Output ~ 0
3V3
Text GLabel 2700 5750 0    50   Input ~ 0
ADC
Wire Wire Line
	2700 5750 2800 5750
Text GLabel 3550 900  2    50   Output ~ 0
ADC
$Comp
L Device:R R1
U 1 1 5CE089C6
P 2700 900
F 0 "R1" V 2493 900 50  0000 C CNN
F 1 "1M" V 2584 900 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2630 900 50  0001 C CNN
F 3 "~" H 2700 900 50  0001 C CNN
	1    2700 900 
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5CE08A65
P 3000 1250
F 0 "R2" H 2930 1204 50  0000 R CNN
F 1 "330K" H 2930 1295 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2930 1250 50  0001 C CNN
F 3 "~" H 3000 1250 50  0001 C CNN
	1    3000 1250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5CE08AFD
P 3350 1500
F 0 "#PWR04" H 3350 1250 50  0001 C CNN
F 1 "GND" H 3355 1327 50  0000 C CNN
F 2 "" H 3350 1500 50  0001 C CNN
F 3 "" H 3350 1500 50  0001 C CNN
	1    3350 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 900  3000 900 
Wire Wire Line
	3000 900  3000 1100
Connection ~ 3000 900 
Wire Wire Line
	3000 1400 3000 1450
Text Notes 2350 1150 0    50   ~ 0
divider:\n4.03v -> 1.0v
$Comp
L Device:C_Small C6
U 1 1 5CE67CB7
P 3350 1250
F 0 "C6" H 3442 1296 50  0000 L CNN
F 1 "100nF" H 3442 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3350 1250 50  0001 C CNN
F 3 "~" H 3350 1250 50  0001 C CNN
	1    3350 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 900  3350 900 
Wire Wire Line
	3350 1150 3350 900 
Connection ~ 3350 900 
Wire Wire Line
	3350 900  3550 900 
Wire Wire Line
	3350 1350 3350 1450
Wire Wire Line
	3350 1450 3000 1450
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
Duplicate 10uF caps\nto reduce ESD, as per\ndatasheet. Larger bulk\ncapacitor for 1x NiMH.
Wire Wire Line
	2600 2550 2600 3050
Wire Wire Line
	2000 2100 2100 2100
Wire Wire Line
	1650 3050 2600 3050
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
P 700 1150
F 0 "#PWR01" H 700 900 50  0001 C CNN
F 1 "GND" H 705 977 50  0000 C CNN
F 2 "" H 700 1150 50  0001 C CNN
F 3 "" H 700 1150 50  0001 C CNN
	1    700  1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 6050 4700 6050
NoConn ~ 4600 5650
NoConn ~ 4600 5750
NoConn ~ 4600 6150
Connection ~ 4100 2100
Wire Wire Line
	1650 2100 1650 2700
Wire Wire Line
	4100 2100 4400 2100
Text GLabel 4750 5950 2    50   BiDi ~ 0
SDA
Text GLabel 4750 5850 2    50   BiDi ~ 0
SCL
Wire Wire Line
	4600 5850 4750 5850
Wire Wire Line
	4750 5950 4600 5950
Text GLabel 7100 4450 0    50   Input ~ 0
3V3
Text GLabel 7100 4850 0    50   BiDi ~ 0
SCL
Text GLabel 7100 4950 0    50   BiDi ~ 0
SDA
$Comp
L power:GND #PWR06
U 1 1 5E852412
P 8050 5050
F 0 "#PWR06" H 8050 4800 50  0001 C CNN
F 1 "GND" H 8055 4877 50  0000 C CNN
F 2 "" H 8050 5050 50  0001 C CNN
F 3 "" H 8050 5050 50  0001 C CNN
	1    8050 5050
	1    0    0    -1  
$EndComp
Text Notes 6950 5550 0    50   ~ 0
NOTE: the MPU-6050\nmodule usually has\nI2C pullups builtin, so we're\nleaving them out here.
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
	2050 2250 2100 2250
$Comp
L Device:Battery_Cell BT1
U 1 1 5E862E9B
P 850 900
F 0 "BT1" V 595 950 50  0000 C CNN
F 1 "NiMH" V 686 950 50  0000 C CNN
F 2 "CPB:Battery_Springs_AAA" V 850 960 50  0001 C CNN
F 3 "~" V 850 960 50  0001 C CNN
	1    850  900 
	0    1    1    0   
$EndComp
Wire Wire Line
	1050 900  1200 900 
Wire Wire Line
	750  900  700  900 
Wire Wire Line
	700  900  700  1150
$Comp
L Switch:SW_SPST SW1
U 1 1 5E86C4A2
P 1400 900
F 0 "SW1" H 1400 675 50  0000 C CNN
F 1 "POWER" H 1400 766 50  0000 C CNN
F 2 "CPB:SPST Switch 9mmx3.5mm C223839" H 1400 900 50  0001 C CNN
F 3 "" H 1400 900 50  0001 C CNN
	1    1400 900 
	-1   0    0    1   
$EndComp
NoConn ~ 2800 6250
Wire Wire Line
	2800 6050 2450 6050
Wire Wire Line
	2450 6900 3700 6900
NoConn ~ 2800 6150
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5E97FF96
P 8300 4750
F 0 "J1" H 8380 4742 50  0000 L CNN
F 1 "MPU-6050" H 8380 4651 50  0000 L CNN
F 2 "CPB:MPU-6050 Breakout I2C" H 8300 4750 50  0001 C CNN
F 3 "~" H 8300 4750 50  0001 C CNN
	1    8300 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 4750 8050 4750
Wire Wire Line
	8050 4750 8050 5050
Wire Wire Line
	8050 4450 8050 4650
Wire Wire Line
	8050 4650 8100 4650
$Comp
L power:GND #PWR05
U 1 1 5E99E221
P 4350 5050
F 0 "#PWR05" H 4350 4800 50  0001 C CNN
F 1 "GND" H 4355 4877 50  0000 C CNN
F 2 "" H 4350 5050 50  0001 C CNN
F 3 "" H 4350 5050 50  0001 C CNN
	1    4350 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 4950 3950 4950
Wire Wire Line
	4150 4950 4350 4950
Wire Wire Line
	4350 4950 4350 5050
$Comp
L Device:R R7
U 1 1 5E9A2F33
P 4700 6650
F 0 "R7" V 4800 6650 50  0000 C CNN
F 1 "10K" V 4850 6650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4630 6650 50  0001 C CNN
F 3 "~" H 4700 6650 50  0001 C CNN
	1    4700 6650
	-1   0    0    1   
$EndComp
Wire Wire Line
	4700 6250 4700 6500
Wire Wire Line
	4700 6800 4700 6900
Wire Wire Line
	4700 6900 3700 6900
$Comp
L Device:R R4
U 1 1 5E9A8366
P 4700 5250
F 0 "R4" V 4493 5250 50  0000 C CNN
F 1 "10K" V 4584 5250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4630 5250 50  0001 C CNN
F 3 "~" H 4700 5250 50  0001 C CNN
	1    4700 5250
	-1   0    0    1   
$EndComp
Wire Wire Line
	3700 4850 4700 4850
Wire Wire Line
	4700 4850 4700 5100
Wire Wire Line
	3700 6900 3700 7100
Connection ~ 3700 4850
Wire Wire Line
	3700 4850 3700 4950
Wire Wire Line
	4700 5400 4700 6050
$Comp
L Device:R_Small R3
U 1 1 5E9ADDC8
P 2100 5800
F 0 "R3" V 2300 5800 50  0000 C CNN
F 1 "1K" V 2200 5800 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 2100 5800 50  0001 C CNN
F 3 "~" H 2100 5800 50  0001 C CNN
	1    2100 5800
	-1   0    0    1   
$EndComp
Wire Wire Line
	2100 4950 2100 5150
Wire Wire Line
	2100 4950 2350 4950
Wire Wire Line
	2100 5650 2100 5700
Wire Wire Line
	2100 5650 2800 5650
Wire Wire Line
	2100 5650 2100 5450
Connection ~ 2100 5650
Wire Wire Line
	2100 5900 2100 5950
Wire Wire Line
	2100 5950 2800 5950
Wire Wire Line
	2350 5850 2350 5450
Wire Wire Line
	2350 5850 2800 5850
Wire Wire Line
	2350 5850 2200 5850
Wire Wire Line
	2200 5850 2200 6700
Connection ~ 2350 5850
Wire Wire Line
	2200 6900 2450 6900
Connection ~ 2450 6900
Wire Wire Line
	2350 5150 2350 4950
Connection ~ 2350 4950
Wire Wire Line
	2350 4950 3700 4950
Wire Wire Line
	3300 2100 3300 1750
Wire Wire Line
	3300 1750 2050 1750
Wire Wire Line
	2050 1750 2050 2250
Wire Wire Line
	1600 900  1650 900 
Wire Wire Line
	1650 2100 1650 900 
Connection ~ 1650 2100
Connection ~ 1650 900 
Wire Wire Line
	1650 900  2550 900 
Wire Wire Line
	3350 1450 3350 1500
Connection ~ 3350 1450
Wire Wire Line
	7100 4950 8100 4950
Wire Wire Line
	7100 4450 8050 4450
Wire Wire Line
	7100 4850 8100 4850
Wire Wire Line
	2450 6800 2450 6900
Wire Wire Line
	2450 6050 2450 6200
$Comp
L Device:Jumper JP1
U 1 1 5E859075
P 2450 6500
F 0 "JP1" V 2496 6412 50  0000 R CNN
F 1 "Cal" V 2405 6412 50  0000 R CNN
F 2 "CPB:Socket_Strip_Straight_1x02_Oval_Pitch2.54mm" H 2450 6500 50  0001 C CNN
F 3 "~" H 2450 6500 50  0001 C CNN
	1    2450 6500
	0    -1   -1   0   
$EndComp
Text Notes 800  7000 0    50   ~ 0
10K/2.2uF RC should give\na startup delay of maybe\n25ms. The boost needs extra\ntime to stabilize.
$EndSCHEMATC
