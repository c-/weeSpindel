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
Text Notes 750  5450 0    50   ~ 0
RST: newer ESP-12F has\ninternal 12k pullup. 47K\nshould allow versions\nwith and without to boot.
$Comp
L Device:R R5
U 1 1 5CA494A3
P 2100 5300
F 0 "R5" V 2200 5300 50  0000 C CNN
F 1 "47K" V 2300 5300 50  0000 C CNN
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
F 1 "100nF" H 2292 6755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2200 6800 50  0001 C CNN
F 3 "~" H 2200 6800 50  0001 C CNN
	1    2200 6800
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5CD61C94
P 3150 2950
F 0 "C2" H 3238 2996 50  0000 L CNN
F 1 "10uF" H 3238 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3150 2950 50  0001 C CNN
F 3 "~" H 3150 2950 50  0001 C CNN
	1    3150 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5CD61CDE
P 4900 2950
F 0 "C3" H 4988 2996 50  0000 L CNN
F 1 "22uF" H 4988 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4900 2950 50  0001 C CNN
F 3 "~" H 4900 2950 50  0001 C CNN
	1    4900 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5CD6223B
P 4200 3600
F 0 "#PWR03" H 4200 3350 50  0001 C CNN
F 1 "GND" H 4205 3427 50  0000 C CNN
F 2 "" H 4200 3600 50  0001 C CNN
F 3 "" H 4200 3600 50  0001 C CNN
	1    4200 3600
	1    0    0    -1  
$EndComp
Connection ~ 4200 3450
Wire Wire Line
	4200 3450 4200 3600
Text GLabel 5100 2500 2    50   Output ~ 0
3V3
Text GLabel 2700 5750 0    50   Input ~ 0
ADC
Wire Wire Line
	2700 5750 2800 5750
Text GLabel 5150 1300 2    50   Output ~ 0
ADC
$Comp
L Device:R R1
U 1 1 5CE089C6
P 4300 1300
F 0 "R1" V 4093 1300 50  0000 C CNN
F 1 "1M" V 4184 1300 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4230 1300 50  0001 C CNN
F 3 "~" H 4300 1300 50  0001 C CNN
	1    4300 1300
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5CE08A65
P 4600 1650
F 0 "R2" H 4530 1604 50  0000 R CNN
F 1 "330K" H 4530 1695 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4530 1650 50  0001 C CNN
F 3 "~" H 4600 1650 50  0001 C CNN
	1    4600 1650
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5CE08AFD
P 4950 1900
F 0 "#PWR04" H 4950 1650 50  0001 C CNN
F 1 "GND" H 4955 1727 50  0000 C CNN
F 2 "" H 4950 1900 50  0001 C CNN
F 3 "" H 4950 1900 50  0001 C CNN
	1    4950 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1300 4600 1300
Wire Wire Line
	4600 1300 4600 1500
Connection ~ 4600 1300
Wire Wire Line
	4600 1800 4600 1850
Text Notes 3950 1550 0    50   ~ 0
divider:\n4.03v -> 1.0v
$Comp
L Device:C_Small C6
U 1 1 5CE67CB7
P 4950 1650
F 0 "C6" H 5042 1696 50  0000 L CNN
F 1 "100nF" H 5042 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4950 1650 50  0001 C CNN
F 3 "~" H 4950 1650 50  0001 C CNN
	1    4950 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1300 4950 1300
Wire Wire Line
	4950 1550 4950 1300
Connection ~ 4950 1300
Wire Wire Line
	4950 1300 5150 1300
Wire Wire Line
	4950 1750 4950 1850
Wire Wire Line
	4950 1850 4600 1850
Wire Wire Line
	4900 2800 4900 2500
Wire Wire Line
	4200 3450 4900 3450
Wire Wire Line
	4900 3450 4900 3100
Wire Wire Line
	4600 6050 4700 6050
NoConn ~ 4600 6150
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
L Device:Battery_Cell BT1
U 1 1 5E862E9B
P 1800 2000
F 0 "BT1" V 1545 2050 50  0000 C CNN
F 1 "Li 3.7v" V 1636 2050 50  0000 C CNN
F 2 "Connector_JST:JST_PH_S2B-PH-K_1x02_P2.00mm_Horizontal" V 1800 2060 50  0001 C CNN
F 3 "~" V 1800 2060 50  0001 C CNN
	1    1800 2000
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_SPST SW1
U 1 1 5E86C4A2
P 2800 1300
F 0 "SW1" H 2800 1075 50  0000 C CNN
F 1 "POWER" H 2800 1166 50  0000 C CNN
F 2 "CPB:SPST Switch 9mmx3.5mm C223839" H 2800 1300 50  0001 C CNN
F 3 "" H 2800 1300 50  0001 C CNN
	1    2800 1300
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
F 1 "10K" V 4900 6650 50  0000 C CNN
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
	4950 1850 4950 1900
Connection ~ 4950 1850
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
$Comp
L Regulator_Linear:MCP1700-3302E_SOT23 U3
U 1 1 5E95A681
P 4200 2500
F 0 "U3" H 4200 2742 50  0000 C CNN
F 1 "HT7233" H 4200 2651 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 4200 2725 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826C.pdf" H 4200 2500 50  0001 C CNN
	1    4200 2500
	1    0    0    -1  
$EndComp
NoConn ~ 4600 5650
NoConn ~ 4600 5750
Wire Wire Line
	3150 2800 3150 2500
Connection ~ 3150 2500
Wire Wire Line
	3150 3100 3150 3450
Wire Wire Line
	3150 3450 4200 3450
Wire Wire Line
	3150 1300 4150 1300
Wire Wire Line
	3150 2500 3900 2500
Wire Wire Line
	4500 2500 4900 2500
Wire Wire Line
	4200 2800 4200 3450
$Comp
L Blue~Rocket:BRCL3130MC U2
U 1 1 5EA658A5
P 1800 2850
F 0 "U2" H 1800 3125 50  0000 C CNN
F 1 "BRCL3130MC" H 1800 3034 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 1850 2500 50  0001 C CNN
F 3 "https://datasheet.lcsc.com/szlcsc/1912111437_Foshan-Blue-Rocket-Elec-BRCL3130MC_C328561.pdf" H 1550 2600 50  0001 C CNN
	1    1800 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	2200 2800 2350 2800
$Comp
L Device:R_Small R8
U 1 1 5EA70F89
P 2350 2250
F 0 "R8" H 2409 2296 50  0000 L CNN
F 1 "100R" H 2409 2205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2350 2250 50  0001 C CNN
F 3 "~" H 2350 2250 50  0001 C CNN
	1    2350 2250
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5EA793F5
P 1850 2450
F 0 "C4" V 2079 2450 50  0000 C CNN
F 1 "100nF" V 1988 2450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1850 2450 50  0001 C CNN
F 3 "~" H 1850 2450 50  0001 C CNN
	1    1850 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	1400 2850 1350 2850
Wire Wire Line
	1350 2850 1350 2450
Wire Wire Line
	2000 2000 2350 2000
Wire Wire Line
	2350 2000 2350 2150
Wire Wire Line
	2350 2350 2350 2450
Wire Wire Line
	1950 2450 2350 2450
Connection ~ 2350 2450
Wire Wire Line
	2350 2450 2350 2800
Wire Wire Line
	1750 2450 1350 2450
Wire Wire Line
	1350 2000 1700 2000
Wire Wire Line
	1350 2450 1350 2000
Connection ~ 1350 2450
Wire Wire Line
	3150 1300 3150 2500
Wire Wire Line
	2200 2900 2350 2900
Wire Wire Line
	2600 1300 2350 1300
Wire Wire Line
	2350 2900 2350 3450
Wire Wire Line
	2350 3450 3150 3450
Connection ~ 3150 3450
Wire Wire Line
	3000 1300 3150 1300
Connection ~ 3150 1300
Connection ~ 2350 2000
Wire Wire Line
	2350 1300 2350 2000
Wire Wire Line
	5100 2500 4900 2500
Connection ~ 4900 2500
$EndSCHEMATC
