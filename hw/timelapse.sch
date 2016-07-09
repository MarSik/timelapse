EESchema Schematic File Version 2
LIBS:timelapse-rescue
LIBS:74xx
LIBS:boards
LIBS:connectors
LIBS:cpu
LIBS:discrete
LIBS:drivers
LIBS:ic
LIBS:mspower
LIBS:opto
LIBS:passives
LIBS:power
LIBS:simple
LIBS:transistors
LIBS:timelapse-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Camera timelapse timer"
Date "2016-06-19"
Rev "A"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 10500 2850 2    60   ~ 0
Camera trigger cable - Olympus e-510\nRed - Ground\nWhite - Focus current 50uA\nYellow - Shutter current 50uA\nSignal trandfer ratio > 50%
$Comp
L Cosmo_1010-817 U102
U 1 1 575412C5
P 9100 3350
F 0 "U102" H 8900 3550 50  0000 L CNN
F 1 "Cosmo_1010-817" H 8900 3650 50  0000 L CNN
F 2 "th-npth:DIP-4_W300mil_LongPads" H 8900 3150 50  0001 L CIN
F 3 "" H 9100 3250 50  0000 L CNN
	1    9100 3350
	1    0    0    -1  
$EndComp
$Comp
L Cosmo_1010-817 U103
U 1 1 57541313
P 9100 3800
F 0 "U103" H 8900 4000 50  0000 L CNN
F 1 "Cosmo_1010-817" H 8900 3600 50  0000 L CNN
F 2 "th-npth:DIP-4_W300mil_LongPads" H 8900 3600 50  0001 L CIN
F 3 "" H 9100 3700 50  0000 L CNN
	1    9100 3800
	1    0    0    -1  
$EndComp
$Comp
L MSP430G2553-RESCUE-timelapse U101
U 1 1 57541397
P 4750 6650
F 0 "U101" H 4250 7550 60  0000 C CNN
F 1 "MSP430G2553" H 4750 7350 60  0000 C CNN
F 2 "th-npth:DIP-20_W300mil_LongPads" H 5050 7250 60  0001 C CNN
F 3 "" H 5050 7250 60  0000 C CNN
	1    4750 6650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X03 P101
U 1 1 575416E6
P 1650 6750
F 0 "P101" H 1650 6950 50  0000 C CNN
F 1 "CONN_01X03" V 1750 6750 50  0000 C CNN
F 2 "th-normal:Pin_Header_Angled_1x03" H 1650 6750 60  0001 C CNN
F 3 "" H 1650 6750 60  0000 C CNN
	1    1650 6750
	-1   0    0    1   
$EndComp
$Comp
L LCD_Nokia_5510 LCD101
U 1 1 57541773
P 9600 5550
F 0 "LCD101" H 9900 6100 60  0000 L CNN
F 1 "LCD_Nokia_5510" H 9600 5550 60  0000 C CNN
F 2 "th-normal:LCD_Nokia_5510" H 9350 5550 60  0001 C CNN
F 3 "" H 9350 5550 60  0000 C CNN
	1    9600 5550
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW102
U 1 1 575418B1
P 4800 1200
F 0 "SW102" H 4950 1310 50  0000 C CNN
F 1 "SW_PUSH" H 4800 1120 50  0000 C CNN
F 2 "th-normal:Button_B3F-315X" H 4800 1200 60  0001 C CNN
F 3 "" H 4800 1200 60  0000 C CNN
	1    4800 1200
	-1   0    0    -1  
$EndComp
$Comp
L SW_PUSH SW103
U 1 1 57541904
P 4800 1500
F 0 "SW103" H 4950 1610 50  0000 C CNN
F 1 "SW_PUSH" H 4800 1420 50  0000 C CNN
F 2 "th-normal:Button_B3F-315X" H 4800 1500 60  0001 C CNN
F 3 "" H 4800 1500 60  0000 C CNN
	1    4800 1500
	-1   0    0    -1  
$EndComp
$Comp
L SW_PUSH SW104
U 1 1 5754192F
P 4800 1800
F 0 "SW104" H 4950 1910 50  0000 C CNN
F 1 "SW_PUSH" H 4800 1720 50  0000 C CNN
F 2 "th-normal:Button_B3F-315X" H 4800 1800 60  0001 C CNN
F 3 "" H 4800 1800 60  0000 C CNN
	1    4800 1800
	-1   0    0    -1  
$EndComp
$Comp
L SW_PUSH SW105
U 1 1 57541A7D
P 4800 2100
F 0 "SW105" H 4950 2210 50  0000 C CNN
F 1 "SW_PUSH" H 4800 2020 50  0000 C CNN
F 2 "th-normal:Button_B3F-315X" H 4800 2100 60  0001 C CNN
F 3 "" H 4800 2100 60  0000 C CNN
	1    4800 2100
	-1   0    0    -1  
$EndComp
$Comp
L LED D101
U 1 1 57541C6D
P 8500 1600
F 0 "D101" H 8500 1700 50  0000 C CNN
F 1 "LED" H 8500 1500 50  0000 C CNN
F 2 "th-normal:LED_L-1384_AL-1" H 8500 1600 60  0001 C CNN
F 3 "" H 8500 1600 60  0000 C CNN
	1    8500 1600
	1    0    0    1   
$EndComp
$Comp
L R R106
U 1 1 57541E6B
P 8450 3250
F 0 "R106" V 8350 3250 50  0000 C CNN
F 1 "10k" V 8450 3250 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 8380 3250 30  0001 C CNN
F 3 "" H 8450 3250 30  0000 C CNN
	1    8450 3250
	0    1    1    0   
$EndComp
$Comp
L R R107
U 1 1 57541EC4
P 8450 3700
F 0 "R107" V 8350 3700 50  0000 C CNN
F 1 "10k" V 8450 3700 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 8380 3700 30  0001 C CNN
F 3 "" H 8450 3700 30  0000 C CNN
	1    8450 3700
	0    1    1    0   
$EndComp
$Comp
L C C102
U 1 1 57542170
P 5450 2300
F 0 "C102" V 5500 2050 50  0000 L CNN
F 1 "10n" H 5500 2400 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 5488 2150 30  0001 C CNN
F 3 "" H 5450 2300 60  0000 C CNN
	1    5450 2300
	-1   0    0    -1  
$EndComp
$Comp
L C C103
U 1 1 575421C8
P 5650 2300
F 0 "C103" V 5700 2050 50  0000 L CNN
F 1 "10n" H 5700 2400 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 5688 2150 30  0001 C CNN
F 3 "" H 5650 2300 60  0000 C CNN
	1    5650 2300
	-1   0    0    -1  
$EndComp
$Comp
L C C104
U 1 1 57542216
P 5850 2300
F 0 "C104" V 5900 2050 50  0000 L CNN
F 1 "10n" H 5900 2400 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 5888 2150 30  0001 C CNN
F 3 "" H 5850 2300 60  0000 C CNN
	1    5850 2300
	-1   0    0    -1  
$EndComp
$Comp
L C C105
U 1 1 57542267
P 6050 2300
F 0 "C105" V 6100 2050 50  0000 L CNN
F 1 "10n" H 6100 2400 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 6088 2150 30  0001 C CNN
F 3 "" H 6050 2300 60  0000 C CNN
	1    6050 2300
	-1   0    0    -1  
$EndComp
$Comp
L R R101
U 1 1 57542325
P 5450 1000
F 0 "R101" H 5600 1100 50  0000 C CNN
F 1 "1k" V 5450 1000 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 5380 1000 30  0001 C CNN
F 3 "" H 5450 1000 30  0000 C CNN
	1    5450 1000
	-1   0    0    -1  
$EndComp
$Comp
L R R102
U 1 1 57542387
P 5650 1000
F 0 "R102" H 5650 1150 50  0000 C CNN
F 1 "1k" V 5650 1000 50  0000 C CNN
F 2 "th-npth:Resistor_Horizontal_RM10mm" V 5580 1000 30  0001 C CNN
F 3 "" H 5650 1000 30  0000 C CNN
	1    5650 1000
	-1   0    0    -1  
$EndComp
$Comp
L R R103
U 1 1 575423DF
P 5850 1000
F 0 "R103" H 5850 850 50  0000 C CNN
F 1 "1k" V 5850 1000 50  0000 C CNN
F 2 "th-npth:Resistor_Horizontal_RM10mm" V 5780 1000 30  0001 C CNN
F 3 "" H 5850 1000 30  0000 C CNN
	1    5850 1000
	-1   0    0    -1  
$EndComp
$Comp
L R R104
U 1 1 5754243C
P 6050 1000
F 0 "R104" H 5900 1100 50  0000 C CNN
F 1 "1k" V 6050 1000 50  0000 C CNN
F 2 "th-npth:Resistor_Horizontal_RM10mm" V 5980 1000 30  0001 C CNN
F 3 "" H 6050 1000 30  0000 C CNN
	1    6050 1000
	-1   0    0    -1  
$EndComp
$Comp
L R R108
U 1 1 57542566
P 8900 1600
F 0 "R108" V 8800 1600 50  0000 C CNN
F 1 "1k" V 8900 1600 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 8830 1600 30  0001 C CNN
F 3 "" H 8900 1600 30  0000 C CNN
	1    8900 1600
	0    -1   1    0   
$EndComp
Wire Wire Line
	4050 6650 1850 6650
Wire Wire Line
	4050 6750 1850 6750
Wire Wire Line
	1850 6850 1950 6850
Wire Wire Line
	1950 6850 1950 6950
$Comp
L GND #PWR103
U 1 1 57542AC3
P 1950 6950
F 0 "#PWR103" H 1950 6700 50  0001 C CNN
F 1 "GND" H 1950 6800 50  0000 C CNN
F 2 "" H 1950 6950 60  0000 C CNN
F 3 "" H 1950 6950 60  0000 C CNN
	1    1950 6950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR106
U 1 1 57542B09
P 3900 6400
F 0 "#PWR106" H 3900 6150 50  0001 C CNN
F 1 "GND" H 3900 6250 50  0000 C CNN
F 2 "" H 3900 6400 60  0000 C CNN
F 3 "" H 3900 6400 60  0000 C CNN
	1    3900 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 6400 3900 6350
Wire Wire Line
	3450 6350 4050 6350
$Comp
L GND #PWR108
U 1 1 57543D4E
P 4300 2250
F 0 "#PWR108" H 4300 2000 50  0001 C CNN
F 1 "GND" H 4300 2100 50  0000 C CNN
F 2 "" H 4300 2250 60  0000 C CNN
F 3 "" H 4300 2250 60  0000 C CNN
	1    4300 2250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4300 1200 4300 2250
Wire Wire Line
	4300 1200 4500 1200
Wire Wire Line
	4500 1500 4300 1500
Connection ~ 4300 1500
Wire Wire Line
	4500 1800 4300 1800
Connection ~ 4300 1800
Wire Wire Line
	4500 2100 4300 2100
Connection ~ 4300 2100
Wire Wire Line
	5450 1150 5450 2150
Wire Wire Line
	5650 1150 5650 2150
Wire Wire Line
	5850 1150 5850 2150
Wire Wire Line
	6050 1150 6050 2150
Wire Wire Line
	5100 1200 6650 1200
Connection ~ 5450 1200
Wire Wire Line
	5100 1500 6650 1500
Connection ~ 5650 1500
Wire Wire Line
	5100 1800 6650 1800
Connection ~ 5850 1800
Wire Wire Line
	5100 2100 6650 2100
Connection ~ 6050 2100
$Comp
L GND #PWR110
U 1 1 57543FAF
P 5450 2550
F 0 "#PWR110" H 5450 2300 50  0001 C CNN
F 1 "GND" H 5450 2400 50  0000 C CNN
F 2 "" H 5450 2550 60  0000 C CNN
F 3 "" H 5450 2550 60  0000 C CNN
	1    5450 2550
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR111
U 1 1 5754400F
P 5650 2550
F 0 "#PWR111" H 5650 2300 50  0001 C CNN
F 1 "GND" H 5650 2400 50  0000 C CNN
F 2 "" H 5650 2550 60  0000 C CNN
F 3 "" H 5650 2550 60  0000 C CNN
	1    5650 2550
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR112
U 1 1 57544068
P 5850 2550
F 0 "#PWR112" H 5850 2300 50  0001 C CNN
F 1 "GND" H 5850 2400 50  0000 C CNN
F 2 "" H 5850 2550 60  0000 C CNN
F 3 "" H 5850 2550 60  0000 C CNN
	1    5850 2550
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR114
U 1 1 575440C1
P 6050 2550
F 0 "#PWR114" H 6050 2300 50  0001 C CNN
F 1 "GND" H 6050 2400 50  0000 C CNN
F 2 "" H 6050 2550 60  0000 C CNN
F 3 "" H 6050 2550 60  0000 C CNN
	1    6050 2550
	-1   0    0    -1  
$EndComp
$Comp
L +3V3 #PWR113
U 1 1 575444B7
P 6050 750
F 0 "#PWR113" H 6050 600 50  0001 C CNN
F 1 "+3V3" H 6050 890 50  0000 C CNN
F 2 "" H 6050 750 60  0000 C CNN
F 3 "" H 6050 750 60  0000 C CNN
	1    6050 750 
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6050 750  6050 850 
Wire Wire Line
	5450 800  6050 800 
Wire Wire Line
	5850 800  5850 850 
Connection ~ 6050 800 
Wire Wire Line
	5650 800  5650 850 
Connection ~ 5850 800 
Wire Wire Line
	5450 800  5450 850 
Connection ~ 5650 800 
$Comp
L +3V3 #PWR118
U 1 1 5754488D
P 9300 1500
F 0 "#PWR118" H 9300 1350 50  0001 C CNN
F 1 "+3V3" H 9300 1640 50  0000 C CNN
F 2 "" H 9300 1500 60  0000 C CNN
F 3 "" H 9300 1500 60  0000 C CNN
	1    9300 1500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9300 1600 9050 1600
Wire Wire Line
	8750 1600 8700 1600
$Comp
L C C101
U 1 1 57544BA4
P 3450 6150
F 0 "C101" H 3200 6050 50  0000 L CNN
F 1 "100n" H 3475 6050 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 3488 6000 30  0001 C CNN
F 3 "" H 3450 6150 60  0000 C CNN
	1    3450 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 6350 3450 6300
Connection ~ 3900 6350
$Comp
L +3V3 #PWR105
U 1 1 57544CE0
P 3450 5750
F 0 "#PWR105" H 3450 5600 50  0001 C CNN
F 1 "+3V3" H 3450 5890 50  0000 C CNN
F 2 "" H 3450 5750 60  0000 C CNN
F 3 "" H 3450 5750 60  0000 C CNN
	1    3450 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 3450 9850 3450
Wire Wire Line
	9850 3450 9850 4600
Connection ~ 9850 3900
Wire Wire Line
	9400 3250 11050 3250
Text Label 9450 3250 0    60   ~ 0
CAM_FOCUS
Text Label 9450 3700 0    60   ~ 0
CAM_TRIGGER
Text Label 9450 3450 0    60   ~ 0
CAM_GND
$Comp
L +3V3 #PWR115
U 1 1 5754570D
P 8100 3250
F 0 "#PWR115" H 8100 3100 50  0001 C CNN
F 1 "+3V3" H 8100 3390 50  0000 C CNN
F 2 "" H 8100 3250 60  0000 C CNN
F 3 "" H 8100 3250 60  0000 C CNN
	1    8100 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 3250 8300 3250
Wire Wire Line
	8100 3250 8100 3700
Wire Wire Line
	8100 3700 8300 3700
Connection ~ 8100 3250
Wire Wire Line
	8600 3250 8800 3250
Wire Wire Line
	8600 3700 8800 3700
Wire Wire Line
	8800 3450 7700 3450
Wire Wire Line
	8800 3900 7700 3900
$Comp
L R R105
U 1 1 57545E8C
P 8350 5250
F 0 "R105" V 8250 5100 50  0000 C CNN
F 1 "1k" V 8350 5250 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 8280 5250 30  0001 C CNN
F 3 "" H 8350 5250 30  0000 C CNN
	1    8350 5250
	0    1    1    0   
$EndComp
Wire Wire Line
	8500 5250 8600 5250
Wire Wire Line
	8600 5350 8200 5350
$Comp
L GND #PWR116
U 1 1 5754600B
P 8200 5350
F 0 "#PWR116" H 8200 5100 50  0001 C CNN
F 1 "GND" H 8200 5200 50  0000 C CNN
F 2 "" H 8200 5350 60  0000 C CNN
F 3 "" H 8200 5350 60  0000 C CNN
	1    8200 5350
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR117
U 1 1 57546059
P 8350 4750
F 0 "#PWR117" H 8350 4600 50  0001 C CNN
F 1 "+3V3" H 8350 4890 50  0000 C CNN
F 2 "" H 8350 4750 60  0000 C CNN
F 3 "" H 8350 4750 60  0000 C CNN
	1    8350 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8350 5150 8600 5150
Wire Wire Line
	7650 5550 8600 5550
Wire Wire Line
	7650 5650 8600 5650
Wire Wire Line
	7650 5750 8600 5750
Wire Wire Line
	7650 5850 8600 5850
Wire Wire Line
	7650 5950 8600 5950
Text Label 6450 1200 2    60   ~ 0
~B1
Text Label 6450 1500 2    60   ~ 0
~B2
Text Label 6450 1800 2    60   ~ 0
~B3
Text Label 6450 2100 2    60   ~ 0
~B4
Wire Wire Line
	8300 1600 7750 1600
Text Label 7950 1600 2    60   ~ 0
~LED
Text Label 2750 6650 0    60   ~ 0
RST
Text Label 2750 6750 0    60   ~ 0
TEST
Text Label 8250 3450 0    60   ~ 0
~FOCUS
Text Label 8250 3900 0    60   ~ 0
~TRIGGER
Wire Wire Line
	7650 5250 8200 5250
Text Label 7650 5250 0    60   ~ 0
LCD_BKL
Text Label 7650 5550 0    60   ~ 0
~LCD_RST
Text Label 7650 5650 0    60   ~ 0
MOSI
Text Label 7650 5750 0    60   ~ 0
SCLK
Text Label 7650 5850 0    60   ~ 0
D~C
Text Label 7650 5950 0    60   ~ 0
~LCD_CS
$Comp
L IRLML6402 Q103
U 1 1 5754B315
P 8250 4950
F 0 "Q103" H 8450 4850 50  0000 L CNN
F 1 "IRLML6402" H 8450 4750 50  0000 L CNN
F 2 "smd-handsolder:SOT-23_HandSoldering" H 8450 5050 29  0001 C CNN
F 3 "" H 8250 4950 60  0000 C CNN
	1    8250 4950
	1    0    0    1   
$EndComp
Wire Wire Line
	8050 4950 7650 4950
Text Label 7650 4950 0    60   ~ 0
~LCD_PWR
$Comp
L GND #PWR101
U 1 1 5754B967
P 800 4250
F 0 "#PWR101" H 800 4000 50  0001 C CNN
F 1 "GND" H 800 4100 50  0000 C CNN
F 2 "" H 800 4250 60  0000 C CNN
F 3 "" H 800 4250 60  0000 C CNN
	1    800  4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 5150 2400 5150
Text Notes 2250 6850 0    60   ~ 0
Programming and debugging
Wire Wire Line
	5450 6150 5900 6150
Text Label 5600 6150 0    60   ~ 0
MOSI
Wire Wire Line
	5450 6350 5900 6350
Text Label 5600 6350 0    60   ~ 0
SCLK
$Comp
L Raltron_32_768kHz_R26 Q102
U 1 1 5754D8F8
P 3350 7200
F 0 "Q102" V 3350 7350 50  0000 C CNN
F 1 "Raltron_32_768kHz_R26" V 3600 6750 50  0000 L CNN
F 2 "th-normal:Crystal_Round_Vertical_2mm" H 3350 7200 50  0001 C CNN
F 3 "" H 3350 7200 50  0000 C CNN
	1    3350 7200
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 7050 3350 7050
Wire Wire Line
	3350 7050 3350 7100
Wire Wire Line
	3350 7300 3350 7350
Wire Wire Line
	3350 7350 4050 7350
$Comp
L GND #PWR104
U 1 1 5754DFCF
P 2500 4150
F 0 "#PWR104" H 2500 3900 50  0001 C CNN
F 1 "GND" H 2500 4000 50  0000 C CNN
F 2 "" H 2500 4150 60  0000 C CNN
F 3 "" H 2500 4150 60  0000 C CNN
	1    2500 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 4100 2500 4150
Wire Wire Line
	5450 5950 5900 5950
Text Label 5600 6250 0    60   ~ 0
D~C
Wire Wire Line
	5450 6050 5900 6050
Text Label 5600 5950 0    60   ~ 0
~LCD_CS
Wire Wire Line
	5450 6250 5900 6250
Text Label 5600 6050 0    60   ~ 0
~LCD_RST
Wire Wire Line
	5450 6450 5900 6450
Wire Wire Line
	5450 6550 5900 6550
Text Label 5600 6450 0    60   ~ 0
~LCD_PWR
Text Label 5600 6550 0    60   ~ 0
LCD_BKL
Wire Wire Line
	9300 1500 9300 1600
Wire Wire Line
	5450 7250 5900 7250
Wire Wire Line
	5450 7350 5900 7350
Text Label 5600 7250 0    60   ~ 0
~FOCUS
Text Label 5600 7350 0    60   ~ 0
~TRIGGER
Wire Wire Line
	5450 6650 5900 6650
Text Label 5600 6650 0    60   ~ 0
~LED
Wire Wire Line
	5450 6850 5900 6850
Wire Wire Line
	5450 6950 5900 6950
Wire Wire Line
	5450 7050 5900 7050
Wire Wire Line
	5450 7150 5900 7150
Text Label 5600 7150 0    60   ~ 0
~B1
Text Label 5600 7050 0    60   ~ 0
~B2
Text Label 5600 6950 0    60   ~ 0
~B3
Text Label 5600 6850 0    60   ~ 0
~B4
Wire Wire Line
	6050 2450 6050 2550
Wire Wire Line
	5850 2450 5850 2550
Wire Wire Line
	5650 2450 5650 2550
Wire Wire Line
	5450 2450 5450 2550
$Comp
L SWITCH_DT SW101
U 1 1 5759E7F0
P 2700 5250
F 0 "SW101" H 2600 5350 50  0000 C CNN
F 1 "SWITCH_DT" H 2550 5150 50  0000 C CNN
F 2 "th-normal:Button_C&K_OS102011MA1QN1" H 2700 5250 60  0001 C CNN
F 3 "" H 2700 5250 60  0000 C CNN
	1    2700 5250
	-1   0    0    -1  
$EndComp
$Comp
L Battery_LiPol_JST-2 BT101
U 1 1 576559E6
P 2500 4000
F 0 "BT101" H 2600 4050 50  0000 L CNN
F 1 "Battery_LiPol_JST-2" H 2200 3900 50  0000 L CNN
F 2 "smd-handsolder:Connector_JST-2_SMD_HandSoldering" V 2500 4020 50  0001 C CNN
F 3 "" V 2500 4020 50  0000 C CNN
	1    2500 4000
	1    0    0    -1  
$EndComp
$Comp
L MAX1555 U104
U 1 1 57656001
P 1400 4000
F 0 "U104" H 1600 4350 60  0000 C CNN
F 1 "MAX1555" H 1500 3650 60  0000 C CNN
F 2 "smd-handsolder:SOT-23-5_HandSoldering" H 1400 4050 60  0001 C CNN
F 3 "" H 1400 4050 60  0000 C CNN
	1    1400 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 3800 2500 3800
Wire Wire Line
	2500 3800 2500 3900
Wire Wire Line
	2150 3800 2150 5150
Connection ~ 2150 3800
Wire Wire Line
	1000 4200 800  4200
Wire Wire Line
	800  4200 800  4250
$Comp
L USB_MINI_B_GMe CONN101
U 1 1 57668B1C
P 1100 2900
F 0 "CONN101" H 950 3000 60  0000 C CNN
F 1 "USB_MINI_B_GMe" H 1000 2900 27  0000 C CNN
F 2 "smd-normal:Connector_USB-Mini-B_GMe_SMD" H 1100 2900 60  0001 C CNN
F 3 "" H 1100 2900 60  0000 C CNN
	1    1100 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  3200 900  3800
Wire Wire Line
	900  3800 1000 3800
$Comp
L GND #PWR102
U 1 1 57668EBB
P 1300 3300
F 0 "#PWR102" H 1300 3050 50  0001 C CNN
F 1 "GND" H 1300 3150 50  0000 C CNN
F 2 "" H 1300 3300 60  0000 C CNN
F 3 "" H 1300 3300 60  0000 C CNN
	1    1300 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 3200 1300 3300
Wire Wire Line
	1300 3250 1500 3250
Wire Wire Line
	1500 3250 1500 2900
Connection ~ 1300 3250
$Comp
L LED D102
U 1 1 57669581
P 1900 4550
F 0 "D102" H 1900 4650 50  0000 C CNN
F 1 "LED" H 1900 4450 50  0000 C CNN
F 2 "th-normal:LED_L-1384_AL-1" H 1900 4550 60  0001 C CNN
F 3 "" H 1900 4550 60  0000 C CNN
	1    1900 4550
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 4200 1900 4200
Wire Wire Line
	1900 4200 1900 4350
$Comp
L R R109
U 1 1 576697A9
P 1900 5000
F 0 "R109" V 1980 5000 50  0000 C CNN
F 1 "1k" V 1900 5000 50  0000 C CNN
F 2 "smd-handsolder:R_1206_HandSoldering" V 1830 5000 30  0001 C CNN
F 3 "" H 1900 5000 30  0000 C CNN
	1    1900 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 4750 1900 4850
Connection ~ 2150 5150
$Comp
L LP2992AIM5-3.3CT-ND-RESCUE-timelapse U105
U 1 1 5766AA84
P 4050 4200
F 0 "U105" H 4150 3850 60  0000 L CNN
F 1 "LP2992AIM5-3.3CT-ND" H 4050 4550 60  0000 C CNN
F 2 "smd-handsolder:SOT-23-5_HandSoldering" H 4350 3950 50  0000 C CNN
F 3 "" H 4050 4100 60  0000 C CNN
	1    4050 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 5250 3000 5250
Wire Wire Line
	3150 4000 3150 5250
Wire Wire Line
	3150 4000 3450 4000
$Comp
L +3.3V #PWR109
U 1 1 5766AF51
P 5050 4000
F 0 "#PWR109" H 5050 3850 50  0001 C CNN
F 1 "+3.3V" H 5050 4140 50  0000 C CNN
F 2 "" H 5050 4000 60  0000 C CNN
F 3 "" H 5050 4000 60  0000 C CNN
	1    5050 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 4000 5050 4000
$Comp
L C C108
U 1 1 5766B09D
P 4900 4300
F 0 "C108" H 4925 4400 50  0000 L CNN
F 1 "100n" H 4925 4200 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 4938 4150 30  0001 C CNN
F 3 "" H 4900 4300 60  0000 C CNN
	1    4900 4300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR107
U 1 1 5766B100
P 4050 5000
F 0 "#PWR107" H 4050 4750 50  0001 C CNN
F 1 "GND" H 4050 4850 50  0000 C CNN
F 2 "" H 4050 5000 60  0000 C CNN
F 3 "" H 4050 5000 60  0000 C CNN
	1    4050 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 4600 4050 5000
Wire Wire Line
	4900 4900 4900 4450
Connection ~ 4050 4900
Wire Wire Line
	4900 4150 4900 4000
Connection ~ 4900 4000
$Comp
L C C107
U 1 1 5766B3EB
P 3500 4900
F 0 "C107" H 3525 5000 50  0000 L CNN
F 1 "100n" H 3525 4800 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 3538 4750 30  0001 C CNN
F 3 "" H 3500 4900 60  0000 C CNN
	1    3500 4900
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 4900 3150 4900
Connection ~ 3150 4900
Wire Wire Line
	3650 4900 4900 4900
$Comp
L C C106
U 1 1 5766B767
P 3350 4500
F 0 "C106" H 3375 4600 50  0000 L CNN
F 1 "100n" H 3375 4400 50  0000 L CNN
F 2 "smd-handsolder:C_1206_HandSoldering" H 3388 4350 30  0001 C CNN
F 3 "" H 3350 4500 60  0000 C CNN
	1    3350 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 4650 3350 4700
Wire Wire Line
	3350 4700 4050 4700
Connection ~ 4050 4700
Wire Wire Line
	3350 4350 3350 4300
Wire Wire Line
	3350 4300 3450 4300
Wire Wire Line
	3450 5750 3450 6000
Wire Wire Line
	2450 5950 4050 5950
Connection ~ 3450 5950
$Comp
L EB-DIO_M06 CONN102
U 1 1 57670DD5
P 10450 4300
F 0 "CONN102" H 10450 3900 60  0000 C CNN
F 1 "EB-DIO_M06" H 10450 4700 60  0000 C CNN
F 2 "th-normal:Connector_DIN-6_EB-DIO-M06" H 10450 4300 60  0001 C CNN
F 3 "" H 10450 4300 60  0000 C CNN
	1    10450 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 3900 9400 3900
Wire Wire Line
	9850 4600 10050 4600
Wire Wire Line
	11050 3250 11050 4100
Wire Wire Line
	11050 4100 10850 4100
Wire Wire Line
	10050 4100 10050 3700
Wire Wire Line
	10050 3700 9400 3700
NoConn ~ 1000 4000
$Comp
L PWR_FLAG #FLG101
U 1 1 576717D5
P 900 3450
F 0 "#FLG101" H 900 3545 50  0001 C CNN
F 1 "PWR_FLAG" H 900 3630 50  0000 C CNN
F 2 "" H 900 3450 60  0000 C CNN
F 3 "" H 900 3450 60  0000 C CNN
	1    900  3450
	0    -1   -1   0   
$EndComp
Connection ~ 900  3450
NoConn ~ 10850 4300
NoConn ~ 10850 4500
NoConn ~ 10050 4500
NoConn ~ 10050 4300
$Comp
L PWR_FLAG #FLG102
U 1 1 57671FA5
P 1500 3250
F 0 "#FLG102" H 1500 3345 50  0001 C CNN
F 1 "PWR_FLAG" H 1500 3430 50  0000 C CNN
F 2 "" H 1500 3250 60  0000 C CNN
F 3 "" H 1500 3250 60  0000 C CNN
	1    1500 3250
	0    1    1    0   
$EndComp
Connection ~ 1500 3250
NoConn ~ 2400 5350
Wire Wire Line
	3450 4100 3150 4100
Connection ~ 3150 4100
$Comp
L PWR_FLAG #FLG103
U 1 1 57673D46
P 3150 4000
F 0 "#FLG103" H 3150 4095 50  0001 C CNN
F 1 "PWR_FLAG" H 3150 4180 50  0000 C CNN
F 2 "" H 3150 4000 60  0000 C CNN
F 3 "" H 3150 4000 60  0000 C CNN
	1    3150 4000
	1    0    0    -1  
$EndComp
Connection ~ 3150 4000
NoConn ~ 1000 3200
NoConn ~ 1100 3200
NoConn ~ 1200 3200
$Comp
L PWR_FLAG #FLG104
U 1 1 5767420B
P 8550 5150
F 0 "#FLG104" H 8550 5245 50  0001 C CNN
F 1 "PWR_FLAG" H 8750 5350 50  0000 C CNN
F 2 "" H 8550 5150 60  0000 C CNN
F 3 "" H 8550 5150 60  0000 C CNN
	1    8550 5150
	1    0    0    -1  
$EndComp
Connection ~ 8550 5150
$Comp
L R R110
U 1 1 577D06C3
P 2450 6150
F 0 "R110" V 2530 6150 50  0000 C CNN
F 1 "47k" V 2450 6150 50  0000 C CNN
F 2 "smd-normal:R_1206" V 2380 6150 30  0001 C CNN
F 3 "" H 2450 6150 30  0000 C CNN
	1    2450 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 6300 2450 6650
Connection ~ 2450 6650
Wire Wire Line
	2450 6000 2450 5950
$EndSCHEMATC
