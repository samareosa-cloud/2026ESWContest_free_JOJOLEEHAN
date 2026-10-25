Smart Cane – ESP32

스마트 지팡이의 센서 데이터 처리, 모터 제어 및 장치 간 통신을 담당하는 ESP32 펌웨어입니다.

Flutter 앱에서 BLE로 방향 명령을 수신하고, Raspberry Pi에서 UART로 횡단보도 및 신호등 인식 결과를 전달받습니다. 또한 거리 센서를 통해 장애물을 감지하고 모터를 제어합니다.

1. System Architecture

flowchart LR
    subgraph APP["Flutter App"]
        direction TB
        NAV["Navigation Command"]
        DISPLAY["Sensor Status / Alerts"]
    end
    subgraph PI["Raspberry Pi 5"]
        direction TB
        CROSS["Crosswalk Detection"]
        LIGHT["Traffic Light Detection"]
        STEER["Crossing Direction"]
    end
    subgraph ESP["ESP32"]
        direction TB
        BLE["BLE Communication"]
        UART["UART Communication"]
        SENSOR["Distance Processing"]
        CTRL["Priority & State Control"]
        AVOID["Obstacle Avoidance"]
        PWM["Motor PWM Control"]
        BLE --> CTRL
        UART --> CTRL
        SENSOR --> CTRL
        CTRL --> AVOID
        AVOID --> PWM
        CTRL --> PWM
    end
    subgraph DIST["Distance Sensors"]
        direction TB
        TF["TFmini<br/>Front"]
        VL["VL53L1X × 2<br/>Left / Right"]
    end
    subgraph MOTOR["Motor Driver"]
        direction TB
        MDD["MDD10A"]
        M1["Left Motor"]
        M2["Right Motor"]
        MDD --> M1
        MDD --> M2
    end
    APP <-->|"BLE"| BLE
    PI -->|"UART1 · 115200"| UART
    TF -->|"UART2 · 115200"| SENSOR
    VL -->|"I2C"| SENSOR
    PWM -->|"PWM / DIR"| MDD

2. Main Features

기능	동작
BLE 통신	Flutter 앱과 방향 명령 및 센서 데이터 송수신
방향 유도	L/R 명령에 따라 한쪽 모터를 짧게 구동
전방 장애물 감지	TFmini로 전방 거리 측정
장애물 경고	40cm 이하 3회 연속 감지 시 역토크 3회
장애물 회피	좌우 VL53L1X 거리 비교 후 회피 방향 결정
횡단보도 제어	Raspberry Pi의 횡단보도·신호등 인식 결과 처리
횡단 방향 보정	Raspberry Pi의 L/R/CENTER 명령에 따라 모터 펄스 제어

Control Priority

Obstacle Avoidance > Crosswalk Control > Navigation

3. Pin Configuration

Device	Signal	ESP32 GPIO
MDD10A	M1 DIR / PWM	25 / 27
MDD10A	M2 DIR / PWM	32 / 14
TFmini	RX / TX	16 / 17
VL53L1X	SDA / SCL	21 / 22
VL53L1X LEFT	XSHUT	23
VL53L1X RIGHT	XSHUT	26
Raspberry Pi	RX / TX	18 / 19

* VL53L1X LEFT I2C Address: 0x30
* VL53L1X RIGHT I2C Address: 0x31
* UART Baud Rate: 115200
* Motor PWM: 5 kHz / 8-bit

4. Communication Protocol

Flutter → ESP32 (BLE)

Command	Description
L:45	Left guidance
R:45	Right guidance
F:0	Forward / Neutral
S:0	Stop motors
B:135	Target bearing

Raspberry Pi → ESP32 (UART)

Command	Description
CROSSWALK:1/0	Crosswalk mode ON/OFF
LIGHT:RED	Red light / Reverse torque × 2 on transition
LIGHT:GREEN	Green light
LIGHT:GREEN_WAIT	Wait for next signal
CROSSING_START	Start crossing
CROSSING_END	End crossing
CROSS_MOTOR:L	Left correction
CROSS_MOTOR:R	Right correction
CROSS_MOTOR:CENTER	Stop correction

ESP32 → Flutter (BLE Notify)

Message	Description
DIST:80	Front distance (cm)
SIDE:80,120	Left / Right distance (cm)
OBSTACLE:35	Obstacle detected
OBSTACLE_BRAKE	Reverse torque warning
OBSTACLE_CLEAR	Obstacle cleared
AVOID:L	Avoid left
AVOID:R	Avoid right
AVOID:BLOCKED	No available direction
MOTOR:L/R	Motor guidance status
MOTOR:STOP	Motor stopped

5. Motor & Obstacle Parameters

Parameter	Value
Obstacle Detection	≤ 40 cm × 3
Obstacle Reset	≥ 50 cm
Side Block Distance	< 30 cm
Direction Margin	20 cm
Guide PWM / Duration	100 / 250 ms
Avoid PWM / Duration	120 / 400 ms
Avoid Repeat Interval	900 ms
Max Avoid Pulses	8
Reverse Torque PWM	100
Obstacle Brake	3 pulses
Red Light Brake	2 pulses
Crosswalk PWM	110
Crosswalk Pulse Max	300 ms
Raspberry Pi Timeout	1000 ms

6. Dependencies

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <VL53L1X.h>

* ESP32 Arduino Core 3.x
* VL53L1X Library (Pololu)
* ESP32 BLE Library
