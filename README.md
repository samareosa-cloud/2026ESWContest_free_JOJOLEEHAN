# Navis

> AI-based semi-autonomous walking assistance system for safe crosswalk navigation and guidance for visually impaired people.

Navis는 시각장애인의 보다 안전한 보행을 지원하기 위해 개발한 **AI 기반 반자율 보행 보조 시스템**입니다.

기존 보행 보조기기가 장애물이나 주변 환경을 감지한 뒤 음성 또는 진동으로 정보를 전달하는 방식에 집중했다면, Navis는 주변 환경과 목적지 방향을 판단하여 **모터의 조향 토크를 통해 사용자가 이동해야 할 방향을 직접 느낄 수 있도록 유도**합니다.

특히 횡단보도에서는 Raspberry Pi와 카메라를 이용해 횡단보도 및 신호등을 인식하고, 횡단 중에도 횡단보도의 방향을 지속적으로 분석하여 사용자가 횡단보도를 벗어나지 않도록 좌·우 방향을 보정합니다.

---

## 1. Project Overview

Navis의 주요 기능은 다음과 같습니다.

- 스마트폰 기반 목적지 검색 및 보행 경로 안내
- 카메라 및 AI를 이용한 횡단보도·신호등 인식
- 횡단보도 진입 및 횡단 중 방향 보정
- TFmini 거리 센서를 이용한 전방 장애물 감지
- 모터의 토크를 이용한 좌·우 방향 유도
- 빨간불 및 장애물 감지 시 안전 동작 수행
- Raspberry Pi, ESP32, Flutter App 간 실시간 통신

사용자가 장치를 직접 앞으로 이동시키며, 모터는 지속적인 주행을 담당하지 않고 **방향을 알려주는 조향 보조 역할**을 수행합니다.

---

## 2. System Architecture

```text
                ┌─────────────────────────┐
                │      Flutter App        │
                │                         │
                │ TMAP / GPS / Compass    │
                │ Destination Navigation  │
                └───────────┬─────────────┘
                            │
                            │ BLE
                            ▼
┌────────────────┐   ┌──────────────────────┐
│ Raspberry Pi   │   │        ESP32         │
│                │   │                      │
│ Camera         │──▶│ Central Controller   │
│ YOLO           │UART│ BLE / UART / Motor  │
│ Crosswalk      │   │ TFmini Processing    │
│ Traffic Light  │   └───────────┬──────────┘
└────────────────┘               │
                                 │
                    ┌────────────┴────────────┐
                    │                         │
                    ▼                         ▼
              ┌──────────┐              ┌──────────┐
              │ MDD10A   │              │ TFmini   │
              │ Motor    │              │ Distance │
              │ Driver   │              │ Sensor   │
              └────┬─────┘              └──────────┘
                   │
              ┌────┴────┐
              │ Motors  │
              │ L / R   │
              └─────────┘
```

---

## 3. Main Functions

### 3.1 Destination Navigation

Flutter App에서 사용자가 음성으로 목적지를 입력하면 TMAP API를 이용해 목적지를 검색하고 보행자 경로를 생성합니다.

스마트폰의 GPS와 나침반 정보를 이용하여 목표 방위각과 현재 진행 방향의 차이를 계산하고 이동 방향을 판단합니다.

Flutter App은 BLE를 통해 ESP32에 다음과 같은 명령을 전송합니다.

```text
L:<angle>    Left guidance
R:<angle>    Right guidance
F:<angle>    Forward / Neutral
S:0          Stop
B:<bearing>  Target bearing
```

ESP32는 명령을 수신한 뒤 좌·우 모터에 짧은 토크를 발생시켜 사용자에게 이동 방향을 전달합니다.

---

### 3.2 AI Crosswalk Detection

Raspberry Pi와 Camera Module을 이용하여 실시간 영상을 입력받고 YOLO 기반 객체 인식 모델을 통해 보행 환경을 분석합니다.

주요 인식 대상은 다음과 같습니다.

- Crosswalk
- Traffic Light
- Ground Light
- Tactile Paving
- Stairs

횡단보도가 일정 시간 안정적으로 인식되고 사용자 전방에 위치하면 일반 목적지 경로 안내보다 횡단보도 제어를 우선합니다.

```text
Normal Navigation
       ↓
Crosswalk Detection
       ↓
Crosswalk Mode
       ↓
Traffic Light Detection
       ↓
Crossing Start
       ↓
Crosswalk Steering Correction
       ↓
Crossing End
       ↓
Normal Navigation
```

---

### 3.3 Traffic Light Detection

YOLO를 이용하여 신호등 영역을 검출한 뒤 HSV 색상 분석을 통해 RED / GREEN 신호를 판단합니다.

Raspberry Pi 내부에서 일정 프레임 이상 동일한 색상이 유지되었을 때 신호를 확정하여 순간적인 오인식을 줄였습니다.

Raspberry Pi에서 ESP32로 다음 메시지를 전송합니다.

```text
LIGHT:RED
LIGHT:GREEN
LIGHT:NONE
```

빨간불이 인식되면 ESP32는 모터 출력을 정지시키고 Flutter App에도 현재 신호 상태를 전달합니다.

---

### 3.4 Crosswalk Steering Correction

횡단이 시작되면 Raspberry Pi는 횡단보도 영상의 위치와 선 방향을 분석합니다.

OpenCV 기반 영상처리 과정은 다음과 같습니다.

```text
Crosswalk ROI
     ↓
Grayscale / HSV
     ↓
Gaussian Blur
     ↓
Canny Edge Detection
     ↓
Hough Line Transform
     ↓
Crosswalk Angle Estimation
     ↓
EMA Filtering
     ↓
LEFT / RIGHT / CENTER
```

분석된 방향은 UART를 통해 ESP32로 전달됩니다.

```text
CROSS_MOTOR:L
CROSS_MOTOR:R
CROSS_MOTOR:CENTER
```

ESP32는 해당 명령에 따라 모터 토크를 발생시켜 사용자가 횡단보도 중앙을 따라 이동할 수 있도록 보조합니다.

---

### 3.5 Obstacle Detection

전방에 고정된 TFmini 거리 센서를 이용하여 장애물을 감지합니다.

```text
Distance > 100 cm
    Normal

Distance <= 100 cm
    Flutter screen warning

Distance <= 30 cm
    ESP32 obstacle confirmation
        ↓
    Reverse Torque × 3

Distance >= 40 cm
    Obstacle State Reset
```

30 cm 이하의 장애물이 감지되면 ESP32에서 양쪽 모터에 짧은 역토크를 3회 발생시켜 사용자에게 위험을 전달합니다.

현재 TFmini는 정면 고정 방식으로 사용하며 별도의 서보모터는 사용하지 않습니다.

---

## 4. Control Priority

여러 입력이 동시에 발생할 수 있기 때문에 다음과 같은 우선순위로 제어됩니다.

```text
1. Obstacle Safety
2. Traffic Light / Crosswalk Control
3. Raspberry Pi Crosswalk Steering
4. Flutter Navigation
```

횡단보도 모드에서는 Flutter의 일반 목적지 방향 명령보다 Raspberry Pi의 횡단보도 방향 보정 명령이 우선됩니다.

---

## 5. Hardware

| Component | Role |
|---|---|
| Raspberry Pi | AI vision processing |
| Camera Module | Crosswalk / traffic light recognition |
| ESP32 | Main controller |
| MDD10A | Motor driver |
| DC Motors | Steering torque |
| TFmini | Front obstacle distance detection |
| Battery | System power supply |
| Smartphone | Navigation / UI / voice input |

---

## 6. ESP32 Pin Map

### MDD10A

| Function | ESP32 GPIO |
|---|---:|
| M1 DIR | GPIO 25 |
| M1 PWM | GPIO 27 |
| M2 DIR | GPIO 32 |
| M2 PWM | GPIO 14 |

### TFmini

| TFmini | ESP32 |
|---|---|
| TX | GPIO 16 |
| RX | GPIO 17 |
| GND | GND |

### Raspberry Pi UART

| Raspberry Pi | ESP32 |
|---|---|
| TX | GPIO 18 |
| RX | GPIO 19 |
| GND | GND |

Raspberry Pi와 ESP32는 반드시 GND를 공통으로 연결해야 합니다.

---

## 7. Communication

### Flutter ↔ ESP32

Communication: **BLE**

Device Name:

```text
SMART_CANE
```

Nordic UART Service:

```text
Service UUID
6e400001-b5a3-f393-e0a9-e50e24dcca9e

Flutter -> ESP32
6e400002-b5a3-f393-e0a9-e50e24dcca9e

ESP32 -> Flutter
6e400003-b5a3-f393-e0a9-e50e24dcca9e
```

---

### Raspberry Pi ↔ ESP32

Communication: **UART 115200 bps**

Main messages:

```text
CROSSWALK:1
CROSSWALK:0

LIGHT:RED
LIGHT:GREEN
LIGHT:NONE

CROSSING_START

CROSS_MOTOR:L
CROSS_MOTOR:R
CROSS_MOTOR:CENTER

CROSSING_END
```

---

### ESP32 → Flutter

```text
DIST:<cm>

OBSTACLE:<cm>
OBSTACLE_CLEAR

CROSSWALK:1
CROSSWALK:0

LIGHT:RED
LIGHT:GREEN
LIGHT:NONE

CROSSING_START
CROSSING_END

CROSS_MOTOR:L
CROSS_MOTOR:R
CROSS_MOTOR:CENTER

MOTOR:L
MOTOR:R
MOTOR:STOP
```

---

## 8. Software

### Raspberry Pi

- Python
- YOLO
- Ultralytics
- OpenCV
- NumPy
- PiCamera2
- PySerial

### ESP32

- Arduino Framework
- C++
- BLE
- UART
- PWM Motor Control

### Mobile Application

- Flutter
- Dart
- flutter_blue_plus
- flutter_compass
- flutter_map
- flutter_tts
- speech_to_text
- geolocator
- TMAP API
- OpenStreetMap

---

## 9. Repository Structure

```text
Navis/
│
├── esp32/
│   └── navis_esp32.ino
│
├── raspberry_pi/
│   └── navis_ai.py
│
├── flutter_app/
│   └── lib/
│       └── main.dart
│
└── README.md
```

### `esp32`

ESP32 firmware responsible for:

- BLE communication
- Raspberry Pi UART communication
- TFmini distance measurement
- Motor control
- Obstacle warning
- Crosswalk steering control

### `raspberry_pi`

AI vision software responsible for:

- Camera input
- YOLO inference
- Crosswalk detection
- Traffic light detection
- Crosswalk direction analysis
- UART communication with ESP32

### `flutter_app`

Mobile application responsible for:

- Voice destination input
- TMAP POI search
- Pedestrian route generation
- GPS tracking
- Smartphone compass
- BLE communication
- Navigation UI
- Crosswalk / traffic light / obstacle status display

---

## 10. Flutter App Execution

TMAP App Key는 GitHub에 직접 저장하지 않습니다.

Flutter 실행 시 다음과 같이 App Key를 전달합니다.

```bash
flutter run --dart-define=TMAP_APP_KEY=YOUR_TMAP_APP_KEY
```

> Do not commit your actual TMAP API key to this repository.

---

## 11. Raspberry Pi Execution

Required packages:

```bash
pip install ultralytics numpy pyserial
```

OpenCV와 PiCamera2는 Raspberry Pi OS 환경에 맞게 설치해야 합니다.

AI model file:

```text
best_ncnn_model
```

Raspberry Pi에서 실행:

```bash
python3 navis_ai.py
```

---

## 12. ESP32

ESP32 firmware can be compiled and uploaded using Arduino IDE.

Main configuration:

```text
BLE Device Name : SMART_CANE
UART Baud Rate  : 115200
Guide PWM       : 40
Guide Time      : 200 ms
Obstacle Range  : 30 cm
Obstacle Reset  : 40 cm
```

Obstacle warning:

```text
PWM      : 33
ON       : 300 ms
OFF      : 200 ms
Count    : 3
```

---

## 13. Key Features

Navis의 핵심 차별점은 단순히 주변 환경을 사용자에게 알려주는 것에 그치지 않고, **사용자가 이동해야 할 방향을 시스템이 판단하고 모터 토크를 통해 직접 전달한다는 점**입니다.

특히 횡단보도에서

```text
횡단보도 인식
→ 신호등 판단
→ 횡단 시작
→ 횡단보도 방향 분석
→ 좌·우 보정
→ 횡단 종료
```

과정을 하나의 시스템으로 구현하여 안전한 횡단을 지원합니다.

---

## 14. Future Work

향후 다음과 같은 기능을 확장할 수 있습니다.

- AI 모델 정확도 개선
- 다양한 횡단보도 환경 데이터 추가
- 야간 환경 인식 성능 향상
- 신호등 인식 안정성 개선
- GPS 및 방향 추정 정확도 개선
- 실외 장거리 보행 테스트
- 하드웨어 경량화
- 사용자 피드백 기반 조향 토크 최적화

---

## 15. Project Goal

Navis aims to provide a safer and more intuitive walking experience by combining:

**AI Vision + Navigation + Sensor Fusion + Motor Steering Guidance**

The system is designed to assist visually impaired pedestrians in recognizing crosswalk environments and maintaining a safe walking direction during road crossing.
