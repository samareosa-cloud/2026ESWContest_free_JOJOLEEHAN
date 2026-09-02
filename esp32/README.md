# ESP32 Firmware

ESP32 firmware for **Navis**, an AI-based semi-autonomous walking assistance system.

ESP32는 Navis의 중앙 제어기로서 Flutter App, Raspberry Pi, TFmini 거리 센서 및 MDD10A 모터 드라이버를 연결합니다.

주요 역할은 다음과 같습니다.

- Flutter App과 BLE 통신
- Raspberry Pi와 UART 통신
- TFmini 기반 전방 장애물 감지
- MDD10A를 이용한 좌·우 조향 토크 제어
- 횡단보도 주행 중 Raspberry Pi 방향 보정 명령 수행
- 장애물 감지 시 역토크를 이용한 촉각 경고
- 각 제어 시스템 간 우선순위 관리

---

## 1. System Role

ESP32는 Navis의 각 시스템에서 전달되는 정보를 받아 실제 모터 동작으로 변환하는 중앙 제어 장치입니다.

```text
Flutter App
TMAP / GPS / Compass
        │
        │ BLE
        ▼
     ┌───────┐
     │ ESP32 │
     └───┬───┘
         │
         │ PWM / DIR
         ▼
      MDD10A
         │
         ▼
   Left / Right Motor


Raspberry Pi
Camera / YOLO
Crosswalk Detection
        │
        │ UART
        ▼
      ESP32


TFmini
Distance
        │
        │ UART
        ▼
      ESP32
```

---

## 2. Hardware

| Component | Function |
|---|---|
| ESP32 | Main controller |
| MDD10A | Dual-channel motor driver |
| DC Motor ×2 | Left / right steering torque |
| TFmini | Front obstacle distance measurement |
| Raspberry Pi | AI vision processing |
| Smartphone | Navigation and user interface |

---

## 3. Pin Configuration

### MDD10A Motor Driver

| Function | ESP32 GPIO |
|---|---:|
| M1 DIR | GPIO 25 |
| M1 PWM | GPIO 27 |
| M2 DIR | GPIO 32 |
| M2 PWM | GPIO 14 |

Motor control is performed using PWM and direction signals.

---

### TFmini

| TFmini | ESP32 |
|---|---|
| TX | GPIO 16 |
| RX | GPIO 17 |
| GND | GND |

The TFmini communicates with the ESP32 through UART2 at **115200 bps**.

For distance reception only, the primary data path is:

```text
TFmini TX → ESP32 GPIO16 (RX)
```

---

### Raspberry Pi UART

| Raspberry Pi | ESP32 |
|---|---|
| TX | GPIO 18 |
| RX | GPIO 19 |
| GND | GND |

UART communication speed:

```text
115200 bps
```

Raspberry Pi and ESP32 must share a common ground.

---

## 4. BLE Communication

The ESP32 communicates with the Flutter application using Bluetooth Low Energy (BLE).

### Device Name

```text
SMART_CANE
```

### Nordic UART Service

```text
Service UUID
6e400001-b5a3-f393-e0a9-e50e24dcca9e
```

Flutter → ESP32:

```text
6e400002-b5a3-f393-e0a9-e50e24dcca9e
```

ESP32 → Flutter:

```text
6e400003-b5a3-f393-e0a9-e50e24dcca9e
```

---

## 5. Flutter → ESP32 Commands

Flutter App calculates the difference between the target bearing and the current smartphone heading.

The calculated navigation command is transmitted to ESP32 through BLE.

```text
B:<bearing>
L:<angle>
R:<angle>
F:<angle>
S:0
```

### Command Description

| Command | Description |
|---|---|
| `B:<bearing>` | Target bearing |
| `L:<angle>` | Left steering guidance |
| `R:<angle>` | Right steering guidance |
| `F:<angle>` | Forward / neutral |
| `S:0` | Stop motor output |

Example:

```text
B:135
L:32.5
R:28.0
F:3.0
S:0
```

---

## 6. Navigation Motor Control

The motors are not used to continuously propel the device.

The user moves the device forward manually, while the motors generate short steering torque to indicate the required direction.

### Left Guidance

```text
Flutter
   ↓
L:<angle>
   ↓
ESP32
   ↓
M2 Motor
   ↓
Left steering torque
```

### Right Guidance

```text
Flutter
   ↓
R:<angle>
   ↓
ESP32
   ↓
M1 Motor
   ↓
Right steering torque
```

### Forward

When the current direction is sufficiently aligned with the target direction, both motors remain neutral.

---

## 7. Motor Parameters

Navigation steering parameters are defined in the firmware.

```cpp
const int GUIDE_PWM = 40;
const int GUIDE_TIME = 200;
```

- `GUIDE_PWM` : steering torque strength
- `GUIDE_TIME` : steering torque duration

These values can be adjusted depending on the weight of the device and the required steering feedback.

---

## 8. TFmini Obstacle Detection

TFmini is mounted facing forward and continuously measures the distance to obstacles.

### Detection Logic

```text
TFmini Distance
      │
      ├── > 100 cm
      │      Normal
      │
      ├── <= 100 cm
      │      Flutter obstacle display
      │
      └── <= 30 cm
             │
             ▼
       Obstacle Confirmed
             │
             ▼
       Reverse Torque ×3
```

The ESP32 confirms an obstacle when a distance of **30 cm or less is detected three consecutive times**.

```text
Obstacle threshold : 30 cm
Reset threshold    : 40 cm
```

When the distance becomes 40 cm or greater, the obstacle state is cleared.

---

## 9. Obstacle Warning

When an obstacle is confirmed, the ESP32 generates three short reverse-torque pulses through both motors.

```text
Obstacle detected
       ↓
Motor Stop
       ↓
Reverse Torque
       ↓
OFF
       ↓
Reverse Torque
       ↓
OFF
       ↓
Reverse Torque
       ↓
Motor Stop
```

Current parameters:

```text
PWM       : 33
ON Time   : 300 ms
OFF Time  : 200 ms
Count     : 3
```

The warning is designed to provide tactile feedback through the device without requiring obstacle voice guidance.

---

## 10. Raspberry Pi → ESP32 Communication

Raspberry Pi performs AI-based crosswalk and traffic-light recognition.

The recognition results are sent to ESP32 through UART.

### Crosswalk Detection

```text
CROSSWALK:1
CROSSWALK:0
```

| Message | Description |
|---|---|
| `CROSSWALK:1` | Crosswalk detected |
| `CROSSWALK:0` | Crosswalk mode ended |

---

### Traffic Light

```text
LIGHT:RED
LIGHT:GREEN
LIGHT:NONE
```

When a red light is detected:

```text
LIGHT:RED
    ↓
trafficRed = true
    ↓
Motor Stop
```

---

### Crossing State

```text
CROSSING_START
CROSSING_END
```

`CROSSING_START` activates crosswalk steering control.

`CROSSING_END` returns the system to normal Flutter navigation.

---

## 11. Crosswalk Steering

During road crossing, Raspberry Pi analyzes the crosswalk position and direction.

It sends one of the following commands:

```text
CROSS_MOTOR:L
CROSS_MOTOR:R
CROSS_MOTOR:CENTER
```

### LEFT

```text
Raspberry Pi
CROSS_MOTOR:L
      ↓
ESP32
      ↓
M2 Motor
      ↓
Left correction
```

### RIGHT

```text
Raspberry Pi
CROSS_MOTOR:R
      ↓
ESP32
      ↓
M1 Motor
      ↓
Right correction
```

### CENTER

```text
Raspberry Pi
CROSS_MOTOR:CENTER
      ↓
ESP32
      ↓
Motor Stop / Neutral
```

This allows the user to receive steering guidance while crossing and helps prevent deviation from the crosswalk path.

---

## 12. Control Mode

The system normally receives navigation commands from Flutter.

When a crosswalk is detected, normal Flutter steering commands are temporarily blocked and Raspberry Pi crosswalk control takes priority.

```text
NORMAL MODE
Flutter Navigation
       │
       ▼
ESP32 Motor Control

       ↓ Crosswalk detected

CROSSWALK MODE
Raspberry Pi Vision
       │
       ▼
ESP32 Motor Control

       ↓ Crossing end

NORMAL MODE
Flutter Navigation
```

---

## 13. Safety Control

ESP32 manages multiple inputs to prevent conflicting motor commands.

The main control conditions are:

```text
Obstacle detected
→ Normal navigation command blocked

Red traffic light
→ Motor stopped

Crosswalk mode
→ Flutter L/R/F commands blocked
→ Raspberry Pi crosswalk correction enabled

Crossing ended
→ Flutter navigation restored
```

The `S:0` stop command from Flutter is always accepted.

---

## 14. ESP32 → Flutter Messages

ESP32 sends sensor and system status information back to the Flutter application.

### Distance

```text
DIST:<cm>
```

Example:

```text
DIST:85
```

### Obstacle

```text
OBSTACLE:<cm>
OBSTACLE_CLEAR
```

### Crosswalk

```text
CROSSWALK:1
CROSSWALK:0
```

### Traffic Light

```text
LIGHT:RED
LIGHT:GREEN
LIGHT:NONE
```

### Crossing

```text
CROSSING_START
CROSSING_END
```

### Crosswalk Steering

```text
CROSS_MOTOR:L
CROSS_MOTOR:R
CROSS_MOTOR:CENTER
```

---

## 15. Development Environment

- ESP32
- Arduino IDE
- Arduino Framework
- C++
- Bluetooth Low Energy
- UART
- PWM Motor Control

---

## 16. Firmware File

```text
navis_esp32.ino
```

The firmware includes:

- BLE server
- Flutter command processing
- Raspberry Pi UART message processing
- TFmini UART distance measurement
- MDD10A motor control
- Crosswalk steering control
- Traffic light safety control
- Obstacle detection
- Reverse-torque obstacle warning

---

## 17. Upload

Open the following file in Arduino IDE:

```text
navis_esp32.ino
```

Connect the ESP32 to the computer through USB, select the correct ESP32 board and COM port, and upload the firmware.

After a successful upload, the firmware is stored in ESP32 flash memory and runs automatically whenever the ESP32 is powered.

---

## 18. System Summary

```text
Flutter Navigation
        │
        │ BLE
        ▼
      ESP32 ◀──── UART ──── Raspberry Pi
        ▲                    Crosswalk / Signal
        │
        │ UART
        │
      TFmini
        │
        ▼
Obstacle Detection

        ESP32
          │
          ▼
        MDD10A
          │
          ▼
    Steering Motors
```

The ESP32 acts as the central controller that integrates **navigation, AI vision, obstacle detection, and motor steering guidance** for the Navis system.
