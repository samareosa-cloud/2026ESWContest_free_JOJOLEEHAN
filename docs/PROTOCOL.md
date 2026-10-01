# Navis 통신 프로토콜 (Raspberry Pi ↔ ESP32 ↔ Flutter)

세 프로그램이 주고받는 메시지 이름은 **이 문서가 기준**입니다.
이름을 바꾸거나 추가할 때는 이 문서부터 고치고, 아래 세 곳을 같이 맞춥니다.

| 위치 | 코드 |
|---|---|
| Raspberry Pi | `raspberry_pi/main.py` → `esp_messages_for_event()` (내부 상태 → 메시지 변환은 이 함수 한 곳에서만) |
| ESP32 | `esp32/navis_esp32.ino` → `handlePiMessage()`, `handleFlutterCommand()` |
| Flutter | `flutter_app/lib/main.dart` → `_handleEsp32Message()` |

모든 메시지는 한 줄 텍스트이고 끝에 `\n`을 붙입니다.

## Raspberry Pi → ESP32 (UART 115200)

| 메시지 | 언제 | ESP32 동작 | 앱 음성 |
|---|---|---|---|
| `CROSSWALK:1` | 횡단보도 앞 도착 | 횡단보도 모드, 모터 정지 | 앞에 횡단보도가 있습니다 |
| `LIGHT:RED` | 빨간불 확인 (대기) | 정지 유지 | 빨간불입니다. 정지해 주세요 |
| `LIGHT:GREEN_WAIT` | 빨간불을 못 보고 초록불부터 봄 (남은 시간 모름) | 정지 유지 | 다음 초록불에 건너겠습니다 |
| `LIGHT:GREEN` + `CROSSING_START` | 빨간불 → 초록불 확인, 횡단 시작 | 보정 준비 | 초록불입니다. 건너세요 |
| `CROSS_MOTOR:L` / `:R` / `:CENTER` | 건너는 중 보정 방향. **0.3초마다 반복 전송** | L/R이면 0.6초마다 당김, CENTER면 멈춤 | (화면 표시만) |
| `CROSSING_END` | 횡단보도가 더 이상 안 보임 | 횡단 모드 해제 | 횡단보도를 다 건넜습니다 |
| `CROSSWALK:0` | 일반 보행 상태 | 횡단 모드 해제 | - |

**안전장치:** ESP32는 `CROSS_MOTOR:L/R`를 받은 뒤 1초 동안 Pi 메시지가 없으면
스스로 `CENTER`로 바꾸고 멈춥니다. (Pi 멈춤, 케이블 빠짐 대비)
그래서 Pi는 건너는 동안 같은 방향이라도 계속 다시 보내야 합니다.

## Flutter → ESP32 (BLE)

| 메시지 | 뜻 |
|---|---|
| `B:<방위각>` | 목표 방위각 (로그용) |
| `L:<각도>` / `R:<각도>` | 왼쪽/오른쪽 유도 (한 번 당김) |
| `F:<각도>` | 직진, 모터 정지 |
| `S:0` | 정지 |

횡단보도 모드(Pi가 보정 중)와 장애물 회피 중에는 ESP32가 앱의 L/R을 무시합니다.

## ESP32 → Flutter (BLE)

| 메시지 | 뜻 |
|---|---|
| `DIST:<cm>` | TFmini 정면 거리 (0.3초마다) |
| `SIDE:<왼쪽cm>,<오른쪽cm>` | VL53L1X 좌/우 거리 (0.3초마다, 센서 고장 시 -1, 반사 없음(멀리 트임) 시 400) |
| `OBSTACLE:<cm>` | 정면 30cm 이하 3회 연속 → 장애물 확정 |
| `OBSTACLE_BRAKE` | 역토크 시작 (앱: "장애물이 감지되었습니다") |
| `AVOID:L` / `AVOID:R` | 그쪽으로 정면이 트일 때까지 계속 유도 |
| `AVOID:BLOCKED` | 양옆도 막혔거나 계속 유도해도 안 트임 → 멈춤 |
| `OBSTACLE_CLEAR` | 정면 40cm 이상 → 해제 |
| `MOTOR:L` / `MOTOR:R` / `MOTOR:STOP` | 모터 동작 확인 (로그용) |
| Pi 메시지 | 위 Pi → ESP32 메시지를 앱에 그대로 전달 (`CROSS_MOTOR`는 바뀔 때만) |

## 우선순위

장애물 회피 > 신호 대기(빨간불) > 횡단 보정(Pi) > 일반 길안내(앱)
