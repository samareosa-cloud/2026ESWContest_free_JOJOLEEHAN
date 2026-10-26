# Raspberry Pi

Camera Module 3 영상으로 **횡단보도·신호등을 인식**하고, 횡단 상태와 조향 방향을 **UART로 ESP32에 전송**하는 Raspberry Pi 4 코드입니다.

## 인식 대상

YOLO(NCNN) 모델 `best4_ncnn_model`로 `cross-walk`, `traffic_light`, `ground_light`를 검출하고, 클래스별로 가장 큰 박스만 사용합니다.

## 상태 흐름

```text
NORMAL → CROSSWALK_AHEAD → WAIT_SIGNAL → CROSSING_START
       → STRAIGHT / LEFT_CORRECTION / RIGHT_CORRECTION → CROSSING_END → NORMAL
```

| 상태 | 진입 조건 | 전송 메시지 |
|---|---|---|
| `NORMAL` | 시작 / 종료 후 0.5초 | `CROSSWALK:0` |
| `CROSSWALK_AHEAD` | 횡단보도 하단이 화면 40% 아래로 5프레임 | `CROSSWALK:1` |
| `WAIT_SIGNAL` | 빨간불 5프레임 (빨간불 없이 초록불부터 보면 대기) | `LIGHT:RED` / `LIGHT:GREEN_WAIT` |
| `CROSSING_START` | 빨간불을 본 뒤 초록불 5프레임 | `LIGHT:GREEN`, `CROSSING_START` |
| 횡단 보정 | 횡단 중 매 프레임 | `CROSS_MOTOR:CENTER/L/R` (0.3초마다 재전송) |
| `CROSSING_END` | 횡단보도 10프레임 연속 미검출 | `CROSSING_END` |

횡단 전에 횡단보도가 10프레임 동안 보이지 않으면 `NORMAL`로 돌아갑니다. 프레임 수는 카메라 FPS가 아니라 추론 루프 기준입니다.

## 횡단 중 조향 우선순위

1. **이탈 복귀**: 박스 폭이 화면의 35% 이하로 줄고 한쪽에 치우치면 그쪽으로 복귀합니다. 박스가 갑자기 사라지면 직전 추세로 최대 0.6초만 복귀합니다. 폭 60% 이상을 2프레임 본 뒤에만 동작합니다 (HUD `ESC` 점: 초록 = 활성).
2. **중앙 보정**: 박스 중심이 화면 중심에서 12% 이상 벗어나면 6% 안으로 들어올 때까지 중앙 쪽으로 보정합니다.
3. **각도 보정**: Canny + HoughLinesP로 줄무늬 각도를 구하고 EMA로 평활화합니다. ±3°를 넘으면 보정을 시작하고, ±2° 안으로 들어오면 직진으로 돌아갑니다. 줄무늬가 2프레임 동안 보이지 않으면 직진합니다.

`FLIP_LEFT_RIGHT`는 **각도 보정에만** 좌우 반전을 적용합니다. 실제 조향 방향이 반대라면 `False`로 바꿉니다.

## 신호등 색 판별

신호등(없으면 바닥 신호등) ROI에서 HSV 빨강/초록 마스크의 픽셀 비율·채도·명도로 점수를 매기고, 점수가 0.18 이상인 색을 택합니다. 색이 바뀌려면 5프레임 연속 확인이 필요하고, 신호등이 10프레임 동안 보이지 않으면 `Unknown`으로 초기화합니다.

## 설정

| 항목 | 값 |
|---|---|
| UART 포트 | `/dev/ttyS0`, 115200 bps |
| 카메라 | 640×480, 180° 회전 |
| 튜닝 값 | `main.py` 상단 상수 |

메시지 전체 목록과 실행 방법은 [루트 README](../README.md)를 참고하세요.
