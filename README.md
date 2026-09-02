# Navis

> AI-based semi-autonomous walking assistance system for safe crosswalk navigation and guidance for visually impaired people.

Navis는 시각장애인의 보다 안전한 보행을 지원하기 위해 개발한 **AI 기반 반자율 보행 보조 시스템**입니다.

기존 보행 보조기기가 장애물이나 주변 환경을 감지한 뒤 음성 또는 진동으로 정보를 전달하는 방식에 집중했다면, Navis는 주변 환경과 목적지 방향을 판단하여 **모터의 조향 토크를 통해 사용자가 이동해야 할 방향을 직접 느낄 수 있도록 유도**합니다.

특히 횡단보도에서는 Raspberry Pi와 카메라를 이용해 횡단보도 및 신호등을 인식하고, 횡단 중에도 횡단보도의 방향을 지속적으로 분석하여 사용자가 횡단보도를 벗어나지 않도록 좌·우 방향을 보정합니다.

---

## 1. Project Overview

Navis의 주요 목적은 다음과 같습니다.

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
