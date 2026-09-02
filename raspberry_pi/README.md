# Raspberry Pi 5 AI Vision

Raspberry Pi 5 software for real-time walking environment recognition and safe crosswalk navigation.

## Main Functions

- Camera Module 3 real-time image capture
- YOLO-based object detection
- Crosswalk detection and direction analysis
- Traffic light color recognition
- Crosswalk deviation detection and correction
- State-based crosswalk navigation
- UART communication with ESP32

## Detection Targets

- Crosswalk
- Traffic light
- Ground light

## Crosswalk Navigation Flow

NORMAL
→ CROSSWALK_AHEAD
→ WAIT_SIGNAL
→ CROSSING_START
→ STRAIGHT / LEFT_CORRECTION / RIGHT_CORRECTION
→ CROSSING_END

## Communication

Raspberry Pi 5 → ESP32

- UART
- 115200 baud
- `/dev/serial0`
