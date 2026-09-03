# Raspberry Pi 5 AI Vision

Raspberry Pi 5 software for real-time pedestrian environment recognition and crosswalk navigation in the NAVIS system.

## Main Functions

- Camera Module 3 real-time image capture
- YOLO-based pedestrian environment recognition
- OpenCV-based crosswalk direction analysis
- Traffic light color recognition
- Crosswalk deviation detection and steering correction
- State-based crosswalk navigation
- UART communication with ESP32

## Detection Targets

- Crosswalk
- Traffic light
- Ground light

## Crosswalk Direction Analysis

Crosswalk direction is analyzed using OpenCV image processing.

- Crosswalk ROI extraction using YOLO detection results
- Edge detection using Canny
- Line detection using HoughLinesP
- Crosswalk angle estimation
- LEFT / CENTER / RIGHT steering command generation

## Crosswalk Navigation Flow

NORMAL  
→ CROSSWALK_AHEAD  
→ WAIT_SIGNAL  
→ CROSSING_START  
→ LEFT / CENTER / RIGHT CORRECTION  
→ CROSSING_END

## AI Model

The trained YOLO model is converted to NCNN format for inference on Raspberry Pi 5.

Model files are stored in:

`best_ncnn_model/`

## Communication

Raspberry Pi 5 → ESP32

- UART
- 115200 baud
- `/dev/serial0`

## Role in NAVIS

The Raspberry Pi 5 recognizes the surrounding pedestrian environment
and determines crosswalk and traffic-light states.

The processed navigation commands are transmitted to the ESP32,
which controls the left and right motors for steering assistance.
