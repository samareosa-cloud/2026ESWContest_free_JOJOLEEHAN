#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <atomic>
#include <stdarg.h>


// =====================================================
// NAVIS ESP32 - FreeRTOS 버전
//
// 기존 navis_esp32.ino(슈퍼루프)를 태스크 구조로 나눈 버전.
// 핀, BLE/UART 메시지 프로토콜, 동작 파라미터는 기존과 같다.
//
// 태스크 구성 (전부 core 1, BLE 스택은 core 0에서 따로 돈다)
//
//  우선순위 5  obstacleTask    TFmini 10ms 주기, 장애물 확정/해제 판단
//  우선순위 4  motorTask       모터를 만지는 "유일한" 태스크
//                              장애물 회피 / 횡단보도 / 신호 상태를 모두 소유
//  우선순위 3  piUartTask      Raspberry Pi 줄 단위 수신 -> 이벤트
//  우선순위 2  bleTxTask       BLE notify 전송 전담
//  우선순위 1  sideSensorTask  VL53L1X 좌/우 측정 (I2C 전담)
//
//  BLE RX 콜백(BLE 스택 태스크)은 명령을 파싱해서 큐에 넣기만 한다.
//
// 기존 코드와 가장 큰 차이
//  1) 모터 제어가 motorTask 한 곳으로 모였다.
//     기존에는 BLE 콜백(guideLeft 등)과 loop(역토크 등)가
//     서로 다른 태스크에서 동시에 ledcWrite를 할 수 있었다.
//  2) 장애물 확정은 진행 중인 길안내 / 빨간불 역토크를 바로 끊는다.
//     기존에는 역토크 delay 동안 TFmini를 아예 읽지 못했다.
//  3) TFmini를 항상 10ms마다 읽기 때문에 flushTFmini()가 필요 없다.
// =====================================================


// =====================================================
// MDD10A
// =====================================================

#define M1_DIR 25
#define M1_PWM 27

#define M2_DIR 32
#define M2_PWM 14


// =====================================================
// TFmini
//
// TFmini TX(초록) -> ESP32 GPIO16
// TFmini RX(흰색)  -> GPIO17 (현재는 연결 안 해도 됨)
// =====================================================

#define TF_RX 16
#define TF_TX 17

HardwareSerial TFSerial(2);


// =====================================================
// VL53L1X LEFT / RIGHT
//
// 두 센서는 기본 I2C 주소가 0x29로 같기 때문에
// XSHUT으로 하나씩 켜서 주소를 바꿔 사용한다.
//
// LEFT  XSHUT -> GPIO23, address 0x30
// RIGHT XSHUT -> GPIO26, address 0x31
// SDA -> GPIO21 (공유)
// SCL -> GPIO22 (공유)
// =====================================================

#define I2C_SDA 21
#define I2C_SCL 22

#define VL_LEFT_XSHUT 23
#define VL_RIGHT_XSHUT 26

#define VL_LEFT_ADDR 0x30
#define VL_RIGHT_ADDR 0x31

VL53L1X vlLeft;
VL53L1X vlRight;

bool vlLeftOK = false;
bool vlRightOK = false;


// =====================================================
// Raspberry Pi UART
//
// Raspberry Pi TX -> ESP32 GPIO18
// Raspberry Pi RX <- ESP32 GPIO19
//
// ESP32와 Raspberry Pi GND 반드시 공통
// =====================================================

#define PI_RX 18
#define PI_TX 19

HardwareSerial PiSerial(1);


// =====================================================
// BLE UUID
// Flutter 앱과 반드시 같아야 함
// =====================================================

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLECharacteristic* txCharacteristic = nullptr;


// =====================================================
// 장애물 설정
//
// 앱:
// 100cm 이하 -> 화면에 "장애물 감지"
//
// ESP32:
// 40cm 이하 -> 역토크 3회
// 50cm 이상 -> 다시 장애물 감지 가능
// =====================================================

const int OBSTACLE_DISTANCE = 40;
const int RESET_DISTANCE = 50;

// TFmini 40cm 이하가 3회 연속 들어와야 실제 장애물로 판단
const int OBSTACLE_CONFIRM_COUNT = 3;

// 좌/우 VL53L1X 판단 기준
const int SIDE_BLOCK_DISTANCE = 30;
const int DIRECTION_MARGIN = 20;


// =====================================================
// 장애물 회피 유도
//
// 역토크 후 고른 방향(좌/우)으로, 정면이 트일 때
// (TFmini RESET_DISTANCE 이상)까지 계속 당겨 준다.
// =====================================================

const int AVOID_PWM = 120;          // 장애물 회피: 강한 방향 유도
const int AVOID_PULSE_TIME = 400;   // 한 번 당기는 시간 (ms)
const int AVOID_REPEAT_MS = 900;    // 당김 시작 간격 (ms)
const int AVOID_MAX_PULSES = 8;     // 이만큼 당겨도 정면이 막혀 있으면 "길 막힘"

enum AvoidState {
  AVOID_NONE,
  AVOID_LEFT,
  AVOID_RIGHT,
  AVOID_BLOCKED
};


// =====================================================
// 모터 PWM 램프업 설정
// 모든 모터 동작 시작 시 0 -> 목표 PWM으로 약 100ms 동안 증가
// =====================================================

const int MOTOR_RAMP_TIME_MS = 100;
const int MOTOR_RAMP_STEPS = 5;


// =====================================================
// 방향 유도 모터 설정
// =====================================================

const int GUIDE_PWM = 100;
const int GUIDE_TIME = 250;


// =====================================================
// 역토크 설정
// =====================================================

const int BRAKE_PWM = 100;

const int BRAKE_ON_TIME = 300;
const int BRAKE_OFF_TIME = 200;

const int BRAKE_COUNT = 3;

// 빨간불로 바뀌면 역토크를 이 횟수만큼 줘서 "멈추세요"를 몸으로 알린다.
// (장애물 경고 3회와 구분되도록 2회)
const int RED_BRAKE_COUNT = 2;


// =====================================================
// 횡단 중 방향 보정 (Raspberry Pi CROSS_MOTOR:L/R/CENTER)
//
// "펄스" 방식:
//  - Pi가 L/R을 보내면 딱 한 번만 당긴다. (스스로 반복하지 않음)
//  - Pi가 STEER_PULSE_SEC 뒤에 CENTER를 보내면 바로 멈춘다.
//  - CROSS_PULSE_MAX_TIME: CENTER를 못 받았을 때 대비한 안전 상한.
//  - CROSS_TIMEOUT_MS 동안 Pi 소식이 없으면 스스로 멈춘다.
// =====================================================

const int CROSS_PWM = 110;
const int CROSS_PULSE_MAX_TIME = 300;  // 램프업 이후 최대 유지 시간 (ms)
const int CROSS_TIMEOUT_MS = 1000;     // Pi 메시지 끊김 판단 (ms)


// =====================================================
// FreeRTOS 설정
// =====================================================

#define APP_CORE 1

const int OBSTACLE_PERIOD_MS = 10;   // TFmini 확인 주기 (TFmini 기본 출력 100Hz)
const int MOTOR_TICK_MS = 10;        // motorTask 타이머(펄스 종료 등) 확인 주기
const int PI_POLL_MS = 5;            // Pi UART 확인 주기
const int SIDE_POLL_MS = 20;         // VL53L1X dataReady 확인 주기
const int SIDE_STALE_MS = 300;       // 이 시간 동안 새 값이 없으면 -1 (읽기 실패)
const int REPORT_MS = 300;           // 앱으로 DIST / SIDE 보내는 주기

const int MOTOR_QUEUE_LEN = 32;      // 역토크(최대 약 1.8초) 동안 쌓일 이벤트까지 여유 있게
const int BLE_TX_QUEUE_LEN = 20;

// 1이면 5초마다 TFmini 확인 주기 최대값과 태스크별 남은 스택을 출력한다.
// 리팩토링 전후 비교 / 스택 크기 조정용.
#define DEBUG_MONITOR 1


// =====================================================
// 태스크 사이에서 주고받는 데이터
// =====================================================

enum EventType : uint8_t {

  // obstacleTask -> motorTask
  EV_OBSTACLE_CONFIRMED,
  EV_OBSTACLE_CLEAR,

  // Flutter(BLE) -> motorTask
  EV_APP_LEFT,
  EV_APP_RIGHT,
  EV_APP_FORWARD,
  EV_APP_STOP,
  EV_APP_DISCONNECTED,

  // Raspberry Pi -> motorTask
  EV_PI_CROSSWALK_ON,
  EV_PI_CROSSWALK_OFF,
  EV_PI_LIGHT_RED,
  EV_PI_LIGHT_GREEN,
  EV_PI_LIGHT_GREEN_WAIT,
  EV_PI_CROSSING_START,
  EV_PI_CROSSING_END,
  EV_PI_CROSS_L,
  EV_PI_CROSS_R,
  EV_PI_CROSS_CENTER
};

struct MotorEvent {
  EventType type;
  float value;        // 앱 L/R 각도, 장애물 거리(cm)
  uint32_t stampUs;   // 이벤트를 만든 시각 (반응 지연 측정용)
};

struct BleMsg {
  char text[104];     // Pi 메시지(최대 100자)를 그대로 전달할 수 있는 크기
};

QueueHandle_t motorQueue = nullptr;
QueueHandle_t bleTxQueue = nullptr;
SemaphoreHandle_t logMutex = nullptr;

TaskHandle_t obstacleTaskHandle = nullptr;
TaskHandle_t motorTaskHandle = nullptr;
TaskHandle_t piTaskHandle = nullptr;
TaskHandle_t bleTxTaskHandle = nullptr;
TaskHandle_t sideTaskHandle = nullptr;

// 여러 태스크가 같이 보는 값은 atomic으로 둔다.
std::atomic<bool> bleConnected(false);
std::atomic<int> sideLeftCm(-1);         // sideSensorTask가 쓰고 motorTask가 읽음
std::atomic<int> sideRightCm(-1);
std::atomic<uint32_t> obstacleMaxPeriodUs(0);

// Flutter에서 받은 목표 방위각 (현재는 로그 확인용)
float targetBearing = -1;


// =====================================================
// 로그
//
// 여러 태스크가 동시에 Serial.print를 여러 번 나눠 부르면
// 줄이 섞이기 때문에, 한 줄을 만들어서 mutex 안에서 한 번에 찍는다.
// =====================================================

// printf 형식 검사: %d 자리에 문자열을 넣는 실수 등을 컴파일러가 잡아준다.
void logLine(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void sendBLEf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

void logLine(const char* fmt, ...) {

  char buf[160];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (logMutex != nullptr) {
    xSemaphoreTake(logMutex, portMAX_DELAY);
  }

  Serial.println(buf);

  if (logMutex != nullptr) {
    xSemaphoreGive(logMutex);
  }
}


// =====================================================
// BLE Notify 요청
//
// 어느 태스크에서 불러도 된다. 실제 notify는 bleTxTask가 한다.
// =====================================================

void sendBLE(const char* message) {

  if (!bleConnected) {
    return;
  }

  BleMsg msg;

  strncpy(msg.text, message, sizeof(msg.text) - 1);
  msg.text[sizeof(msg.text) - 1] = '\0';

  if (xQueueSend(bleTxQueue, &msg, 0) != pdTRUE) {
    logLine("[BLE TX] queue full, dropped: %s", message);
  }
}

void sendBLEf(const char* fmt, ...) {

  char buf[sizeof(BleMsg::text)];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  sendBLE(buf);
}


// =====================================================
// motorTask로 이벤트 보내기
//
// 큐가 꽉 차 있으면 기다리지 않고 버린다. (보내는 쪽이 멈추지 않게)
// 장애물 이벤트만 obstacleTask에서 따로 큐 맨 앞에 넣는다.
// =====================================================

bool postEvent(EventType type, float value = 0) {

  MotorEvent ev = { type, value, (uint32_t)micros() };

  if (xQueueSend(motorQueue, &ev, 0) != pdTRUE) {
    logLine("[EVENT] queue full, dropped type=%d", (int)type);
    return false;
  }

  return true;
}


// #####################################################
//
// 여기부터 motorTask 전용
//
// 아래 변수와 함수는 motorTask 안에서만 쓴다.
// 다른 태스크는 postEvent()로 "요청"만 보내기 때문에
// mutex 없이도 상태가 꼬이지 않는다.
//
// #####################################################

bool obstacleTriggered = false;

bool crosswalkMode = false;
bool trafficRed = false;

AvoidState avoidState = AVOID_NONE;

int avoidPulseCount = 0;
unsigned long lastAvoidPulseAt = 0;
unsigned long avoidPulseEndAt = 0;
bool avoidPulseRunning = false;

char crossSteer = 'C';               // 'L' / 'R' / 'C'
unsigned long lastCrossMsgAt = 0;
unsigned long crossPulseEndAt = 0;
bool crossPulseRunning = false;
bool crossPulseStartNow = false;


// =====================================================
// 대기
//
// abortable = true : 기다리는 도중 obstacleTask가 xTaskNotifyGive()를 보내면
//                    바로 깨어나서 false를 돌려준다. (장애물이 최우선)
// abortable = false: 끝까지 기다린다.
//                    (장애물 역토크 자체처럼 끊으면 안 되는 동작)
//
// 기존 delay()를 이걸로 바꾼 것이 핵심이다.
// =====================================================

bool waitMs(int ms, bool abortable) {

  if (!abortable) {
    vTaskDelay(pdMS_TO_TICKS(ms));
    return true;
  }

  return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms)) == 0;
}


// =====================================================
// 모터 정지
// =====================================================

void stopMotors() {
  ledcWrite(M1_PWM, 0);
  ledcWrite(M2_PWM, 0);
}


// =====================================================
// PWM 램프업
// 갑자기 목표 PWM을 넣지 않고 0 -> 목표값으로 단계적으로 증가
// 중간에 끊기면 PWM을 0으로 두고 false
// =====================================================

bool rampSingleMotor(int pwmPin, int targetPwm, bool abortable) {

  const int stepDelay = MOTOR_RAMP_TIME_MS / MOTOR_RAMP_STEPS;

  for (int i = 1; i <= MOTOR_RAMP_STEPS; i++) {

    int pwm = (targetPwm * i) / MOTOR_RAMP_STEPS;
    ledcWrite(pwmPin, pwm);

    if (!waitMs(stepDelay, abortable)) {
      ledcWrite(pwmPin, 0);
      return false;
    }
  }

  return true;
}

bool rampBothMotors(int targetPwm, bool abortable) {

  const int stepDelay = MOTOR_RAMP_TIME_MS / MOTOR_RAMP_STEPS;

  for (int i = 1; i <= MOTOR_RAMP_STEPS; i++) {

    int pwm = (targetPwm * i) / MOTOR_RAMP_STEPS;
    ledcWrite(M1_PWM, pwm);
    ledcWrite(M2_PWM, pwm);

    if (!waitMs(stepDelay, abortable)) {
      stopMotors();
      return false;
    }
  }

  return true;
}


// =====================================================
// LEFT / RIGHT 유도
//
// 사용자가 앞으로 끌고 있기 때문에
// LEFT : 오른쪽 바퀴 M2에 짧게 힘을 줘서 왼쪽 방향 유도
// RIGHT: 왼쪽 바퀴 M1에 짧게 힘을 줘서 오른쪽 방향 유도
//
// 기존처럼 끝날 때까지(약 350ms) 기다리지만,
// 그 사이 장애물이 확정되면 바로 끊긴다.
// =====================================================

void guidePulse(char side, float angle) {

  const char* name = (side == 'L') ? "LEFT" : "RIGHT";

  if (obstacleTriggered) {
    logLine("[MOTOR] %s ignored - obstacle", name);
    return;
  }

  if (crosswalkMode) {
    logLine("[MOTOR] %s ignored - crosswalk mode", name);
    return;
  }

  logLine("[MOTOR] %s | angle=%.2f", name, angle);

  int pwmPin  = (side == 'L') ? M2_PWM : M1_PWM;
  int dirPin  = (side == 'L') ? M2_DIR : M1_DIR;
  int idlePin = (side == 'L') ? M1_PWM : M2_PWM;

  ledcWrite(idlePin, 0);
  digitalWrite(dirPin, LOW);

  bool done = rampSingleMotor(pwmPin, GUIDE_PWM, true) &&
              waitMs(GUIDE_TIME, true);

  ledcWrite(pwmPin, 0);

  if (!done) {
    logLine("[MOTOR] %s aborted - obstacle", name);
    return;
  }

  sendBLE(side == 'L' ? "MOTOR:L" : "MOTOR:R");
}

void guideLeft(float angle) {
  guidePulse('L', angle);
}

void guideRight(float angle) {
  guidePulse('R', angle);
}


// =====================================================
// 직진
//
// 현재 장치는 사용자가 직접 앞으로 끌기 때문에
// F 명령에서는 모터를 돌리지 않음.
// =====================================================

void guideForward() {

  if (obstacleTriggered) {
    return;
  }

  stopMotors();

  logLine("[MOTOR] FORWARD / NEUTRAL");
}


// =====================================================
// 역토크 count회 (장애물 경고 / 빨간불 정지 알림 공용)
//
// 장애물 경고는 abortable=false (끝까지)
// 빨간불 알림은 abortable=true  (도중에 장애물이 오면 그쪽 우선)
// =====================================================

bool reverseTorquePulses(int count, bool abortable) {

  // 진행 방향 반대
  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);

  bool done = true;

  for (int i = 0; i < count; i++) {

    logLine("[BRAKE] %d/%d", i + 1, count);

    // 역토크 ON: 두 모터를 동시에 0 -> BRAKE_PWM으로 램프업
    if (!rampBothMotors(BRAKE_PWM, abortable) ||
        !waitMs(BRAKE_ON_TIME, abortable)) {
      done = false;
      break;
    }

    // OFF
    stopMotors();

    if (!waitMs(BRAKE_OFF_TIME, abortable)) {
      done = false;
      break;
    }
  }

  stopMotors();

  // 일반 방향으로 복귀
  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);

  logLine(done ? "*** BRAKE END ***" : "*** BRAKE ABORTED - obstacle ***");

  return done;
}


// 장애물 역토크 3회
void obstacleWarning() {

  logLine("");
  logLine("*** OBSTACLE WARNING ***");

  reverseTorquePulses(BRAKE_COUNT, false);
}


// =====================================================
// 횡단보도 보정 상태 변경
// Raspberry Pi -> CROSS_MOTOR:L / R / CENTER
//
// 여기서는 상태만 바꾸고, 실제 당김은 updateCrosswalkSteering()이
// 한 번 실행한다. 앱에는 방향이 바뀔 때만 알린다.
// =====================================================

void setCrossSteer(char dir) {

  lastCrossMsgAt = millis();

  // 같은 방향이 반복으로 와도 다시 당기지 않는다.
  // (Pi는 펄스마다 CENTER를 사이에 끼워서 보낸다)
  if (dir == crossSteer) {
    return;
  }

  crossSteer = dir;

  // 당기던 중이면 끊는다. (CENTER가 오면 여기서 바로 멈춤)
  if (crossPulseRunning && !obstacleTriggered) {
    stopMotors();
  }

  crossPulseRunning = false;

  // L/R일 때만 한 번 당기기 예약
  crossPulseStartNow = (dir == 'L' || dir == 'R');

  logLine("[CROSS MOTOR] %c", dir);

  if (dir == 'L') {
    sendBLE("CROSS_MOTOR:L");
  }
  else if (dir == 'R') {
    sendBLE("CROSS_MOTOR:R");
  }
  else {
    sendBLE("CROSS_MOTOR:CENTER");
  }
}


// 횡단 보정 멈춤 (장애물 회피 중이면 그 모터 동작은 건드리지 않음)
void stopCrossSteer() {

  crossSteer = 'C';

  if (!obstacleTriggered) {
    stopMotors();
  }

  crossPulseRunning = false;
  crossPulseStartNow = false;
}


// =====================================================
// 횡단 보정 펄스 처리 (motorTask에서 매번 호출)
//
// L/R을 받으면 한 번만 당기고, 스스로 반복하지 않는다.
// =====================================================

void updateCrosswalkSteering() {

  unsigned long now = millis();

  // 안전 상한: CENTER를 못 받아도 최대 시간이 지나면 끄기
  if (crossPulseRunning && (long)(now - crossPulseEndAt) >= 0) {

    if (!obstacleTriggered) {
      stopMotors();
    }

    crossPulseRunning = false;
  }

  // 장애물 회피가 최우선. 대기 중이던 당김도 버린다.
  if (obstacleTriggered) {
    crossPulseRunning = false;
    crossPulseStartNow = false;
    return;
  }

  if (!crosswalkMode || trafficRed || (crossSteer != 'L' && crossSteer != 'R')) {
    crossPulseStartNow = false;
    return;
  }

  // Pi 메시지가 끊기면 멈춤 (Pi 멈춤 / 케이블 빠짐 대비)
  if (now - lastCrossMsgAt > (unsigned long)CROSS_TIMEOUT_MS) {

    logLine("[CROSS MOTOR] Pi timeout -> CENTER");

    stopCrossSteer();
    sendBLE("CROSS_MOTOR:CENTER");
    return;
  }

  if (crossPulseRunning || !crossPulseStartNow) {
    return;
  }

  crossPulseStartNow = false;

  bool ramped;

  if (crossSteer == 'L') {
    // 오른쪽 바퀴 M2로 왼쪽 유도
    ledcWrite(M1_PWM, 0);
    digitalWrite(M2_DIR, LOW);
    ramped = rampSingleMotor(M2_PWM, CROSS_PWM, true);
  }
  else {
    // 왼쪽 바퀴 M1로 오른쪽 유도
    ledcWrite(M2_PWM, 0);
    digitalWrite(M1_DIR, LOW);
    ramped = rampSingleMotor(M1_PWM, CROSS_PWM, true);
  }

  // 램프업 도중 장애물 확정 -> 큐 맨 앞의 장애물 이벤트가 이어서 처리됨
  if (!ramped) {
    stopMotors();
    return;
  }

  crossPulseRunning = true;

  // 램프업이 끝난 시점부터 최대 유지시간 계산
  crossPulseEndAt = millis() + CROSS_PULSE_MAX_TIME;

  // 기존에는 여기서 readRaspberryPi()를 한 번 더 불렀지만,
  // 램프업 동안 온 CENTER는 큐에 들어 있으므로 바로 다음에 처리된다.
}


// =====================================================
// 장애물 회피용 당김 (non-blocking)
//
// 끝나는 시각만 기록하고 돌아가서,
// 당기는 동안에도 다른 이벤트(장애물 해제 등)를 처리한다.
//
// LEFT : 오른쪽 바퀴 M2 구동
// RIGHT: 왼쪽 바퀴 M1 구동
// =====================================================

void startAvoidPulse(AvoidState side) {

  bool left = side == AVOID_LEFT;

  logLine("[AVOID] PULL %s #%d", left ? "LEFT" : "RIGHT", avoidPulseCount + 1);

  if (left) {
    ledcWrite(M1_PWM, 0);
    digitalWrite(M2_DIR, LOW);
    rampSingleMotor(M2_PWM, AVOID_PWM, false);
  }
  else {
    ledcWrite(M2_PWM, 0);
    digitalWrite(M1_DIR, LOW);
    rampSingleMotor(M1_PWM, AVOID_PWM, false);
  }

  avoidPulseRunning = true;
  lastAvoidPulseAt = millis();
  avoidPulseEndAt = lastAvoidPulseAt + AVOID_PULSE_TIME;
  avoidPulseCount++;

  sendBLE(left ? "MOTOR:L" : "MOTOR:R");
}


// 당기는 시간이 끝났으면 모터를 끈다.
void updateAvoidPulse() {

  if (avoidPulseRunning && (long)(millis() - avoidPulseEndAt) >= 0) {
    stopMotors();
    avoidPulseRunning = false;
  }
}


// =====================================================
// 회피 방향 고르기
//
// - 30cm 미만(또는 읽기 실패)인 쪽은 막힌 것으로 본다.
// - 양쪽 다 막힘 -> BLOCKED
// - 이미 피하던 쪽이 아직 열려 있으면 유지 (좌우 왔다갔다 방지)
// - 한쪽만 열림 -> 그쪽
// - 둘 다 열림 -> 20cm 넘게 더 넓은 쪽, 비슷하면 오른쪽(우측 보행)
// =====================================================

AvoidState chooseAvoidSide(int leftDistance, int rightDistance) {

  bool leftOpen = leftDistance >= SIDE_BLOCK_DISTANCE;
  bool rightOpen = rightDistance >= SIDE_BLOCK_DISTANCE;

  if (!leftOpen && !rightOpen) {
    return AVOID_BLOCKED;
  }

  if (avoidState == AVOID_LEFT && leftOpen) {
    return AVOID_LEFT;
  }

  if (avoidState == AVOID_RIGHT && rightOpen) {
    return AVOID_RIGHT;
  }

  if (leftOpen && !rightOpen) {
    return AVOID_LEFT;
  }

  if (rightOpen && !leftOpen) {
    return AVOID_RIGHT;
  }

  if (leftDistance > rightDistance + DIRECTION_MARGIN) {
    return AVOID_LEFT;
  }

  return AVOID_RIGHT;
}


// 회피 상태가 바뀔 때만 앱에 알린다. (앱이 음성으로 안내)
void setAvoidState(AvoidState next) {

  if (next == avoidState) {
    return;
  }

  avoidState = next;

  if (next == AVOID_LEFT) {
    sendBLE("AVOID:L");
  }
  else if (next == AVOID_RIGHT) {
    sendBLE("AVOID:R");
  }
  else if (next == AVOID_BLOCKED) {
    stopMotors();
    avoidPulseRunning = false;
    sendBLE("AVOID:BLOCKED");
  }
}


// =====================================================
// 장애물 확정 (EV_OBSTACLE_CONFIRMED)
//
// 1) 역토크 3회로 멈춤 알림
// 2) 좌/우 VL53L1X로 회피 방향 결정 (chooseAvoidSide)
// 3) 그 방향으로 첫 당김 시작
//    -> 이후 updateObstacleAvoidance()가 정면이 트일 때까지 반복
// =====================================================

void handleObstacleConfirmed(const MotorEvent& ev) {

  // 대기 중이 아닐 때 받은 중단 신호가 남아 있으면 정리한다.
  // (안 그러면 다음 abortable 대기가 괜히 바로 끝난다)
  ulTaskNotifyTake(pdTRUE, 0);

  stopMotors();

  logLine("[LATENCY] obstacle confirm -> motor stop: %lu us",
       (unsigned long)(micros() - ev.stampUs));

  obstacleTriggered = true;

  // 대기 중이던 횡단 보정도 버린다.
  crossPulseRunning = false;
  crossPulseStartNow = false;

  logLine("");
  logLine("[AVOID] FRONT OBSTACLE CONFIRMED -> BRAKE FIRST");

  // Flutter 앱에 장애물 음성 안내 요청.
  // 장애물 1건당 역토크 시작 직전에 딱 한 번만 전송한다.
  sendBLE("OBSTACLE_BRAKE");

  // BLE Notify가 앱으로 전달될 시간을 아주 짧게 확보.
  vTaskDelay(pdMS_TO_TICKS(50));

  // 역토크 3회
  obstacleWarning();

  // 역토크 직후 잠깐 안정화
  vTaskDelay(pdMS_TO_TICKS(100));

  // 기존의 flushTFmini()는 필요 없다.
  // obstacleTask가 역토크 동안에도 10ms마다 계속 읽고 있다.

  // 좌우 거리: sideSensorTask가 계속 갱신하는 최신 값
  int leftDistance = sideLeftCm;
  int rightDistance = sideRightCm;

  logLine("");
  logLine("========== OBSTACLE AVOID ==========");
  logLine("FRONT = %d cm", (int)ev.value);
  logLine("LEFT  = %d cm", leftDistance);
  logLine("RIGHT = %d cm", rightDistance);

  avoidPulseCount = 0;
  avoidPulseRunning = false;
  avoidState = AVOID_NONE;

  AvoidState side = chooseAvoidSide(leftDistance, rightDistance);

  setAvoidState(side);

  if (side == AVOID_BLOCKED) {
    // 양쪽 다 막힘 (또는 센서 읽기 실패). 역토크는 이미 실행됨.
    logLine("[AVOID] BOTH SIDES BLOCKED -> STOP");
    return;
  }

  startAvoidPulse(side);
}


// =====================================================
// 장애물 해제 (EV_OBSTACLE_CLEAR)
// 정면이 50cm 이상으로 다시 멀어짐
// =====================================================

void handleObstacleClear() {

  obstacleTriggered = false;

  // 회피 유도 종료
  stopMotors();
  avoidPulseRunning = false;
  avoidPulseCount = 0;
  avoidState = AVOID_NONE;

  sendBLE("OBSTACLE_CLEAR");

  logLine("[TFmini] Obstacle cleared");
}


// =====================================================
// 장애물 회피 중 반복 (obstacleTriggered일 때 매번 호출)
//
// 정면이 아직 막혀 있으면 AVOID_REPEAT_MS마다 다시 당긴다.
// AVOID_MAX_PULSES번 당겨도 정면이 안 트이면 "길 막힘"으로
// 역토크를 한 번 더 주고 멈춘다.
// =====================================================

void updateObstacleAvoidance() {

  updateAvoidPulse();

  if (avoidState != AVOID_LEFT && avoidState != AVOID_RIGHT) {
    return;
  }

  if (avoidPulseRunning) {
    return;
  }

  if (millis() - lastAvoidPulseAt < (unsigned long)AVOID_REPEAT_MS) {
    return;
  }

  if (avoidPulseCount >= AVOID_MAX_PULSES) {

    logLine("[AVOID] front still blocked -> BLOCKED");

    setAvoidState(AVOID_BLOCKED);

    obstacleWarning();

    return;
  }

  AvoidState side = chooseAvoidSide(sideLeftCm, sideRightCm);

  setAvoidState(side);

  if (side == AVOID_LEFT || side == AVOID_RIGHT) {
    startAvoidPulse(side);
  }
}


// =====================================================
// Raspberry Pi 이벤트 처리 (기존 handlePiMessage의 상태 변경 부분)
// =====================================================

void handlePiEvent(EventType type) {

  switch (type) {

    // 횡단보도 감지
    case EV_PI_CROSSWALK_ON:
      crosswalkMode = true;
      stopCrossSteer();
      sendBLE("CROSSWALK:1");
      break;

    // 횡단보도 사라짐
    case EV_PI_CROSSWALK_OFF:
      crosswalkMode = false;
      trafficRed = false;
      stopCrossSteer();
      sendBLE("CROSSWALK:0");
      break;

    // 빨간불
    case EV_PI_LIGHT_RED: {

      // 빨간불로 "바뀐" 순간에만 역토크 (같은 메시지 반복 시 다시 안 줌)
      bool turnedRed = !trafficRed;

      crosswalkMode = true;
      trafficRed = true;

      stopCrossSteer();

      sendBLE("LIGHT:RED");

      // 장애물 회피 중에는 그쪽 동작을 우선
      if (turnedRed && !obstacleTriggered) {

        logLine("[TRAFFIC] RED -> stop warning");

        // 기존과 달리, 역토크 도중 장애물이 확정되면 바로 끊고 장애물 처리로 넘어간다.
        reverseTorquePulses(RED_BRAKE_COUNT, true);
      }
      break;
    }

    // 초록불
    case EV_PI_LIGHT_GREEN:
      crosswalkMode = true;
      trafficRed = false;
      stopCrossSteer();
      sendBLE("LIGHT:GREEN");
      break;

    // 초록불이 이미 켜져 있음 (빨간불을 못 봄)
    // 남은 시간을 모르므로 건너지 않고 다음 신호를 기다린다.
    case EV_PI_LIGHT_GREEN_WAIT:
      crosswalkMode = true;
      trafficRed = true;
      stopCrossSteer();
      sendBLE("LIGHT:GREEN_WAIT");
      break;

    // 횡단 시작
    case EV_PI_CROSSING_START:
      crosswalkMode = true;
      trafficRed = false;
      stopCrossSteer();
      sendBLE("CROSSING_START");
      break;

    // 횡단 종료
    case EV_PI_CROSSING_END:
      crosswalkMode = false;
      trafficRed = false;
      stopCrossSteer();
      sendBLE("CROSSING_END");
      break;

    // 횡단 중 방향 보정
    case EV_PI_CROSS_L:
      setCrossSteer('L');
      break;

    case EV_PI_CROSS_R:
      setCrossSteer('R');
      break;

    case EV_PI_CROSS_CENTER:
      setCrossSteer('C');
      break;

    default:
      break;
  }
}


void handleMotorEvent(const MotorEvent& ev) {

  switch (ev.type) {

    case EV_OBSTACLE_CONFIRMED:
      handleObstacleConfirmed(ev);
      break;

    case EV_OBSTACLE_CLEAR:
      handleObstacleClear();
      break;

    case EV_APP_LEFT:
      guideLeft(ev.value);
      break;

    case EV_APP_RIGHT:
      guideRight(ev.value);
      break;

    case EV_APP_FORWARD:
      guideForward();
      break;

    case EV_APP_STOP:
      stopMotors();
      sendBLE("MOTOR:STOP");
      break;

    case EV_APP_DISCONNECTED:
      // 기존 onDisconnect()의 stopMotors()를 여기로 옮겼다.
      stopMotors();
      crossPulseRunning = false;
      avoidPulseRunning = false;
      break;

    default:
      handlePiEvent(ev.type);
      break;
  }
}


// =====================================================
// motorTask
//
// 이벤트가 오면 처리하고, 안 와도 MOTOR_TICK_MS마다 깨어나서
// 펄스 종료 / 회피 반복 / Pi 타임아웃 같은 시간 조건을 확인한다.
// (기존 loop()에서 매번 부르던 update 함수들)
// =====================================================

void motorTask(void*) {

  MotorEvent ev;

  for (;;) {

    if (xQueueReceive(motorQueue, &ev, pdMS_TO_TICKS(MOTOR_TICK_MS)) == pdTRUE) {
      handleMotorEvent(ev);
    }

    if (obstacleTriggered) {
      updateObstacleAvoidance();
    }

    updateCrosswalkSteering();
  }
}

// #####################################################
// motorTask 전용 끝
// #####################################################


// =====================================================
// TFmini 읽기 (obstacleTask 전용)
//
// 버퍼에 쌓인 프레임을 모두 읽고 "가장 최근" 거리를 돌려준다.
// =====================================================

int readTFmini() {

  int latest = -1;

  while (TFSerial.available() >= 9) {

    // 첫 번째 헤더
    if (TFSerial.read() != 0x59) {
      continue;
    }

    // 두 번째 헤더 (아니면 버리지 않고 다음 바이트부터 다시 찾음)
    if (TFSerial.peek() != 0x59) {
      continue;
    }

    TFSerial.read();

    uint8_t buf[9];

    buf[0] = 0x59;
    buf[1] = 0x59;

    // available() >= 9 였으므로 나머지 7바이트는 이미 들어와 있음
    for (int i = 2; i < 9; i++) {
      buf[i] = TFSerial.read();
    }

    // checksum
    uint16_t checksum = 0;

    for (int i = 0; i < 8; i++) {
      checksum += buf[i];
    }

    checksum &= 0xFF;

    if (checksum != buf[8]) {
      logLine("[TFmini] checksum error");
      continue;
    }

    // 거리 cm
    latest = buf[2] | (buf[3] << 8);
  }

  return latest;
}


// =====================================================
// obstacleTask (우선순위 최고)
//
// 10ms마다 TFmini를 읽고 장애물 확정 / 해제를 판단한다.
// 모터는 직접 건드리지 않는다.
//
// 확정 시:
//  1) 장애물 이벤트를 motorQueue "맨 앞"에 넣고
//  2) xTaskNotifyGive()로 motorTask의 진행 중 대기(길안내, 빨간불 역토크)를 끊는다.
//
// 이벤트를 못 넣으면(큐가 꽉 참) 상태를 바꾸지 않고 다음 주기에 다시 시도한다.
// =====================================================

void obstacleTask(void*) {

  int obstacleCount = 0;
  bool triggered = false;

  unsigned long lastDistancePrint = 0;
  unsigned long lastDistanceSend = 0;

  uint32_t lastRunUs = micros();

  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {

    // 실제 확인 주기 측정 (기존 loop와 비교용)
    uint32_t nowUs = micros();
    uint32_t period = nowUs - lastRunUs;
    lastRunUs = nowUs;

    if (period > obstacleMaxPeriodUs) {
      obstacleMaxPeriodUs = period;
    }

    int distance = readTFmini();

    if (distance > 0) {

      unsigned long now = millis();

      if (now - lastDistancePrint >= (unsigned long)REPORT_MS) {
        logLine("[TFmini] %d cm", distance);
        lastDistancePrint = now;
      }

      // 앱에 거리 전송
      if (now - lastDistanceSend >= (unsigned long)REPORT_MS) {
        sendBLEf("DIST:%d", distance);
        lastDistanceSend = now;
      }

      // TFmini 40cm 이하가 3회 연속 들어오면 장애물 확정
      if (distance <= OBSTACLE_DISTANCE) {

        if (obstacleCount < OBSTACLE_CONFIRM_COUNT) {
          obstacleCount++;
          logLine("[OBSTACLE COUNT] %d/%d", obstacleCount, OBSTACLE_CONFIRM_COUNT);
        }
      }
      else {
        obstacleCount = 0;
      }

      if (obstacleCount >= OBSTACLE_CONFIRM_COUNT && !triggered) {

        MotorEvent ev = { EV_OBSTACLE_CONFIRMED, (float)distance, (uint32_t)micros() };

        if (xQueueSendToFront(motorQueue, &ev, pdMS_TO_TICKS(20)) == pdTRUE) {

          triggered = true;

          // 진행 중인 모터 동작 즉시 중단
          xTaskNotifyGive(motorTaskHandle);

          // Flutter에도 전달
          sendBLEf("OBSTACLE:%d", distance);
        }
        else {
          logLine("[OBSTACLE] motor queue full - retry");
        }
      }

      // 정면이 50cm 이상으로 다시 멀어지면 해제
      if (distance >= RESET_DISTANCE && triggered) {

        MotorEvent ev = { EV_OBSTACLE_CLEAR, (float)distance, (uint32_t)micros() };

        if (xQueueSend(motorQueue, &ev, pdMS_TO_TICKS(20)) == pdTRUE) {
          triggered = false;
          obstacleCount = 0;
        }
        else {
          logLine("[OBSTACLE] motor queue full - retry clear");
        }
      }
    }

    // 정확히 OBSTACLE_PERIOD_MS 주기로 실행 (처리 시간과 무관)
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(OBSTACLE_PERIOD_MS));
  }
}


// =====================================================
// VL53L1X 2개 초기화 (setup에서, 태스크 시작 전에 한 번)
// =====================================================

void setupVL53L1X() {

  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(VL_LEFT_XSHUT, OUTPUT);
  pinMode(VL_RIGHT_XSHUT, OUTPUT);

  // 둘 다 끄기
  digitalWrite(VL_LEFT_XSHUT, LOW);
  digitalWrite(VL_RIGHT_XSHUT, LOW);

  delay(20);

  // LEFT 먼저 켜고 0x30으로 주소 변경
  digitalWrite(VL_LEFT_XSHUT, HIGH);
  delay(20);

  vlLeft.setTimeout(100);

  if (vlLeft.init()) {

    vlLeft.setAddress(VL_LEFT_ADDR);
    vlLeft.setDistanceMode(VL53L1X::Long);
    vlLeft.setMeasurementTimingBudget(50000);
    vlLeft.startContinuous(50);

    vlLeftOK = true;

    logLine("[OK] VL53L1X LEFT  addr=0x30");
  }
  else {

    vlLeftOK = false;

    logLine("[ERROR] VL53L1X LEFT init failed");
  }

  // RIGHT 켜고 기본주소 0x29에서 0x31로 변경
  digitalWrite(VL_RIGHT_XSHUT, HIGH);
  delay(20);

  vlRight.setTimeout(100);

  if (vlRight.init()) {

    vlRight.setAddress(VL_RIGHT_ADDR);
    vlRight.setDistanceMode(VL53L1X::Long);
    vlRight.setMeasurementTimingBudget(50000);
    vlRight.startContinuous(50);

    vlRightOK = true;

    logLine("[OK] VL53L1X RIGHT addr=0x31");
  }
  else {

    vlRightOK = false;

    logLine("[ERROR] VL53L1X RIGHT init failed");
  }
}


// =====================================================
// sideSensorTask (I2C 전담)
//
// 기존 readVL53Left()/Right()는 read()가 새 값이 나올 때까지
// 바쁘게 기다렸다(최대 수십 ms). 여기서는 dataReady()일 때만
// read(false)로 바로 가져와서 다른 태스크에 CPU를 양보한다.
//
// SIDE_STALE_MS 동안 새 값이 없으면 -1 (기존 timeout과 같은 의미)
// =====================================================

int updateSideSensor(VL53L1X& sensor, bool ok, unsigned long& lastUpdateAt,
                     std::atomic<int>& out, const char* name) {

  if (!ok) {
    out = -1;
    return -1;
  }

  unsigned long now = millis();

  if (sensor.dataReady()) {
    uint16_t mm = sensor.read(false);
    out = mm / 10;
    lastUpdateAt = now;
  }
  else if (out != -1 && now - lastUpdateAt > (unsigned long)SIDE_STALE_MS) {
    logLine("[VL53 %s] timeout", name);
    out = -1;
  }

  return out;
}

void sideSensorTask(void*) {

  unsigned long leftUpdateAt = 0;
  unsigned long rightUpdateAt = 0;
  unsigned long lastSideDistanceSend = 0;

  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {

    int leftDistance = updateSideSensor(vlLeft, vlLeftOK, leftUpdateAt, sideLeftCm, "LEFT");
    int rightDistance = updateSideSensor(vlRight, vlRightOK, rightUpdateAt, sideRightCm, "RIGHT");

    // 300 ms마다 SIDE:왼쪽cm,오른쪽cm 전송 (읽기 실패 시 -1 -> 앱에서 -- cm)
    if (millis() - lastSideDistanceSend >= (unsigned long)REPORT_MS) {

      logLine("[VL53] LEFT=%d cm | RIGHT=%d cm", leftDistance, rightDistance);

      sendBLEf("SIDE:%d,%d", leftDistance, rightDistance);

      lastSideDistanceSend = millis();
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SIDE_POLL_MS));
  }
}


// =====================================================
// Raspberry Pi 메시지 (piUartTask)
//
// Raspberry Pi에서 아래처럼 보내면 됨. (docs/PROTOCOL.md)
//
// CROSSWALK:1 / CROSSWALK:0
// LIGHT:RED / LIGHT:GREEN / LIGHT:GREEN_WAIT / LIGHT:NONE
// CROSSING_START / CROSSING_END
// CROSS_MOTOR:L / CROSS_MOTOR:R / CROSS_MOTOR:CENTER
//
// 각 메시지 뒤에는 반드시 \n
//
// 여기서는 문자열 -> 이벤트 변환만 하고, 상태 변경은 motorTask가 한다.
// =====================================================

struct PiCommand {
  const char* text;
  EventType type;
};

const PiCommand PI_COMMANDS[] = {
  { "CROSSWALK:1",        EV_PI_CROSSWALK_ON },
  { "CROSSWALK:0",        EV_PI_CROSSWALK_OFF },
  { "LIGHT:RED",          EV_PI_LIGHT_RED },
  { "SIGNAL_RED",         EV_PI_LIGHT_RED },
  { "LIGHT:GREEN",        EV_PI_LIGHT_GREEN },
  { "SIGNAL_GREEN",       EV_PI_LIGHT_GREEN },
  { "LIGHT:GREEN_WAIT",   EV_PI_LIGHT_GREEN_WAIT },
  { "CROSSING_START",     EV_PI_CROSSING_START },
  { "CROSSING_END",       EV_PI_CROSSING_END },
  { "CROSS_MOTOR:L",      EV_PI_CROSS_L },
  { "CROSS_MOTOR:R",      EV_PI_CROSS_R },
  { "CROSS_MOTOR:CENTER", EV_PI_CROSS_CENTER },
};


// 앞뒤 공백 제거 (기존 String::trim()과 같은 역할)
char* trimInPlace(char* s) {

  while (*s == ' ' || *s == '\t') {
    s++;
  }

  int len = strlen(s);

  while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t')) {
    s[--len] = '\0';
  }

  return s;
}


void handlePiLine(char* raw) {

  char* message = trimInPlace(raw);

  if (*message == '\0') {
    return;
  }

  logLine("[PI RX] %s", message);

  for (const PiCommand& cmd : PI_COMMANDS) {
    if (strcmp(message, cmd.text) == 0) {
      postEvent(cmd.type);
      return;
    }
  }

  // 신호등 인식 안 됨: 상태 변화 없이 앱에만 알림
  if (strcmp(message, "LIGHT:NONE") == 0) {
    sendBLE("LIGHT:NONE");
    return;
  }

  // 그 외 Raspberry Pi 메시지는 앱에서도 확인할 수 있도록 그대로 전달
  sendBLE(message);
}


void piUartTask(void*) {

  char line[101];
  int len = 0;

  for (;;) {

    while (PiSerial.available()) {

      char c = PiSerial.read();

      if (c == '\n') {
        line[len] = '\0';
        handlePiLine(line);
        len = 0;
      }
      else if (c != '\r') {

        // 비정상 데이터 방지 (기존과 같이 100자 넘으면 버림)
        if (len >= 100) {
          len = 0;
        }
        else {
          line[len++] = c;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(PI_POLL_MS));
  }
}


// =====================================================
// bleTxTask
//
// sendBLE()가 넣은 메시지를 순서대로 notify한다.
// notify는 BLE 스택 상태에 따라 오래 걸릴 수 있어서
// 모터/센서 태스크가 직접 부르지 않게 따로 뺐다.
// =====================================================

void bleTxTask(void*) {

  BleMsg msg;

  for (;;) {

    xQueueReceive(bleTxQueue, &msg, portMAX_DELAY);

    if (!bleConnected || txCharacteristic == nullptr) {
      continue;
    }

    txCharacteristic->setValue(msg.text);
    txCharacteristic->notify();

    logLine("[BLE TX] %s", msg.text);
  }
}


// =====================================================
// Flutter BLE 명령 처리 (BLE 스택 태스크에서 실행됨)
//
// B:90 / F:3.5 / L:45.0 / R:32.0 / S:0
//
// 여기서는 모터를 직접 돌리지 않고 이벤트만 보낸다.
// =====================================================

float parseAngle(const String& command) {

  int colon = command.indexOf(':');

  if (colon < 0) {
    return 0;
  }

  return command.substring(colon + 1).toFloat();
}


void handleFlutterCommand(String command) {

  command.trim();

  if (command.length() == 0) {
    return;
  }

  logLine("[BLE RX] %s", command.c_str());

  // 목표 방위각 (예: B:135) - 로그용
  if (command.startsWith("B:")) {

    targetBearing = command.substring(2).toFloat();

    logLine("[NAV] Target bearing = %.2f", targetBearing);

    return;
  }

  if (command == "L" || command.startsWith("L:")) {
    postEvent(EV_APP_LEFT, parseAngle(command));
    return;
  }

  if (command == "R" || command.startsWith("R:")) {
    postEvent(EV_APP_RIGHT, parseAngle(command));
    return;
  }

  // 실제 모터 직진 X, 유도 모터만 정지
  if (command == "F" || command.startsWith("F:")) {
    postEvent(EV_APP_FORWARD);
    return;
  }

  if (command == "S" || command.startsWith("S:")) {
    postEvent(EV_APP_STOP);
    return;
  }

  logLine("[BLE] Unknown command");
}


// =====================================================
// BLE RX Callback
// =====================================================

class RxCallback : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic* characteristic) override {

    String command = characteristic->getValue().c_str();

    handleFlutterCommand(command);
  }
};


// =====================================================
// BLE 연결 상태
// =====================================================

class ServerCallback : public BLEServerCallbacks {

  void onConnect(BLEServer*) override {

    bleConnected = true;

    logLine("");
    logLine("BLE CONNECTED");
  }

  void onDisconnect(BLEServer* server) override {

    bleConnected = false;

    // 모터 정지도 motorTask에 맡긴다.
    postEvent(EV_APP_DISCONNECTED);

    logLine("");
    logLine("BLE DISCONNECTED");

    delay(200);

    server->getAdvertising()->start();

    logLine("BLE advertising restart");
  }
};


// =====================================================
// 디버그 모니터
//
// [MON] TFmini 최대 주기: 기존 loop였다면 역토크 중 1초 이상으로 늘어난다.
//       RTOS 버전은 항상 약 10ms여야 정상.
// stack free: 태스크별로 한 번이라도 남았던 최소 스택(byte).
//       수백 byte 이하로 떨어지면 xTaskCreate의 스택 크기를 늘린다.
// =====================================================

#if DEBUG_MONITOR
void monitorTask(void*) {

  for (;;) {

    vTaskDelay(pdMS_TO_TICKS(5000));

    logLine("[MON] TFmini max period=%lu us | stack free: obstacle=%u motor=%u pi=%u bleTx=%u side=%u",
         (unsigned long)obstacleMaxPeriodUs.exchange(0),
         (unsigned)uxTaskGetStackHighWaterMark(obstacleTaskHandle),
         (unsigned)uxTaskGetStackHighWaterMark(motorTaskHandle),
         (unsigned)uxTaskGetStackHighWaterMark(piTaskHandle),
         (unsigned)uxTaskGetStackHighWaterMark(bleTxTaskHandle),
         (unsigned)uxTaskGetStackHighWaterMark(sideTaskHandle));
  }
}
#endif


// =====================================================
// SETUP
// =====================================================

void setup() {

  // 로그 mutex와 큐는 다른 무엇보다 먼저 만든다.
  // (BLE 콜백이 setup 도중에 불려도 안전하게)
  logMutex = xSemaphoreCreateMutex();
  motorQueue = xQueueCreate(MOTOR_QUEUE_LEN, sizeof(MotorEvent));
  bleTxQueue = xQueueCreate(BLE_TX_QUEUE_LEN, sizeof(BleMsg));

  Serial.begin(115200);

  delay(500);

  logLine("");
  logLine("==============================");
  logLine("SMART CANE START (FreeRTOS)");
  logLine("==============================");

  // =================================================
  // Motor
  // =================================================
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);

  ledcAttach(M1_PWM, 5000, 8);
  ledcAttach(M2_PWM, 5000, 8);

  stopMotors();

  logLine("[OK] MDD10A");

  // =================================================
  // TFmini
  // =================================================
  TFSerial.begin(115200, SERIAL_8N1, TF_RX, TF_TX);

  logLine("[OK] TFmini UART2");

  // =================================================
  // VL53L1X LEFT / RIGHT
  // =================================================
  setupVL53L1X();

  // =================================================
  // Raspberry Pi UART
  // =================================================
  PiSerial.begin(115200, SERIAL_8N1, PI_RX, PI_TX);

  logLine("[OK] Raspberry Pi UART1");

  // =================================================
  // BLE
  // =================================================
  BLEDevice::init("SMART_CANE");

  BLEServer* server = BLEDevice::createServer();

  server->setCallbacks(new ServerCallback());

  BLEService* service = server->createService(SERVICE_UUID);

  // Flutter -> ESP32
  BLECharacteristic* rxCharacteristic =
      service->createCharacteristic(
        RX_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
      );

  rxCharacteristic->setCallbacks(new RxCallback());

  // ESP32 -> Flutter
  txCharacteristic =
      service->createCharacteristic(
        TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
      );

  txCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();

  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);

  BLEDevice::startAdvertising();

  logLine("[OK] BLE SMART_CANE");

  // =================================================
  // Tasks
  //
  // motorTask를 obstacleTask보다 먼저 만든다.
  // (obstacleTask가 motorTaskHandle로 알림을 보내기 때문)
  // =================================================
  xTaskCreatePinnedToCore(motorTask,      "motor",    4096, nullptr, 4, &motorTaskHandle,    APP_CORE);
  xTaskCreatePinnedToCore(obstacleTask,   "obstacle", 4096, nullptr, 5, &obstacleTaskHandle, APP_CORE);
  xTaskCreatePinnedToCore(piUartTask,     "piUart",   4096, nullptr, 3, &piTaskHandle,       APP_CORE);
  xTaskCreatePinnedToCore(bleTxTask,      "bleTx",    4096, nullptr, 2, &bleTxTaskHandle,    APP_CORE);
  xTaskCreatePinnedToCore(sideSensorTask, "side",     4096, nullptr, 1, &sideTaskHandle,     APP_CORE);

#if DEBUG_MONITOR
  xTaskCreatePinnedToCore(monitorTask,    "monitor",  3072, nullptr, 1, nullptr,             APP_CORE);
#endif

  logLine("[OK] Tasks started");

  logLine("");
  logLine("TFmini warning:");
  logLine(" <= 100 cm : Flutter screen warning");
  logLine(" <= 40 cm x3 : voice alert + reverse torque x3 + compare LEFT/RIGHT");
  logLine(" then pull toward open side until front >= 50 cm");
  logLine(" both side < 30 cm or still blocked : stop");
  logLine("");
}


// =====================================================
// LOOP
//
// 모든 일은 태스크가 한다. Arduino loopTask는 지운다.
// =====================================================

void loop() {
  vTaskDelete(nullptr);
}
