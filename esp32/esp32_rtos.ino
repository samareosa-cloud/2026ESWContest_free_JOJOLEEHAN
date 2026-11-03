#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <atomic>
#include <stdarg.h>
#include <esp_arduino_version.h>

// ledcAttach(pin, freq, bits) / 핀 번호 ledcWrite()는 Arduino-ESP32 3.x API
#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Arduino-ESP32 core 3.x 이상이 필요합니다. (보드 매니저에서 esp32 3.x 설치)"
#endif


// =====================================================
// NAVIS ESP32 - FreeRTOS
//
//  우선순위 5  obstacleTask    TFmini 10ms 주기, 정면 상태 판단
//  우선순위 4  motorTask       모터를 만지는 유일한 태스크
//  우선순위 3  piUartTask      Raspberry Pi 수신
//  우선순위 2  bleTxTask       BLE notify 전송
//  우선순위 1  sideSensorTask  VL53L1X 좌/우 측정
// =====================================================


// MDD10A
#define M1_DIR 25
#define M1_PWM 27

#define M2_DIR 32
#define M2_PWM 14


// TFmini TX -> GPIO16
#define TF_RX 16
#define TF_TX 17

HardwareSerial TFSerial(2);


// VL53L1X: 기본 주소(0x29)가 같아서 XSHUT으로 하나씩 켜고 주소를 바꾼다.
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


// Raspberry Pi TX -> GPIO18, RX <- GPIO19 (GND 공통)
#define PI_RX 18
#define PI_TX 19

HardwareSerial PiSerial(1);


#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLECharacteristic* txCharacteristic = nullptr;


// =====================================================
// 장애물
// =====================================================

const int OBSTACLE_DISTANCE = 40;
const int RESET_DISTANCE = 50;

const int OBSTACLE_CONFIRM_COUNT = 3;   // 40cm 이하 연속 횟수
const int CLEAR_CONFIRM_COUNT = 3;      // 50cm 이상 연속 횟수
const int OBSTACLE_GAP_RESET_MS = 50;   // 유효 측정이 이만큼 끊기면 연속 횟수 초기화

// TFmini 신호 세기가 이보다 낮거나 65535(과노출)면 거리를 믿지 않는다.
const int TF_MIN_STRENGTH = 100;
const int TF_STALE_MS = 300;            // 프레임이 안 옴
const int TF_INVALID_LOST_MS = 1000;    // 프레임은 오는데 유효 거리가 없음
const int TF_LOST_REPEAT_MS = 3000;

const int OBSTACLE_NOTIFY_DELAY_MS = 50;
const int OBSTACLE_SETTLE_MS = 100;

const int SIDE_BLOCK_DISTANCE = 30;
const int DIRECTION_MARGIN = 20;

const int AVOID_PWM = 120;
const int AVOID_PULSE_TIME = 400;
const int AVOID_REPEAT_MS = 900;
const int AVOID_MAX_PULSES = 8;

enum AvoidState {
  AVOID_NONE,
  AVOID_LEFT,
  AVOID_RIGHT,
  AVOID_BLOCKED
};


// =====================================================
// 모터
// =====================================================

const int MOTOR_RAMP_TIME_MS = 100;
const int MOTOR_RAMP_STEPS = 5;

const int GUIDE_PWM = 100;
const int GUIDE_TIME = 250;

const int BRAKE_PWM = 100;
const int BRAKE_ON_TIME = 300;
const int BRAKE_OFF_TIME = 200;
const int BRAKE_COUNT = 3;
const int RED_BRAKE_COUNT = 2;

const int CROSS_PWM = 110;
const int CROSS_PULSE_MAX_TIME = 300;
const int CROSS_TIMEOUT_MS = 1000;
const int CROSS_CMD_MAX_AGE_MS = 100;   // 이보다 늦게 처리되는 CROSS_MOTOR:L/R은 버린다.

// 횡단 중 / 횡단 종료 후 이 시간 안에 본 빨간불은 바로 멈추게 하지 않고 보류한다.
// 유예가 끝났는데 다시 건너는 중이 아니고 초록불도 없으면 그때 빨간불로 처리한다.
const int CROSSING_RED_GRACE_MS = 3000;


// =====================================================
// FreeRTOS
// =====================================================

#define APP_CORE 1

const int OBSTACLE_PERIOD_MS = 10;
const int MOTOR_TICK_MS = 10;
const int PI_POLL_MS = 5;
const int SIDE_POLL_MS = 20;
const int SIDE_STALE_MS = 300;
const int REPORT_MS = 300;
const int ADVERTISE_RESTART_DELAY_MS = 200;

const int MOTOR_QUEUE_LEN = 32;
const int BLE_ALERT_QUEUE_LEN = 24;   // 장애물·신호·상태 알림
const int BLE_REPORT_QUEUE_LEN = 4;   // DIST / SIDE / Pi 기타 메시지 (밀리면 버려도 됨)
const int BLE_MAX_PAYLOAD = 20;       // 기본 MTU(23) - 3
const int LOG_QUEUE_LEN = 32;
const int PI_PENDING_LEN = 16;
const int PI_LINE_MAX = 100;

#define DEBUG_MONITOR 1


enum EventType : uint8_t {

  // obstacleTask -> motorTask
  EV_FRONT_STATE,

  // Flutter -> motorTask (S / 연결 해제는 requestStop)
  EV_APP_LEFT,
  EV_APP_RIGHT,
  EV_APP_FORWARD,
  EV_APP_STATE_REQUEST,

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

// EV_FRONT_STATE flags
const uint8_t FRONT_LOST = 1 << 0;
const uint8_t FRONT_OBSTACLE = 1 << 1;

struct MotorEvent {
  EventType type;
  float value;          // 앱 L/R 각도, 정면 거리(cm)
  uint32_t stampUs;
  uint32_t seq;         // 정면 상태 번호 / 앱 명령 번호
  uint32_t obstacleId;  // EV_FRONT_STATE: 장애물마다 1씩 증가
  uint8_t flags;        // EV_FRONT_STATE
};

struct BleMsg {
  char text[PI_LINE_MAX + 4];
};

QueueHandle_t motorQueue = nullptr;
QueueHandle_t bleAlertQueue = nullptr;
QueueHandle_t bleReportQueue = nullptr;
QueueHandle_t logQueue = nullptr;

TaskHandle_t obstacleTaskHandle = nullptr;
TaskHandle_t motorTaskHandle = nullptr;
TaskHandle_t piTaskHandle = nullptr;
TaskHandle_t bleTxTaskHandle = nullptr;
TaskHandle_t logTaskHandle = nullptr;
TaskHandle_t sideTaskHandle = nullptr;

std::atomic<bool> bleConnected(false);
std::atomic<uint32_t> advertiseRestartAt(0);   // 0 = 예약 없음

// S보다 먼저 큐에 들어간 L/R/F는 번호가 정지 요청보다 작아서 버려진다.
std::atomic<uint32_t> appCmdSeq(0);
std::atomic<uint32_t> stopRequestSeq(0);
std::atomic<bool> stopByDisconnect(false);
std::atomic<int> sideLeftCm(-1);
std::atomic<int> sideRightCm(-1);
std::atomic<uint32_t> sideLeftAt(0);     // 마지막 측정 시각 (millis)
std::atomic<uint32_t> sideRightAt(0);
std::atomic<bool> bleAlertDropped(false);
std::atomic<uint32_t> droppedLogs(0);
std::atomic<uint32_t> obstacleMaxPeriodUs(0);

float targetBearing = -1;


// =====================================================
// 로그 / BLE 전송
// =====================================================

void logLine(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void sendBLEf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void sendReportf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

struct LogMsg {
  char text[160];
};

// 로그는 큐에 넣기만 하고 출력은 logTask가 한다. 큐가 차면 로그만 버린다.
void logLine(const char* fmt, ...) {

  LogMsg msg;

  va_list args;
  va_start(args, fmt);
  vsnprintf(msg.text, sizeof(msg.text), fmt, args);
  va_end(args);

  if (logQueue == nullptr) {
    Serial.println(msg.text);
    return;
  }

  if (xQueueSend(logQueue, &msg, 0) != pdTRUE) {
    droppedLogs++;
  }
}


bool fitsBle(const char* message) {

  if (strlen(message) <= (size_t)BLE_MAX_PAYLOAD) {
    return true;
  }

  logLine("[BLE TX] too long for MTU, dropped: %s", message);
  return false;
}


// 중요한 알림. 큐가 차서 버려지면 큐가 빈 뒤 ST: 상태를 한 번 다시 보낸다.
void sendBLE(const char* message) {

  if (!bleConnected || !fitsBle(message)) {
    return;
  }

  BleMsg msg;

  strncpy(msg.text, message, sizeof(msg.text) - 1);
  msg.text[sizeof(msg.text) - 1] = '\0';

  if (xQueueSend(bleAlertQueue, &msg, 0) != pdTRUE) {
    bleAlertDropped = true;
    logLine("[BLE TX] alert queue full, dropped: %s", message);
  }
}

// 주기 보고 / 기타 메시지. 밀리면 그냥 버린다.
void sendReport(const char* message) {

  if (!bleConnected || !fitsBle(message)) {
    return;
  }

  BleMsg msg;

  strncpy(msg.text, message, sizeof(msg.text) - 1);
  msg.text[sizeof(msg.text) - 1] = '\0';

  xQueueSend(bleReportQueue, &msg, 0);
}

void sendBLEf(const char* fmt, ...) {

  char buf[sizeof(BleMsg::text)];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  sendBLE(buf);
}

void sendReportf(const char* fmt, ...) {

  char buf[sizeof(BleMsg::text)];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  sendReport(buf);
}


bool postEvent(EventType type, float value = 0, uint32_t seq = 0, bool logFail = true) {

  MotorEvent ev = { type, value, (uint32_t)micros(), seq, 0, 0 };

  if (xQueueSend(motorQueue, &ev, 0) != pdTRUE) {
    if (logFail) {
      logLine("[EVENT] queue full, type=%d", (int)type);
    }
    return false;
  }

  return true;
}

bool postAppEvent(EventType type, float value = 0) {
  return postEvent(type, value, ++appCmdSeq);
}

// S / BLE 연결 해제. 큐를 거치지 않아서 큐가 꽉 차도 사라지지 않는다.
void requestStop(bool disconnected) {
  stopByDisconnect = disconnected;
  stopRequestSeq = ++appCmdSeq;
}


// #####################################################
// 여기부터 motorTask 전용 (다른 태스크는 이벤트로만 요청)
// #####################################################

enum ObstaclePhase : uint8_t {
  OBS_NONE,
  OBS_BRAKING,
  OBS_SETTLING,
  OBS_AVOIDING,
  OBS_FINAL_BRAKE,
  OBS_HOLD          // 막힘 / 앱 정지 / 연결 해제. 해제될 때까지 당기지 않음
};

ObstaclePhase obstaclePhase = OBS_NONE;
uint32_t lastFrontSeq = 0;
uint32_t appliedObstacleId = 0;
unsigned long settleUntil = 0;

// S / 연결 해제 뒤 새 앱 명령이 올 때까지 당기지 않는다. (역토크 경고는 함)
bool motionLatched = false;
uint32_t handledStopSeq = 0;

// 첫 유효 측정 전까지도 이상으로 본다.
bool frontSensorLost = true;

bool crosswalkMode = false;
bool trafficRed = false;        // 건너면 안 됨 (RED / GREEN_WAIT) -> 횡단 보정 금지
bool redDeferred = false;       // 횡단 중 / 종료 유예 중 본 빨간불
char lightState = 'N';          // 관측한 신호: N / R / G / W

bool crossingActive = false;
unsigned long crossingEndedAt = 0;

AvoidState avoidState = AVOID_NONE;

int avoidPulseCount = 0;
unsigned long lastAvoidPulseAt = 0;

char crossSteer = 'C';
unsigned long lastCrossMsgAt = 0;
bool crossPulseStartNow = false;


// sideSensorTask가 멈춰 값이 오래됐으면 막힘(-1)으로 본다.
int sideDistance(bool left) {

  int distance = left ? sideLeftCm : sideRightCm;
  uint32_t at = left ? sideLeftAt : sideRightAt;

  if (distance < 0 || (uint32_t)millis() - at > (uint32_t)SIDE_STALE_MS) {
    return -1;
  }

  return distance;
}


bool obstacleActive() {
  return obstaclePhase != OBS_NONE;
}

const char* pullBlockedReason() {
  if (frontSensorLost)  return "front sensor lost";
  if (motionLatched)    return "stop latched";
  if (obstacleActive()) return "obstacle";
  return nullptr;
}


void stopMotors() {
  ledcWrite(M1_PWM, 0);
  ledcWrite(M2_PWM, 0);
}


// =====================================================
// 모터 동작 상태 머신
//
//   [시작 지연] -> [램프업] -> [유지] -> [쉼] -> 다음 펄스 ...
//
// 한 번에 하나만 돈다. 새 동작을 시작하면 이전 동작은 취소된다.
// =====================================================

enum JobKind : uint8_t {
  JOB_NONE,
  JOB_GUIDE,
  JOB_CROSS,
  JOB_AVOID,
  JOB_OBSTACLE_BRAKE,
  JOB_RED_BRAKE
};

enum JobPhase : uint8_t {
  PH_DELAY,
  PH_RAMP,
  PH_HOLD,
  PH_OFF
};

struct MotorJob {
  JobKind kind = JOB_NONE;
  JobPhase phase = PH_DELAY;
  bool useM1 = false;
  bool useM2 = false;
  int pwm = 0;
  int holdMs = 0;
  int offMs = 0;
  int pulses = 0;
  int pulseIndex = 0;
  int rampStep = 0;
  char side = 'C';
  unsigned long nextAt = 0;
};

MotorJob job;

const char* jobName(JobKind kind) {
  switch (kind) {
    case JOB_GUIDE:          return "GUIDE";
    case JOB_CROSS:          return "CROSS";
    case JOB_AVOID:          return "AVOID";
    case JOB_OBSTACLE_BRAKE: return "OBSTACLE_BRAKE";
    case JOB_RED_BRAKE:      return "RED_BRAKE";
    default:                 return "NONE";
  }
}

void writeJobPwm(int pwm) {
  ledcWrite(M1_PWM, job.useM1 ? pwm : 0);
  ledcWrite(M2_PWM, job.useM2 ? pwm : 0);
}


// 모터부터 끄고 로그는 그 뒤에 (Serial 출력 시간만큼 정지가 늦어지지 않게)
void cancelJob(const char* reason) {

  JobKind previous = job.kind;

  job.kind = JOB_NONE;

  stopMotors();

  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);

  if (previous != JOB_NONE) {
    logLine("[JOB] %s cancelled - %s", jobName(previous), reason);
  }
}


void startJob(JobKind kind, bool useM1, bool useM2, bool reverse,
              int pwm, int holdMs, int offMs, int pulses,
              int startDelayMs = 0, char side = 'C') {

  cancelJob("replaced");

  job.kind = kind;
  job.useM1 = useM1;
  job.useM2 = useM2;
  job.pwm = pwm;
  job.holdMs = holdMs;
  job.offMs = offMs;
  job.pulses = pulses;
  job.pulseIndex = 0;
  job.rampStep = 0;
  job.side = side;
  job.phase = PH_DELAY;
  job.nextAt = millis() + startDelayMs;

  digitalWrite(M1_DIR, (useM1 && reverse) ? HIGH : LOW);
  digitalWrite(M2_DIR, (useM2 && reverse) ? HIGH : LOW);
}


void onJobDone(JobKind kind, char side);

void stepJob() {

  const int rampStepMs = MOTOR_RAMP_TIME_MS / MOTOR_RAMP_STEPS;

  unsigned long now = millis();

  while (job.kind != JOB_NONE && (long)(now - job.nextAt) >= 0) {

    switch (job.phase) {

      case PH_DELAY:
      case PH_OFF:
        if (job.kind == JOB_OBSTACLE_BRAKE || job.kind == JOB_RED_BRAKE) {
          logLine("[BRAKE] %d/%d", job.pulseIndex + 1, job.pulses);
        }
        job.phase = PH_RAMP;
        job.rampStep = 0;
        job.nextAt = now;
        break;

      case PH_RAMP:
        job.rampStep++;
        writeJobPwm((job.pwm * job.rampStep) / MOTOR_RAMP_STEPS);

        if (job.rampStep >= MOTOR_RAMP_STEPS) {
          job.phase = PH_HOLD;
          job.nextAt = now + rampStepMs + job.holdMs;
        }
        else {
          job.nextAt = now + rampStepMs;
        }
        break;

      case PH_HOLD:
        writeJobPwm(0);
        job.pulseIndex++;

        if (job.pulseIndex < job.pulses) {
          job.phase = PH_OFF;
          job.nextAt = now + job.offMs;
          break;
        }

        {
          JobKind done = job.kind;
          char side = job.side;

          job.kind = JOB_NONE;
          stopMotors();
          digitalWrite(M1_DIR, LOW);
          digitalWrite(M2_DIR, LOW);

          onJobDone(done, side);
        }
        break;
    }
  }
}


TickType_t jobWaitTicks() {

  if (job.kind == JOB_NONE) {
    return pdMS_TO_TICKS(MOTOR_TICK_MS);
  }

  long remain = (long)(job.nextAt - millis());

  if (remain <= 0) {
    return 0;
  }

  if (remain > MOTOR_TICK_MS) {
    remain = MOTOR_TICK_MS;
  }

  return pdMS_TO_TICKS(remain);
}


// =====================================================
// 앱 길안내
// LEFT: M2(오른쪽 바퀴) / RIGHT: M1(왼쪽 바퀴)
// F는 모터를 돌리지 않고 길안내 당김만 끊는다.
// =====================================================

void guidePulse(char side, float angle) {

  const char* name = (side == 'L') ? "LEFT" : "RIGHT";

  const char* blocked = pullBlockedReason();

  if (blocked != nullptr) {
    logLine("[MOTOR] %s ignored - %s", name, blocked);
    return;
  }

  if (crosswalkMode) {
    logLine("[MOTOR] %s ignored - crosswalk mode", name);
    return;
  }

  if (job.kind != JOB_NONE && job.kind != JOB_GUIDE) {
    logLine("[MOTOR] %s ignored - %s running", name, jobName(job.kind));
    return;
  }

  logLine("[MOTOR] %s | angle=%.2f", name, angle);

  bool left = side == 'L';

  startJob(JOB_GUIDE, !left, left, false, GUIDE_PWM, GUIDE_TIME, 0, 1, 0, side);
}


void guideForward() {

  if (job.kind == JOB_GUIDE) {
    cancelJob("forward");
  }

  logLine("[MOTOR] FORWARD / NEUTRAL");
}


// =====================================================
// 정지 (앱 S / BLE 연결 해제)
// =====================================================

void holdStop(const char* reason) {

  cancelJob(reason);

  motionLatched = true;

  if (obstacleActive()) {
    obstaclePhase = OBS_HOLD;
  }

  avoidState = AVOID_NONE;

  crossSteer = 'C';
  crossPulseStartNow = false;

  logLine("[MOTOR] STOP (%s) -> hold until next app command", reason);
}


void checkStopRequest() {

  uint32_t req = stopRequestSeq;

  if (req == handledStopSeq) {
    return;
  }

  handledStopSeq = req;

  bool disconnected = stopByDisconnect;

  holdStop(disconnected ? "BLE disconnected" : "app S");

  if (!disconnected) {
    sendBLE("MOTOR:STOP");
  }
}


bool acceptAppCommand(const MotorEvent& ev) {

  if ((int32_t)(ev.seq - handledStopSeq) <= 0) {
    logLine("[MOTOR] app command before stop dropped (seq=%lu)", (unsigned long)ev.seq);
    return false;
  }

  if (motionLatched) {
    motionLatched = false;
    logLine("[MOTOR] stop latch released by new app command");
  }

  return true;
}


// =====================================================
// 횡단 보정 (Pi CROSS_MOTOR:L / R / CENTER)
// L/R을 받으면 한 번만 당기고, CENTER가 오면 바로 멈춘다.
// =====================================================

void setCrossSteer(char dir) {

  lastCrossMsgAt = millis();

  if (dir == crossSteer) {
    return;
  }

  crossSteer = dir;

  if (job.kind == JOB_CROSS) {
    cancelJob("cross direction changed");
  }

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


void stopCrossSteer() {

  crossSteer = 'C';
  crossPulseStartNow = false;

  if (job.kind == JOB_CROSS) {
    cancelJob("cross steer stop");
  }
}


// 횡단보도 앞 / 신호 대기 / 횡단 시작: 길안내와 횡단 보정 당김을 모두 끊는다.
void stopPullsForCrosswalk(const char* reason) {

  stopCrossSteer();

  if (job.kind == JOB_GUIDE) {
    cancelJob(reason);
  }
}


void updateCrosswalkSteering() {

  if (pullBlockedReason() != nullptr) {
    crossPulseStartNow = false;
    return;
  }

  if (!crosswalkMode || trafficRed || (crossSteer != 'L' && crossSteer != 'R')) {
    crossPulseStartNow = false;
    return;
  }

  if (millis() - lastCrossMsgAt > (unsigned long)CROSS_TIMEOUT_MS) {

    logLine("[CROSS MOTOR] Pi timeout -> CENTER");

    stopCrossSteer();
    sendBLE("CROSS_MOTOR:CENTER");
    return;
  }

  if (!crossPulseStartNow || job.kind != JOB_NONE) {
    return;
  }

  crossPulseStartNow = false;

  bool left = crossSteer == 'L';

  startJob(JOB_CROSS, !left, left, false, CROSS_PWM, CROSS_PULSE_MAX_TIME, 0, 1);
}


// =====================================================
// 장애물 회피
//
// - 30cm 미만 / 측정 무효(-1)인 쪽은 막힘
// - 이미 피하던 쪽이 아직 열려 있으면 유지
// - 둘 다 열림 -> 20cm 넘게 넓은 쪽, 비슷하면 오른쪽
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
    sendBLE("AVOID:BLOCKED");
  }
}


void startAvoidPulse(AvoidState side) {

  bool left = side == AVOID_LEFT;

  logLine("[AVOID] PULL %s #%d", left ? "LEFT" : "RIGHT", avoidPulseCount + 1);

  startJob(JOB_AVOID, !left, left, false, AVOID_PWM, AVOID_PULSE_TIME, 0, 1, 0,
           left ? 'L' : 'R');

  lastAvoidPulseAt = millis();
  avoidPulseCount++;

  sendBLE(left ? "MOTOR:L" : "MOTOR:R");
}


void startObstacleBrake(int startDelayMs) {

  logLine("*** OBSTACLE WARNING ***");

  startJob(JOB_OBSTACLE_BRAKE, true, true, true,
           BRAKE_PWM, BRAKE_ON_TIME, BRAKE_OFF_TIME, BRAKE_COUNT, startDelayMs);
}


// 끝까지 끝난 동작만 불린다. (취소된 경우 X)
void onJobDone(JobKind kind, char side) {

  switch (kind) {

    case JOB_GUIDE:
      sendBLE(side == 'L' ? "MOTOR:L" : "MOTOR:R");
      break;

    case JOB_OBSTACLE_BRAKE:
      logLine("*** BRAKE END ***");

      if (obstaclePhase == OBS_BRAKING) {
        obstaclePhase = OBS_SETTLING;
        settleUntil = millis() + OBSTACLE_SETTLE_MS;
      }
      else if (obstaclePhase == OBS_FINAL_BRAKE) {
        obstaclePhase = OBS_HOLD;
      }
      break;

    case JOB_RED_BRAKE:
      logLine("*** RED BRAKE END ***");
      break;

    default:
      break;
  }
}


void handleObstacleConfirmed(const MotorEvent& ev) {

  cancelJob("obstacle");

  logLine("[LATENCY] obstacle confirm -> motor stop: %lu us",
          (unsigned long)(micros() - ev.stampUs));

  crossPulseStartNow = false;

  avoidPulseCount = 0;
  avoidState = AVOID_NONE;

  logLine("[AVOID] FRONT OBSTACLE CONFIRMED (%d cm) -> BRAKE FIRST", (int)ev.value);

  sendBLE("OBSTACLE_BRAKE");

  obstaclePhase = OBS_BRAKING;

  startObstacleBrake(OBSTACLE_NOTIFY_DELAY_MS);
}


void handleObstacleClear() {

  if (job.kind == JOB_OBSTACLE_BRAKE || job.kind == JOB_AVOID) {
    cancelJob("obstacle cleared");
  }

  obstaclePhase = OBS_NONE;
  avoidPulseCount = 0;
  avoidState = AVOID_NONE;

  sendBLE("OBSTACLE_CLEAR");

  logLine("[TFmini] Obstacle cleared");
}


// 정면 센서 이상: 모든 당김 금지. 빨간불 역토크 경고는 계속.
// (OBSTACLE_CLEAR를 보내면 앱이 "장애물을 지났습니다"라고 하므로 보내지 않는다)
void handleFrontSensorLost() {

  frontSensorLost = true;

  if (job.kind != JOB_NONE && job.kind != JOB_RED_BRAKE) {
    cancelJob("front sensor lost");
  }

  crossPulseStartNow = false;

  obstaclePhase = OBS_NONE;
  avoidPulseCount = 0;
  avoidState = AVOID_NONE;

  logLine("[TFmini] sensor lost -> all pulls blocked");
}


void handleFrontSensorOk() {

  frontSensorLost = false;

  logLine("[TFmini] sensor OK -> pulls allowed again");
}


// =====================================================
// 정면 상태 (EV_FRONT_STATE)
//
// obstacleTask는 센서 이상 / 장애물 여부를 한 묶음으로 보낸다.
// 번호(seq)가 가장 큰 묶음이 최신 상태이므로 오래된 묶음은 버려도
// 정보가 사라지지 않는다.
// =====================================================

void handleFrontState(const MotorEvent& ev) {

  if ((int32_t)(ev.seq - lastFrontSeq) <= 0) {
    logLine("[FRONT] stale state dropped (seq=%lu last=%lu)",
            (unsigned long)ev.seq, (unsigned long)lastFrontSeq);
    return;
  }

  lastFrontSeq = ev.seq;

  if (ev.flags & FRONT_LOST) {
    if (!frontSensorLost) {
      handleFrontSensorLost();
    }
    return;
  }

  if (frontSensorLost) {
    handleFrontSensorOk();
  }

  if (ev.flags & FRONT_OBSTACLE) {
    if (ev.obstacleId != appliedObstacleId) {
      appliedObstacleId = ev.obstacleId;
      handleObstacleConfirmed(ev);
    }
  }
  else if (obstacleActive()) {
    handleObstacleClear();
  }
}


// 회피 다음 단계 고르기: 막혔으면 멈추고, 아니면 그쪽으로 당긴다.
void pullTowardOpenSide() {

  AvoidState side = chooseAvoidSide(sideDistance(true), sideDistance(false));

  setAvoidState(side);

  if (side == AVOID_BLOCKED) {
    logLine("[AVOID] BOTH SIDES BLOCKED -> STOP");
    obstaclePhase = OBS_HOLD;
    return;
  }

  if (motionLatched) {
    logLine("[AVOID] stop latched -> no pull");
    obstaclePhase = OBS_HOLD;
    return;
  }

  if (avoidPulseCount >= AVOID_MAX_PULSES) {

    logLine("[AVOID] front still blocked -> BLOCKED");

    setAvoidState(AVOID_BLOCKED);

    obstaclePhase = OBS_FINAL_BRAKE;
    startObstacleBrake(0);
    return;
  }

  obstaclePhase = OBS_AVOIDING;
  startAvoidPulse(side);
}


void updateObstacleAvoidance() {

  unsigned long now = millis();

  if (obstaclePhase == OBS_SETTLING) {

    if ((long)(now - settleUntil) < 0) {
      return;
    }

    logLine("[AVOID] LEFT=%d cm RIGHT=%d cm", sideDistance(true), sideDistance(false));

    pullTowardOpenSide();
    return;
  }

  if (obstaclePhase != OBS_AVOIDING) {
    return;
  }

  // 당기는 중에 그쪽이 막히거나 측정이 무효가 되면 바로 멈춘다.
  // 반대쪽 당김은 다음 간격(AVOID_REPEAT_MS)에 시작한다.
  if (job.kind == JOB_AVOID) {

    int distance = sideDistance(avoidState == AVOID_LEFT);

    if (distance >= SIDE_BLOCK_DISTANCE) {
      return;
    }

    cancelJob("avoid side blocked");
    logLine("[AVOID] %s side now %d cm -> stop",
            avoidState == AVOID_LEFT ? "LEFT" : "RIGHT", distance);

    AvoidState side = chooseAvoidSide(sideDistance(true), sideDistance(false));

    setAvoidState(side);

    if (side == AVOID_BLOCKED) {
      logLine("[AVOID] BOTH SIDES BLOCKED -> STOP");
      obstaclePhase = OBS_HOLD;
    }
    return;
  }

  if (now - lastAvoidPulseAt < (unsigned long)AVOID_REPEAT_MS) {
    return;
  }

  pullTowardOpenSide();
}


// =====================================================
// 신호 / 횡단
// =====================================================

bool isCrossingNow() {

  if (crossingActive) {
    return true;
  }

  return crossingEndedAt != 0 &&
         millis() - crossingEndedAt < (unsigned long)CROSSING_RED_GRACE_MS;
}


// ageMs: CROSSING_END를 받은 뒤 처리되기까지 걸린 시간 (유예를 받은 시각부터 센다)
void endCrossing(uint32_t ageMs = 0) {

  if (crossingActive) {
    crossingEndedAt = millis() - ageMs;
    if (crossingEndedAt == 0) {
      crossingEndedAt = 1;
    }
  }

  crossingActive = false;
}


// 앱 LIGHT:RED + (기다리던 중이 아니었으면) 역토크 2회
void applyRed() {

  if (crosswalkMode && trafficRed && lightState == 'R') {
    return;
  }

  bool brake = !trafficRed;

  crosswalkMode = true;
  trafficRed = true;
  lightState = 'R';

  stopPullsForCrosswalk("red");

  sendBLE("LIGHT:RED");

  if (brake && !obstacleActive()) {

    logLine("[TRAFFIC] RED -> stop warning");

    startJob(JOB_RED_BRAKE, true, true, true,
             BRAKE_PWM, BRAKE_ON_TIME, BRAKE_OFF_TIME, RED_BRAKE_COUNT);
  }
}


void updateDeferredRed() {

  if (!redDeferred || isCrossingNow()) {
    return;
  }

  redDeferred = false;

  logLine("[TRAFFIC] crossing over, red still pending -> RED");

  applyRed();
}


void cancelRedBrake(const char* reason) {
  if (job.kind == JOB_RED_BRAKE) {
    cancelJob(reason);
  }
}


// 같은 메시지가 반복으로 와도 결과가 같다. 상태가 바뀔 때만 앱에 알린다.
void handlePiEvent(EventType type, uint32_t ageMs) {

  switch (type) {

    case EV_PI_CROSSWALK_ON:

      if (crosswalkMode) {
        break;
      }

      crosswalkMode = true;
      stopPullsForCrosswalk("crosswalk");
      sendBLE("CROSSWALK:1");
      break;

    case EV_PI_CROSSWALK_OFF:

      redDeferred = false;

      if (!crosswalkMode && !crossingActive) {
        break;
      }

      crosswalkMode = false;
      trafficRed = false;
      lightState = 'N';
      endCrossing();
      cancelRedBrake("crosswalk off");
      stopCrossSteer();
      sendBLE("CROSSWALK:0");
      break;

    // 건너는 중이면 역토크 없이 끝까지 건너게 하고, 본 빨간불은 저장해 둔다.
    case EV_PI_LIGHT_RED:

      if (isCrossingNow()) {

        if (!redDeferred) {
          logLine("[TRAFFIC] RED while crossing -> no brake, deferred");
        }

        redDeferred = true;

        if (crossingActive) {
          lightState = 'R';
        }

        sendBLE("CROSSING_RED");
        break;
      }

      applyRed();
      break;

    case EV_PI_LIGHT_GREEN: {

      bool changed = !crosswalkMode || lightState != 'G';

      crosswalkMode = true;
      trafficRed = false;
      lightState = 'G';
      redDeferred = false;

      cancelRedBrake("green");

      if (!changed) {
        break;
      }

      if (!crossingActive) {
        stopPullsForCrosswalk("green");
      }

      sendBLE("LIGHT:GREEN");
      break;
    }

    // 초록불이 이미 켜져 있음: 남은 시간을 모르므로 다음 신호를 기다린다.
    case EV_PI_LIGHT_GREEN_WAIT:

      if (isCrossingNow()) {
        logLine("[TRAFFIC] GREEN_WAIT during crossing -> ignored");
        break;
      }

      if (crosswalkMode && lightState == 'W') {
        break;
      }

      crosswalkMode = true;
      trafficRed = true;
      lightState = 'W';
      cancelRedBrake("green wait");
      stopPullsForCrosswalk("green wait");
      sendBLE("LIGHT:GREEN_WAIT");
      break;

    case EV_PI_CROSSING_START:

      if (crossingActive) {
        break;
      }

      crosswalkMode = true;
      trafficRed = false;
      lightState = redDeferred ? 'R' : 'G';
      crossingActive = true;
      cancelRedBrake("crossing start");
      stopPullsForCrosswalk("crossing start");
      sendBLE("CROSSING_START");
      break;

    case EV_PI_CROSSING_END:

      if (!crossingActive && !crosswalkMode) {
        break;
      }

      crosswalkMode = false;
      trafficRed = false;
      lightState = 'N';
      endCrossing(ageMs);
      stopCrossSteer();
      sendBLE("CROSSING_END");
      break;

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


// =====================================================
// 상태 응답 (앱 Q -> ST:abcdefg)
//
//  a 장애물 처리 중   0 / 1
//  b 회피 방향        N / L / R / B
//  c 횡단보도 모드    0 / 1
//  d 신호             N / R / G / W
//  e 횡단 중          0 / 1
//  f 정면 센서        1 정상 / 0 이상
//  g 정지 유지        0 / 1
// =====================================================

void sendCaneState() {

  char avoid = 'N';

  if (obstacleActive()) {
    avoid = avoidState == AVOID_LEFT    ? 'L'
          : avoidState == AVOID_RIGHT   ? 'R'
          : avoidState == AVOID_BLOCKED ? 'B'
          : 'N';
  }

  char light = redDeferred ? 'R' : crosswalkMode ? lightState : 'N';

  sendBLEf("ST:%c%c%c%c%c%c%c",
           obstacleActive() ? '1' : '0',
           avoid,
           crosswalkMode ? '1' : '0',
           light,
           crossingActive ? '1' : '0',
           frontSensorLost ? '0' : '1',
           motionLatched ? '1' : '0');
}


void handleMotorEvent(const MotorEvent& ev) {

  switch (ev.type) {

    case EV_FRONT_STATE:
      handleFrontState(ev);
      break;

    case EV_APP_LEFT:
      if (acceptAppCommand(ev)) {
        guidePulse('L', ev.value);
      }
      break;

    case EV_APP_RIGHT:
      if (acceptAppCommand(ev)) {
        guidePulse('R', ev.value);
      }
      break;

    case EV_APP_FORWARD:
      if (acceptAppCommand(ev)) {
        guideForward();
      }
      break;

    case EV_APP_STATE_REQUEST:
      sendCaneState();
      break;

    default: {

      uint32_t ageMs = (uint32_t)(micros() - ev.stampUs) / 1000;

      if ((ev.type == EV_PI_CROSS_L || ev.type == EV_PI_CROSS_R) &&
          ageMs > (uint32_t)CROSS_CMD_MAX_AGE_MS) {
        logLine("[CROSS MOTOR] stale steer dropped (%lu ms old)", (unsigned long)ageMs);
        break;
      }

      handlePiEvent(ev.type, ageMs);
      break;
    }
  }
}


void motorTask(void*) {

  MotorEvent ev;

  for (;;) {

    checkStopRequest();

    if (xQueueReceive(motorQueue, &ev, jobWaitTicks()) == pdTRUE) {
      checkStopRequest();
      handleMotorEvent(ev);
    }

    stepJob();

    updateObstacleAvoidance();

    updateDeferredRed();

    updateCrosswalkSteering();
  }
}

// #####################################################
// motorTask 전용 끝
// #####################################################


// =====================================================
// TFmini
//
// 버퍼에 쌓인 프레임을 모두 읽고 가장 최근의 유효한 거리를 돌려준다. (없으면 -1)
// alive: checksum이 맞는 프레임을 하나라도 받았는지
// =====================================================

int readTFmini(bool& alive) {

  int latest = -1;

  alive = false;

  while (TFSerial.available() >= 9) {

    if (TFSerial.read() != 0x59) {
      continue;
    }

    if (TFSerial.peek() != 0x59) {
      continue;
    }

    TFSerial.read();

    uint8_t buf[9];

    buf[0] = 0x59;
    buf[1] = 0x59;

    for (int i = 2; i < 9; i++) {
      buf[i] = TFSerial.read();
    }

    uint16_t checksum = 0;

    for (int i = 0; i < 8; i++) {
      checksum += buf[i];
    }

    if ((checksum & 0xFF) != buf[8]) {
      logLine("[TFmini] checksum error");
      continue;
    }

    alive = true;

    int distance = buf[2] | (buf[3] << 8);
    int strength = buf[4] | (buf[5] << 8);

    if (distance <= 0 || strength < TF_MIN_STRENGTH || strength == 65535) {
      continue;
    }

    latest = distance;
  }

  return latest;
}


// =====================================================
// obstacleTask
//
// 정면 상태(센서 이상 / 장애물)가 바뀌면 한 묶음으로 motorTask 큐 맨 앞에 넣는다.
// 큐가 꽉 차 있으면 기다리지 않고 다음 주기에 최신 상태로 다시 보낸다.
// =====================================================

bool sendFrontState(uint32_t& seq, bool lost, bool obstacle, uint32_t obstacleId, int distance) {

  uint8_t flags = (lost ? FRONT_LOST : 0) | (obstacle ? FRONT_OBSTACLE : 0);

  MotorEvent ev = { EV_FRONT_STATE, (float)distance, (uint32_t)micros(),
                    seq + 1, obstacleId, flags };

  if (xQueueSendToFront(motorQueue, &ev, 0) != pdTRUE) {
    return false;
  }

  seq++;
  return true;
}


void obstacleTask(void*) {

  int obstacleCount = 0;
  int clearCount = 0;
  bool triggered = false;
  uint32_t obstacleId = 0;

  bool sensorLost = true;
  bool dirty = false;
  uint32_t seq = 0;
  int lastDistance = -1;

  unsigned long lastFrameAt = millis();
  unsigned long lastValidAt = millis();
  unsigned long lastLostNotice = 0;
  unsigned long lastDistanceReport = 0;

  uint32_t lastRunUs = micros();

  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {

    uint32_t nowUs = micros();
    uint32_t period = nowUs - lastRunUs;
    lastRunUs = nowUs;

    if (period > obstacleMaxPeriodUs) {
      obstacleMaxPeriodUs = period;
    }

    bool alive = false;
    int distance = readTFmini(alive);
    unsigned long now = millis();

    if (alive) {
      lastFrameAt = now;
    }

    if (distance <= 0) {

      if (now - lastValidAt > (unsigned long)OBSTACLE_GAP_RESET_MS) {
        obstacleCount = 0;
        clearCount = 0;
      }

      bool noFrames = now - lastFrameAt > (unsigned long)TF_STALE_MS;
      bool noValid = now - lastValidAt > (unsigned long)TF_INVALID_LOST_MS;

      if (noFrames || noValid) {

        if (!sensorLost) {
          sensorLost = true;
          triggered = false;
          dirty = true;
          lastLostNotice = 0;

          logLine(noFrames ? "[TFmini] no frames -> SENSOR LOST"
                           : "[TFmini] no valid distance -> SENSOR LOST");
        }

        if (lastLostNotice == 0 ||
            now - lastLostNotice >= (unsigned long)TF_LOST_REPEAT_MS) {
          sendBLE("SENSOR:FRONT_LOST");
          lastLostNotice = now;
        }
      }
    }
    else {

      lastValidAt = now;
      lastDistance = distance;

      if (sensorLost) {
        sensorLost = false;
        dirty = true;
        logLine("[TFmini] sensor OK");
        sendBLE("SENSOR:FRONT_OK");
      }

      if (now - lastDistanceReport >= (unsigned long)REPORT_MS) {
        logLine("[TFmini] %d cm", distance);
        sendReportf("DIST:%d", distance);
        lastDistanceReport = now;
      }

      if (distance <= OBSTACLE_DISTANCE) {
        clearCount = 0;
        if (obstacleCount < OBSTACLE_CONFIRM_COUNT) {
          obstacleCount++;
        }
      }
      else {
        obstacleCount = 0;
      }

      if (!triggered && obstacleCount >= OBSTACLE_CONFIRM_COUNT) {
        triggered = true;
        obstacleId++;
        clearCount = 0;
        dirty = true;
        sendBLEf("OBSTACLE:%d", distance);
      }

      if (triggered) {
        if (distance >= RESET_DISTANCE) {
          if (++clearCount >= CLEAR_CONFIRM_COUNT) {
            triggered = false;
            obstacleCount = 0;
            clearCount = 0;
            dirty = true;
          }
        }
        else {
          clearCount = 0;
        }
      }
    }

    if (dirty) {
      if (sendFrontState(seq, sensorLost, triggered, obstacleId, lastDistance)) {
        dirty = false;
      }
      else {
        logLine("[FRONT] motor queue full - retry next cycle");
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(OBSTACLE_PERIOD_MS));
  }
}


// =====================================================
// VL53L1X
// =====================================================

void setupVL53L1X() {

  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(VL_LEFT_XSHUT, OUTPUT);
  pinMode(VL_RIGHT_XSHUT, OUTPUT);

  digitalWrite(VL_LEFT_XSHUT, LOW);
  digitalWrite(VL_RIGHT_XSHUT, LOW);

  delay(20);

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


// RangeValid / MinRangeClipped / NoWrapCheckFail만 쓰고, 나머지는 -1(막힘)
bool isUsableRange(VL53L1X::RangeStatus status) {
  return status == VL53L1X::RangeValid ||
         status == VL53L1X::RangeValidMinRangeClipped ||
         status == VL53L1X::RangeValidNoWrapCheckFail;
}

int updateSideSensor(VL53L1X& sensor, bool ok, unsigned long& lastUpdateAt,
                     std::atomic<int>& out, std::atomic<uint32_t>& outAt, const char* name) {

  if (!ok) {
    out = -1;
    return -1;
  }

  unsigned long now = millis();

  if (sensor.dataReady()) {

    uint16_t mm = sensor.read(false);
    VL53L1X::RangeStatus status = sensor.ranging_data.range_status;

    lastUpdateAt = now;

    if (isUsableRange(status)) {
      out = mm / 10;
      outAt = (uint32_t)now;
    }
    else {
      if (out != -1) {
        logLine("[VL53 %s] invalid: %s", name, VL53L1X::rangeStatusToString(status));
      }
      out = -1;
    }
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

    int leftDistance = updateSideSensor(vlLeft, vlLeftOK, leftUpdateAt, sideLeftCm, sideLeftAt, "LEFT");
    int rightDistance = updateSideSensor(vlRight, vlRightOK, rightUpdateAt, sideRightCm, sideRightAt, "RIGHT");

    if (millis() - lastSideDistanceSend >= (unsigned long)REPORT_MS) {

      logLine("[VL53] LEFT=%d cm | RIGHT=%d cm", leftDistance, rightDistance);

      sendReportf("SIDE:%d,%d", leftDistance, rightDistance);

      lastSideDistanceSend = millis();
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SIDE_POLL_MS));
  }
}


// =====================================================
// Raspberry Pi
//
// CROSSWALK:1 / CROSSWALK:0
// LIGHT:RED / LIGHT:GREEN / LIGHT:GREEN_WAIT / LIGHT:NONE
// CROSSING_START / CROSSING_END
// CROSS_MOTOR:L / CROSS_MOTOR:R / CROSS_MOTOR:CENTER
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


// 큐가 꽉 찼을 때 상태 이벤트는 받은 시각과 함께 대기열에 두고 순서대로 다시 넣는다.
// 방향 펄스(L/R)는 늦으면 의미가 없으므로 버린다.
// 대기열도 넘치면 가장 오래된 것부터 버린다. (Pi가 상태를 주기적으로 다시 보내야 복구됨)
MotorEvent piPending[PI_PENDING_LEN];
int piPendingHead = 0;
int piPendingCount = 0;

void flushPiPending() {

  while (piPendingCount > 0) {

    if (xQueueSend(motorQueue, &piPending[piPendingHead], 0) != pdTRUE) {
      return;
    }

    piPendingHead = (piPendingHead + 1) % PI_PENDING_LEN;
    piPendingCount--;
  }
}

void postPiEvent(EventType type) {

  MotorEvent ev = { type, 0, (uint32_t)micros(), 0, 0, 0 };

  bool pulse = type == EV_PI_CROSS_L || type == EV_PI_CROSS_R;

  if (pulse) {
    if (piPendingCount > 0 || xQueueSend(motorQueue, &ev, 0) != pdTRUE) {
      logLine("[PI] queue busy, steer pulse dropped");
    }
    return;
  }

  if (piPendingCount > 0) {

    // 같은 상태가 반복으로 오면 하나만 둔다. (처리 결과가 같음)
    int last = (piPendingHead + piPendingCount - 1) % PI_PENDING_LEN;

    if (piPending[last].type == type) {
      flushPiPending();
      return;
    }
  }

  if (piPendingCount == PI_PENDING_LEN) {
    logLine("[PI] pending full, oldest dropped (type=%d)", (int)piPending[piPendingHead].type);
    piPendingHead = (piPendingHead + 1) % PI_PENDING_LEN;
    piPendingCount--;
  }

  piPending[(piPendingHead + piPendingCount) % PI_PENDING_LEN] = ev;
  piPendingCount++;

  flushPiPending();

  if (piPendingCount > 0) {
    logLine("[PI] motor queue full, %d event(s) pending", piPendingCount);
  }
}


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
      postPiEvent(cmd.type);
      return;
    }
  }

  if (strcmp(message, "LIGHT:NONE") == 0) {
    sendBLE("LIGHT:NONE");
    return;
  }

  sendReport(message);
}


void piUartTask(void*) {

  char line[PI_LINE_MAX + 1];
  int len = 0;
  bool overflow = false;

  for (;;) {

    flushPiPending();

    while (PiSerial.available()) {

      char c = PiSerial.read();

      if (c == '\n') {
        if (!overflow) {
          line[len] = '\0';
          handlePiLine(line);
        }
        len = 0;
        overflow = false;
      }
      else if (c != '\r' && !overflow) {

        // 너무 긴 줄은 다음 \n까지 통째로 버린다.
        if (len >= PI_LINE_MAX) {
          overflow = true;
          logLine("[PI] line too long, dropped");
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
// BLE
// =====================================================

bool takeBleMessage(BleMsg& msg) {

  if (xQueueReceive(bleAlertQueue, &msg, 0) == pdTRUE) {
    return true;
  }

  // 버려진 알림이 있었으면, 알림 큐가 빈 지금 최신 상태를 한 번 보낸다.
  // 요청을 못 넣으면 표시를 되살려 다음에 다시 시도한다.
  if (bleConnected && bleAlertDropped.exchange(false)) {
    if (!postEvent(EV_APP_STATE_REQUEST)) {
      bleAlertDropped.store(true);
    }
  }

  if (xQueueReceive(bleReportQueue, &msg, 0) == pdTRUE) {
    return true;
  }

  return xQueueReceive(bleAlertQueue, &msg, pdMS_TO_TICKS(10)) == pdTRUE;
}


// 알림 큐를 먼저 보낸다.
void bleTxTask(void*) {

  BleMsg msg;

  for (;;) {

    uint32_t restartAt = advertiseRestartAt;

    if (restartAt != 0 && (int32_t)(millis() - restartAt) >= 0) {
      advertiseRestartAt = 0;
      BLEDevice::startAdvertising();
      logLine("BLE advertising restart");
    }

    if (!takeBleMessage(msg)) {
      continue;
    }

    if (!bleConnected || txCharacteristic == nullptr) {
      continue;
    }

    txCharacteristic->setValue(msg.text);
    txCharacteristic->notify();

    logLine("[BLE TX] %s", msg.text);
  }
}


void logTask(void*) {

  LogMsg msg;

  for (;;) {

    xQueueReceive(logQueue, &msg, portMAX_DELAY);

    Serial.println(msg.text);

    uint32_t dropped = droppedLogs.exchange(0);

    if (dropped > 0) {
      snprintf(msg.text, sizeof(msg.text), "[LOG] dropped lines: %lu", (unsigned long)dropped);
      Serial.println(msg.text);
    }
  }
}


float parseAngle(const String& command) {

  int colon = command.indexOf(':');

  if (colon < 0) {
    return 0;
  }

  return command.substring(colon + 1).toFloat();
}


// B:90 / F:3.5 / L:45.0 / R:32.0 / S:0 / Q
void handleFlutterCommand(String command) {

  command.trim();

  if (command.length() == 0) {
    return;
  }

  logLine("[BLE RX] %s", command.c_str());

  if (command.startsWith("B:")) {
    targetBearing = command.substring(2).toFloat();
    logLine("[NAV] Target bearing = %.2f", targetBearing);
    return;
  }

  if (command == "L" || command.startsWith("L:")) {
    postAppEvent(EV_APP_LEFT, parseAngle(command));
    return;
  }

  if (command == "R" || command.startsWith("R:")) {
    postAppEvent(EV_APP_RIGHT, parseAngle(command));
    return;
  }

  if (command == "F" || command.startsWith("F:")) {
    postAppEvent(EV_APP_FORWARD);
    return;
  }

  // 정지 유지는 풀지 않는다.
  if (command == "Q") {
    postEvent(EV_APP_STATE_REQUEST);
    return;
  }

  if (command == "S" || command.startsWith("S:")) {
    requestStop(false);
    return;
  }

  logLine("[BLE] Unknown command");
}


class RxCallback : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic* characteristic) override {

    String command = characteristic->getValue().c_str();

    handleFlutterCommand(command);
  }
};


class ServerCallback : public BLEServerCallbacks {

  void onConnect(BLEServer*) override {

    bleConnected = true;

    logLine("BLE CONNECTED");
  }

  // BLE 스택 콜백 안에서 기다리지 않도록 광고 재시작은 bleTxTask가 한다.
  void onDisconnect(BLEServer*) override {

    bleConnected = false;

    requestStop(true);

    uint32_t at = millis() + ADVERTISE_RESTART_DELAY_MS;
    advertiseRestartAt = (at == 0) ? 1 : at;

    logLine("BLE DISCONNECTED");
  }
};


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


// 초기화 실패: 모터 핀을 LOW로 묶고 BLE를 켜지 않은 채 멈춘다.
void haltOnInitError(const char* what) {

  TaskHandle_t* handles[] = {
    &obstacleTaskHandle, &motorTaskHandle, &piTaskHandle, &bleTxTaskHandle, &sideTaskHandle,
    &logTaskHandle
  };

  for (TaskHandle_t* h : handles) {
    if (*h != nullptr) {
      vTaskDelete(*h);
      *h = nullptr;
    }
  }

  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  digitalWrite(M1_PWM, LOW);
  digitalWrite(M2_PWM, LOW);

  for (;;) {
    Serial.print("[FATAL] init failed: ");
    Serial.println(what);
    delay(2000);
  }
}


bool startTask(TaskFunction_t fn, const char* name, uint32_t stack,
               UBaseType_t priority, TaskHandle_t* handle) {

  if (xTaskCreatePinnedToCore(fn, name, stack, nullptr, priority, handle, APP_CORE) != pdPASS) {
    if (handle != nullptr) {
      *handle = nullptr;
    }
    return false;
  }

  return true;
}


void setup() {

  Serial.begin(115200);

  delay(500);

  motorQueue = xQueueCreate(MOTOR_QUEUE_LEN, sizeof(MotorEvent));
  bleAlertQueue = xQueueCreate(BLE_ALERT_QUEUE_LEN, sizeof(BleMsg));
  bleReportQueue = xQueueCreate(BLE_REPORT_QUEUE_LEN, sizeof(BleMsg));
  logQueue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogMsg));

  if (motorQueue == nullptr || bleAlertQueue == nullptr ||
      bleReportQueue == nullptr || logQueue == nullptr) {
    haltOnInitError("queue (heap)");
  }

  if (!startTask(logTask, "log", 3072, 1, &logTaskHandle)) {
    haltOnInitError("logTask");
  }

  logLine("SMART CANE START (FreeRTOS)");

  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);

  if (!ledcAttach(M1_PWM, 5000, 8) || !ledcAttach(M2_PWM, 5000, 8)) {
    haltOnInitError("ledcAttach (MDD10A PWM)");
  }

  stopMotors();

  logLine("[OK] MDD10A");

  TFSerial.begin(115200, SERIAL_8N1, TF_RX, TF_TX);

  logLine("[OK] TFmini UART2");

  // 실패해도 멈추지 않는다. 그쪽은 -1(막힘)로 처리된다.
  setupVL53L1X();

  PiSerial.begin(115200, SERIAL_8N1, PI_RX, PI_TX);

  logLine("[OK] Raspberry Pi UART1");

  if (!startTask(motorTask,      "motor",    4096, 4, &motorTaskHandle))    haltOnInitError("motorTask");
  if (!startTask(obstacleTask,   "obstacle", 4096, 5, &obstacleTaskHandle)) haltOnInitError("obstacleTask");
  if (!startTask(piUartTask,     "piUart",   4096, 3, &piTaskHandle))       haltOnInitError("piUartTask");
  if (!startTask(bleTxTask,      "bleTx",    4096, 2, &bleTxTaskHandle))    haltOnInitError("bleTxTask");
  if (!startTask(sideSensorTask, "side",     4096, 1, &sideTaskHandle))     haltOnInitError("sideSensorTask");

#if DEBUG_MONITOR
  if (!startTask(monitorTask,    "monitor",  3072, 1, nullptr)) {
    logLine("[WARN] monitorTask not started");
  }
#endif

  logLine("[OK] Tasks started");

  BLEDevice::init("SMART_CANE");

  BLEServer* server = BLEDevice::createServer();

  server->setCallbacks(new ServerCallback());

  BLEService* service = server->createService(SERVICE_UUID);

  BLECharacteristic* rxCharacteristic =
      service->createCharacteristic(
        RX_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
      );

  rxCharacteristic->setCallbacks(new RxCallback());

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
}


void loop() {
  vTaskDelete(nullptr);
}
