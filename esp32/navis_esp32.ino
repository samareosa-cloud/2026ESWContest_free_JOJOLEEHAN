#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <VL53L1X.h>


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

String piBuffer = "";


// =====================================================
// BLE UUID
// Flutter 앱과 반드시 같아야 함
// =====================================================

#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


BLECharacteristic* txCharacteristic = nullptr;

bool bleConnected = false;


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

// 저번 장애물 회피 로직
// TFmini 40cm 이하가 3회 연속 들어와야 실제 장애물로 판단
const int OBSTACLE_CONFIRM_COUNT = 3;

// 좌/우 VL53L1X 판단 기준
const int SIDE_BLOCK_DISTANCE = 30;
const int DIRECTION_MARGIN = 20;

int obstacleCount = 0;
bool obstacleTriggered = false;


// =====================================================
// 장애물 회피 유도
//
// 역토크 후 고른 방향(좌/우)으로, 정면이 트일 때
// (TFmini RESET_DISTANCE 이상)까지 계속 당겨 준다.
// 한 번만 당기고 끝내지 않고, 정면이 여전히
// 막혀 있어도 아무 동작을 하지 않았다.
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

AvoidState avoidState = AVOID_NONE;

int avoidPulseCount = 0;
unsigned long lastAvoidPulseAt = 0;
unsigned long avoidPulseEndAt = 0;
bool avoidPulseRunning = false;

// loop에서 300ms마다 갱신되는 최근 좌/우 거리 (cm, 실패 시 -1)
int lastLeftDistance = -1;
int lastRightDistance = -1;


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
// 횡단보도 모드
//
// true:
// 앱에서 오는 일반 L/R 유도 명령 무시
// =====================================================

bool crosswalkMode = false;
bool trafficRed = false;


// =====================================================
// 횡단 중 방향 보정 (Raspberry Pi CROSS_MOTOR:L/R/CENTER)
//
// Pi는 건너는 동안 현재 보정 방향을 0.3초마다 계속 보낸다.
// ESP32는 받은 방향을 "상태"로 기억하고, CENTER가 오거나
// 방향이 바뀔 때까지 CROSS_REPEAT_MS마다 계속 당긴다.
// 안전장치: CROSS_TIMEOUT_MS 동안 Pi 소식이 없으면 스스로 멈춘다.
// =====================================================

const int CROSS_PWM = 110;
const int CROSS_PULSE_TIME = 300;    // 한 번 당기는 시간 (ms)
const int CROSS_REPEAT_MS = 600;     // 당김 시작 간격 (ms)
const int CROSS_TIMEOUT_MS = 1000;   // Pi 메시지 끊김 판단 (ms)

char crossSteer = 'C';               // 'L' / 'R' / 'C'
unsigned long lastCrossMsgAt = 0;
unsigned long lastCrossPulseAt = 0;
unsigned long crossPulseEndAt = 0;
bool crossPulseRunning = false;
bool crossPulseStartNow = false;


// =====================================================
// Flutter에서 받은 목표 방위각
//
// 현재는 로그 확인용.
// 실제 좌/우 계산은 Flutter에서 함.
// =====================================================

float targetBearing = -1;


// =====================================================
// BLE Notify
// ESP32 -> Flutter
// =====================================================

void sendBLE(String message) {

  if (!bleConnected) {
    return;
  }

  if (txCharacteristic == nullptr) {
    return;
  }

  txCharacteristic->setValue(
    message.c_str()
  );

  txCharacteristic->notify();


  Serial.print("[BLE TX] ");
  Serial.println(message);
}


// =====================================================
// 모터 정지
// =====================================================

void stopMotors() {

  ledcWrite(
    M1_PWM,
    0
  );

  ledcWrite(
    M2_PWM,
    0
  );
}


// =====================================================
// PWM 램프업
// 갑자기 목표 PWM을 넣지 않고 0 -> 목표값으로 단계적으로 증가
// =====================================================

void rampSingleMotor(int pwmPin, int targetPwm) {

  const int stepDelay =
      MOTOR_RAMP_TIME_MS / MOTOR_RAMP_STEPS;

  for (int i = 1; i <= MOTOR_RAMP_STEPS; i++) {

    int pwm =
        (targetPwm * i) / MOTOR_RAMP_STEPS;

    ledcWrite(
      pwmPin,
      pwm
    );

    delay(stepDelay);
  }
}


void rampBothMotors(int targetPwm) {

  const int stepDelay =
      MOTOR_RAMP_TIME_MS / MOTOR_RAMP_STEPS;

  for (int i = 1; i <= MOTOR_RAMP_STEPS; i++) {

    int pwm =
        (targetPwm * i) / MOTOR_RAMP_STEPS;

    ledcWrite(
      M1_PWM,
      pwm
    );

    ledcWrite(
      M2_PWM,
      pwm
    );

    delay(stepDelay);
  }
}


// =====================================================
// LEFT 유도
//
// 사용자가 앞으로 끌고 있기 때문에
// 오른쪽 바퀴 M2에 짧게 힘을 줘서 왼쪽 방향 유도
// =====================================================

void guideLeft(float angle) {

  if (obstacleTriggered) {
    Serial.println(
      "[MOTOR] LEFT ignored - obstacle"
    );

    return;
  }


  if (crosswalkMode) {
    Serial.println(
      "[MOTOR] LEFT ignored - crosswalk mode"
    );

    return;
  }


  Serial.print(
    "[MOTOR] LEFT | angle="
  );

  Serial.println(angle);


  // M1 정지
  ledcWrite(
    M1_PWM,
    0
  );


  // M2 일반 방향
  digitalWrite(
    M2_DIR,
    LOW
  );


  // M2 짧게 동작
  rampSingleMotor(
    M2_PWM,
    GUIDE_PWM
  );


  delay(
    GUIDE_TIME
  );


  ledcWrite(
    M2_PWM,
    0
  );


  sendBLE(
    "MOTOR:L"
  );
}


// =====================================================
// RIGHT 유도
//
// 왼쪽 바퀴 M1에 짧게 힘을 줘서 오른쪽 방향 유도
// =====================================================

void guideRight(float angle) {

  if (obstacleTriggered) {

    Serial.println(
      "[MOTOR] RIGHT ignored - obstacle"
    );

    return;
  }


  if (crosswalkMode) {

    Serial.println(
      "[MOTOR] RIGHT ignored - crosswalk mode"
    );

    return;
  }


  Serial.print(
    "[MOTOR] RIGHT | angle="
  );

  Serial.println(angle);


  // M2 정지
  ledcWrite(
    M2_PWM,
    0
  );


  // M1 일반 방향
  digitalWrite(
    M1_DIR,
    LOW
  );


  // M1 짧게 동작
  rampSingleMotor(
    M1_PWM,
    GUIDE_PWM
  );


  delay(
    GUIDE_TIME
  );


  ledcWrite(
    M1_PWM,
    0
  );


  sendBLE(
    "MOTOR:R"
  );
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


  Serial.println(
    "[MOTOR] FORWARD / NEUTRAL"
  );
}

// =====================================================
// 횡단보도 보정 상태 변경
// Raspberry Pi -> CROSS_MOTOR:L / R / CENTER
//
// 여기서는 상태만 바꾸고, 실제 당김은 updateCrosswalkSteering()이
// loop에서 반복한다. 앱에는 방향이 바뀔 때만 알린다.
// =====================================================

void setCrossSteer(char dir) {

  lastCrossMsgAt = millis();

  if (dir == crossSteer) {
    return;
  }

  crossSteer = dir;

  // 당기던 중이면 끊고, 새 방향은 바로 한 번 당긴다.
  if (crossPulseRunning && !obstacleTriggered) {
    stopMotors();
  }

  crossPulseRunning = false;
  crossPulseStartNow = true;

  Serial.print("[CROSS MOTOR] ");
  Serial.println(dir);

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
}


void crosswalkLeft() {
  setCrossSteer('L');
}


void crosswalkRight() {
  setCrossSteer('R');
}


void crosswalkCenter() {
  setCrossSteer('C');
}


// =====================================================
// 횡단 보정 반복 (loop에서 매번 호출)
// =====================================================

void updateCrosswalkSteering() {

  unsigned long now = millis();

  // 당기는 시간이 끝났으면 끄기
  if (
    crossPulseRunning
    &&
    (long)(now - crossPulseEndAt) >= 0
  ) {

    if (!obstacleTriggered) {
      stopMotors();
    }

    crossPulseRunning = false;
  }

  // 장애물 회피가 최우선
  if (obstacleTriggered) {
    crossPulseRunning = false;
    return;
  }

  if (
    !crosswalkMode
    ||
    trafficRed
    ||
    (crossSteer != 'L' && crossSteer != 'R')
  ) {
    return;
  }

  // Pi 메시지가 끊기면 멈춤 (Pi 멈춤 / 케이블 빠짐 대비)
  if (now - lastCrossMsgAt > (unsigned long)CROSS_TIMEOUT_MS) {

    Serial.println("[CROSS MOTOR] Pi timeout -> CENTER");

    stopCrossSteer();
    sendBLE("CROSS_MOTOR:CENTER");
    return;
  }

  if (crossPulseRunning) {
    return;
  }

  if (
    !crossPulseStartNow
    &&
    now - lastCrossPulseAt < (unsigned long)CROSS_REPEAT_MS
  ) {
    return;
  }

  crossPulseStartNow = false;

  if (crossSteer == 'L') {
    // 오른쪽 바퀴 M2로 왼쪽 유도
    ledcWrite(M1_PWM, 0);
    digitalWrite(M2_DIR, LOW);
    rampSingleMotor(M2_PWM, CROSS_PWM);
  }
  else {
    // 왼쪽 바퀴 M1로 오른쪽 유도
    ledcWrite(M2_PWM, 0);
    digitalWrite(M1_DIR, LOW);
    rampSingleMotor(M1_PWM, CROSS_PWM);
  }

  crossPulseRunning = true;

  // 램프업이 끝난 시점부터 펄스 유지시간 계산
  lastCrossPulseAt = millis();

  crossPulseEndAt = lastCrossPulseAt + CROSS_PULSE_TIME;
}


// =====================================================
// 장애물 역토크 3회
// =====================================================

void obstacleWarning() {

  Serial.println();
  Serial.println(
    "*** OBSTACLE WARNING ***"
  );

  reverseTorquePulses(BRAKE_COUNT);
}


// =====================================================
// 역토크 count회 (장애물 경고 / 빨간불 정지 알림 공용)
// =====================================================

void reverseTorquePulses(int count) {


  // 진행 방향 반대
  digitalWrite(
    M1_DIR,
    HIGH
  );

  digitalWrite(
    M2_DIR,
    HIGH
  );


  for (
    int i = 0;
    i < count;
    i++
  ) {

    Serial.print(
      "[BRAKE] "
    );

    Serial.print(
      i + 1
    );

    Serial.print(
      "/"
    );

    Serial.println(
      count
    );


    // 역토크 ON: 두 모터를 동시에 0 -> BRAKE_PWM으로 램프업
    rampBothMotors(BRAKE_PWM);


    delay(
      BRAKE_ON_TIME
    );


    // OFF
    ledcWrite(
      M1_PWM,
      0
    );

    ledcWrite(
      M2_PWM,
      0
    );


    delay(
      BRAKE_OFF_TIME
    );
  }


  stopMotors();


  // 일반 방향으로 복귀
  digitalWrite(
    M1_DIR,
    LOW
  );

  digitalWrite(
    M2_DIR,
    LOW
  );


  Serial.println(
    "*** BRAKE END ***"
  );
}


// =====================================================
// 장애물 회피용 당김 (non-blocking)
//
// 기존 guideLeft()/guideRight()는 obstacleTriggered=true일 때
// 무시하므로 회피 전용 함수를 따로 둔다.
// delay()로 기다리지 않고 끝나는 시각만 기록해서,
// 당기는 동안에도 TFmini/BLE 처리가 계속 돌게 한다.
//
// LEFT : 오른쪽 바퀴 M2 구동
// RIGHT: 왼쪽 바퀴 M1 구동
// =====================================================

void startAvoidPulse(AvoidState side) {

  bool left = side == AVOID_LEFT;

  Serial.print("[AVOID] PULL ");
  Serial.print(left ? "LEFT" : "RIGHT");
  Serial.print(" #");
  Serial.println(avoidPulseCount + 1);

  if (left) {
    ledcWrite(M1_PWM, 0);
    digitalWrite(M2_DIR, LOW);
    rampSingleMotor(M2_PWM, AVOID_PWM);
  }
  else {
    ledcWrite(M2_PWM, 0);
    digitalWrite(M1_DIR, LOW);
    rampSingleMotor(M1_PWM, AVOID_PWM);
  }

  avoidPulseRunning = true;
  lastAvoidPulseAt = millis();
  avoidPulseEndAt = lastAvoidPulseAt + AVOID_PULSE_TIME;
  avoidPulseCount++;

  sendBLE(left ? "MOTOR:L" : "MOTOR:R");
}


// 당기는 시간이 끝났으면 모터를 끈다. loop에서 매번 호출.
void updateAvoidPulse() {

  if (
    avoidPulseRunning
    &&
    (long)(millis() - avoidPulseEndAt) >= 0
  ) {

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
// VL53L1X 2개 초기화
// =====================================================

void setupVL53L1X() {

  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(VL_LEFT_XSHUT, OUTPUT);
  pinMode(VL_RIGHT_XSHUT, OUTPUT);

  // 둘 다 끄기
  digitalWrite(VL_LEFT_XSHUT, LOW);
  digitalWrite(VL_RIGHT_XSHUT, LOW);

  delay(20);

  // -------------------------------------------------
  // LEFT 먼저 켜고 0x30으로 주소 변경
  // -------------------------------------------------
  digitalWrite(VL_LEFT_XSHUT, HIGH);
  delay(20);

  vlLeft.setTimeout(100);

  if (vlLeft.init()) {

    vlLeft.setAddress(VL_LEFT_ADDR);
    vlLeft.setDistanceMode(VL53L1X::Long);
    vlLeft.setMeasurementTimingBudget(50000);
    vlLeft.startContinuous(50);

    vlLeftOK = true;

    Serial.println("[OK] VL53L1X LEFT  addr=0x30");
  }
  else {

    vlLeftOK = false;

    Serial.println("[ERROR] VL53L1X LEFT init failed");
  }


  // -------------------------------------------------
  // RIGHT 켜고 기본주소 0x29에서 0x31로 변경
  // -------------------------------------------------
  digitalWrite(VL_RIGHT_XSHUT, HIGH);
  delay(20);

  vlRight.setTimeout(100);

  if (vlRight.init()) {

    vlRight.setAddress(VL_RIGHT_ADDR);
    vlRight.setDistanceMode(VL53L1X::Long);
    vlRight.setMeasurementTimingBudget(50000);
    vlRight.startContinuous(50);

    vlRightOK = true;

    Serial.println("[OK] VL53L1X RIGHT addr=0x31");
  }
  else {

    vlRightOK = false;

    Serial.println("[ERROR] VL53L1X RIGHT init failed");
  }
}


// =====================================================
// VL53L1X LEFT 거리 읽기 (cm)
// =====================================================

int readVL53Left() {

  if (!vlLeftOK) {
    return -1;
  }

  uint16_t mm = vlLeft.read();

  if (vlLeft.timeoutOccurred()) {

    Serial.println("[VL53 LEFT] timeout");

    return -1;
  }

  return mm / 10;
}


// =====================================================
// VL53L1X RIGHT 거리 읽기 (cm)
// =====================================================

int readVL53Right() {

  if (!vlRightOK) {
    return -1;
  }

  uint16_t mm = vlRight.read();

  if (vlRight.timeoutOccurred()) {

    Serial.println("[VL53 RIGHT] timeout");

    return -1;
  }

  return mm / 10;
}


// =====================================================
// 장애물 확정 시 1회 실행
//
// 1) 역토크 3회로 멈춤 알림
// 2) 좌/우 VL53L1X로 회피 방향 결정 (chooseAvoidSide)
// 3) 그 방향으로 첫 당김 시작
//    -> 이후 updateObstacleAvoidance()가 정면이 트일 때까지 반복
// =====================================================

void handleObstacleAvoidance(int frontDistance) {

  // ===================================================
  // 1. 정면 장애물 확정 후 역토크 3회 먼저 실행
  // ===================================================
  stopMotors();

  Serial.println();
  Serial.println("[AVOID] FRONT OBSTACLE CONFIRMED -> BRAKE FIRST");

  // Flutter 앱에 장애물 음성 안내 요청.
  // 장애물 1건당 역토크 시작 직전에 딱 한 번만 전송한다.
  sendBLE("OBSTACLE_BRAKE");

  // BLE Notify가 앱으로 전달될 시간을 아주 짧게 확보.
  delay(50);

  // 역토크 3회
  obstacleWarning();

  // 역토크 직후 잠깐 안정화
  delay(100);

  // 역토크 동안 쌓인 옛날 거리 데이터 버리기
  flushTFmini();


  // ===================================================
  // 2. 좌우 VL53L1X 거리 측정
  // ===================================================
  int leftDistance = readVL53Left();
  int rightDistance = readVL53Right();


  Serial.println();
  Serial.println("========== OBSTACLE AVOID ==========");

  Serial.print("FRONT = ");
  Serial.print(frontDistance);
  Serial.println(" cm");

  Serial.print("LEFT  = ");
  Serial.print(leftDistance);
  Serial.println(" cm");

  Serial.print("RIGHT = ");
  Serial.print(rightDistance);
  Serial.println(" cm");


  lastLeftDistance = leftDistance;
  lastRightDistance = rightDistance;

  avoidPulseCount = 0;
  avoidPulseRunning = false;
  avoidState = AVOID_NONE;

  AvoidState side = chooseAvoidSide(leftDistance, rightDistance);

  setAvoidState(side);

  if (side == AVOID_BLOCKED) {
    // 양쪽 다 막힘 (또는 센서 읽기 실패). 역토크는 이미 실행됨.
    Serial.println("[AVOID] BOTH SIDES BLOCKED -> STOP");
    return;
  }

  startAvoidPulse(side);
}


// =====================================================
// 장애물 회피 중 반복 (loop에서 obstacleTriggered일 때 매번 호출)
//
// 정면이 아직 막혀 있으면 AVOID_REPEAT_MS마다 다시 당긴다.
// 당길 때마다 좌/우를 다시 확인해서, 피하던 쪽이 막히면
// 반대쪽으로 바꾸고 둘 다 막히면 멈춘다.
// AVOID_MAX_PULSES번 당겨도 정면이 안 트이면 "길 막힘"으로
// 역토크를 한 번 더 주고 멈춘다.
// =====================================================

void updateObstacleAvoidance() {

  updateAvoidPulse();

  if (
    avoidState != AVOID_LEFT
    &&
    avoidState != AVOID_RIGHT
  ) {
    return;
  }

  if (avoidPulseRunning) {
    return;
  }

  if (millis() - lastAvoidPulseAt < (unsigned long)AVOID_REPEAT_MS) {
    return;
  }

  if (avoidPulseCount >= AVOID_MAX_PULSES) {

    Serial.println("[AVOID] front still blocked -> BLOCKED");

    setAvoidState(AVOID_BLOCKED);

    obstacleWarning();
    flushTFmini();

    return;
  }

  AvoidState side =
      chooseAvoidSide(lastLeftDistance, lastRightDistance);

  setAvoidState(side);

  if (
    side == AVOID_LEFT
    ||
    side == AVOID_RIGHT
  ) {

    startAvoidPulse(side);
  }
}


// =====================================================
// TFmini 읽기
//
// 버퍼에 쌓인 프레임을 모두 읽고 "가장 최근" 거리를 돌려준다.
// 역토크처럼 delay()가 긴 동작 뒤에는 옛날 프레임이 쌓여 있어서,
// 첫 프레임만 읽으면 이미 지난 거리로 판단하게 된다.
// =====================================================

int readTFmini() {

  int latest = -1;

  while (
    TFSerial.available() >= 9
  ) {

    // 첫 번째 헤더
    if (
      TFSerial.read() != 0x59
    ) {
      continue;
    }


    // 두 번째 헤더 (아니면 버리지 않고 다음 바이트부터 다시 찾음)
    if (
      TFSerial.peek() != 0x59
    ) {
      continue;
    }

    TFSerial.read();


    uint8_t buf[9];

    buf[0] = 0x59;
    buf[1] = 0x59;

    // available() >= 9 였으므로 나머지 7바이트는 이미 들어와 있음
    for (
      int i = 2;
      i < 9;
      i++
    ) {

      buf[i] =
          TFSerial.read();
    }


    // checksum
    uint16_t checksum = 0;

    for (
      int i = 0;
      i < 8;
      i++
    ) {

      checksum +=
          buf[i];
    }

    checksum &=
        0xFF;


    if (
      checksum != buf[8]
    ) {

      Serial.println(
        "[TFmini] checksum error"
      );

      continue;
    }


    // 거리 cm
    latest =
        buf[2] |
        (buf[3] << 8);
  }


  return latest;
}


// 긴 동작(역토크 등) 뒤에 쌓인 옛날 TFmini 데이터를 버린다.
void flushTFmini() {

  while (
    TFSerial.available()
  ) {

    TFSerial.read();
  }
}


// =====================================================
// Raspberry Pi 메시지 처리
//
// Raspberry Pi에서 아래처럼 보내면 됨. (docs/PROTOCOL.md)
//
// CROSSWALK:1
// CROSSWALK:0
//
// LIGHT:RED
// LIGHT:GREEN
// LIGHT:GREEN_WAIT
// LIGHT:NONE
//
// CROSSING_START / CROSSING_END
// CROSS_MOTOR:L / CROSS_MOTOR:R / CROSS_MOTOR:CENTER  (0.3초마다 반복)
//
// 각 메시지 뒤에는 반드시 \n
// =====================================================

void handlePiMessage(
  String message
) {

  message.trim();


  if (
    message.length() == 0
  ) {

    return;
  }


  Serial.print(
    "[PI RX] "
  );

  Serial.println(
    message
  );


  // -------------------------------------------------
  // 횡단보도 감지
  // -------------------------------------------------

  if (
    message == "CROSSWALK:1"
  ) {

    crosswalkMode = true;

    stopCrossSteer();


    sendBLE(
      "CROSSWALK:1"
    );


    return;
  }


  // -------------------------------------------------
  // 횡단보도 사라짐
  // -------------------------------------------------

  if (
    message == "CROSSWALK:0"
  ) {

    crosswalkMode = false;
    trafficRed = false;

    stopCrossSteer();


    sendBLE(
      "CROSSWALK:0"
    );


    return;
  }


  // -------------------------------------------------
  // 빨간불
  // -------------------------------------------------
  if (
    message == "LIGHT:RED"
    ||
    message == "SIGNAL_RED"
  ) {

    // 빨간불로 "바뀐" 순간에만 역토크 (같은 메시지 반복 시 다시 안 줌)
    bool turnedRed = !trafficRed;

    crosswalkMode = true;
    trafficRed = true;

    stopCrossSteer();

    sendBLE(
      "LIGHT:RED"
    );

    // 장애물 회피 중에는 그쪽 동작을 우선
    if (turnedRed && !obstacleTriggered) {

      Serial.println("[TRAFFIC] RED -> stop warning");

      reverseTorquePulses(RED_BRAKE_COUNT);
      flushTFmini();
    }

    return;
  }


  // -------------------------------------------------
  // 초록불
  // -------------------------------------------------
  if (
    message == "LIGHT:GREEN"
    ||
    message == "SIGNAL_GREEN"
  ) {

    crosswalkMode = true;
    trafficRed = false;

    stopCrossSteer();

    sendBLE(
      "LIGHT:GREEN"
    );

    return;
  }


  // -------------------------------------------------
  // 초록불이 이미 켜져 있음 (빨간불을 못 봄)
  // 남은 시간을 모르므로 건너지 않고 다음 신호를 기다린다.
  // -------------------------------------------------
  if (
    message == "LIGHT:GREEN_WAIT"
  ) {

    crosswalkMode = true;
    trafficRed = true;

    stopCrossSteer();

    sendBLE(
      "LIGHT:GREEN_WAIT"
    );

    return;
  }


  // -------------------------------------------------
  // 신호등 인식 안 됨
  // -------------------------------------------------
  if (
    message == "LIGHT:NONE"
  ) {

    sendBLE(
      "LIGHT:NONE"
    );

    return;
  }


  // -------------------------------------------------
  // 횡단 시작
  // -------------------------------------------------
  if (
    message == "CROSSING_START"
  ) {

    crosswalkMode = true;
    trafficRed = false;

    stopCrossSteer();

    sendBLE(
      "CROSSING_START"
    );

    return;
  }


  // -------------------------------------------------
  // 횡단 중 왼쪽 보정
  // -------------------------------------------------
  if (
    message == "CROSS_MOTOR:L"
  ) {

    crosswalkLeft();

    return;
  }


  // -------------------------------------------------
  // 횡단 중 오른쪽 보정
  // -------------------------------------------------
  if (
    message == "CROSS_MOTOR:R"
  ) {

    crosswalkRight();

    return;
  }


  // -------------------------------------------------
  // 횡단 중 중앙 유지
  // -------------------------------------------------
  if (
    message == "CROSS_MOTOR:CENTER"
  ) {

    crosswalkCenter();

    return;
  }


  // -------------------------------------------------
  // 횡단 종료
  // -------------------------------------------------
  if (
    message == "CROSSING_END"
  ) {

    crosswalkMode = false;
    trafficRed = false;

    stopCrossSteer();

    sendBLE(
      "CROSSING_END"
    );

    return;
  }


  // -------------------------------------------------
  // 그 외 Raspberry Pi 메시지는
  // 앱에서도 확인할 수 있도록 그대로 전달
  // -------------------------------------------------

  sendBLE(
    message
  );
}


// =====================================================
// Raspberry Pi UART 읽기
// =====================================================

void readRaspberryPi() {

  while (
    PiSerial.available()
  ) {

    char c =
        PiSerial.read();


    if (
      c == '\n'
    ) {

      handlePiMessage(
        piBuffer
      );


      piBuffer = "";
    }

    else if (
      c != '\r'
    ) {

      piBuffer += c;


      // 비정상 데이터 방지
      if (
        piBuffer.length() > 100
      ) {

        piBuffer = "";
      }
    }
  }
}


// =====================================================
// Flutter BLE 명령 처리
//
// 현재 Flutter 앱에서:
//
// B:90
// F:3.5
// L:45.0
// R:32.0
// S:0
//
// 형태로 들어올 수 있음.
// =====================================================

void handleFlutterCommand(
  String command
) {

  command.trim();


  if (
    command.length() == 0
  ) {

    return;
  }


  Serial.print(
    "[BLE RX] "
  );

  Serial.println(
    command
  );


  // -------------------------------------------------
  // 목표 방위각
  //
  // 예: B:135
  // -------------------------------------------------

  if (
    command.startsWith(
      "B:"
    )
  ) {

    targetBearing =
        command
            .substring(2)
            .toFloat();


    Serial.print(
      "[NAV] Target bearing = "
    );

    Serial.println(
      targetBearing
    );


    return;
  }


  // -------------------------------------------------
  // LEFT
  //
  // L
  // L:45
  // 둘 다 처리
  // -------------------------------------------------

  if (
    command == "L"
    ||
    command.startsWith(
      "L:"
    )
  ) {

    float angle = 0;


    int colon =
        command.indexOf(':');


    if (
      colon >= 0
    ) {

      angle =
          command
              .substring(
                colon + 1
              )
              .toFloat();
    }


    guideLeft(
      angle
    );


    return;
  }


  // -------------------------------------------------
  // RIGHT
  // -------------------------------------------------

  if (
    command == "R"
    ||
    command.startsWith(
      "R:"
    )
  ) {

    float angle = 0;


    int colon =
        command.indexOf(':');


    if (
      colon >= 0
    ) {

      angle =
          command
              .substring(
                colon + 1
              )
              .toFloat();
    }


    guideRight(
      angle
    );


    return;
  }


  // -------------------------------------------------
  // FORWARD
  //
  // 실제 모터 직진 X
  // 유도 모터만 정지
  // -------------------------------------------------

  if (
    command == "F"
    ||
    command.startsWith(
      "F:"
    )
  ) {

    guideForward();


    return;
  }


  // -------------------------------------------------
  // STOP
  // -------------------------------------------------

  if (
    command == "S"
    ||
    command.startsWith(
      "S:"
    )
  ) {

    stopMotors();


    sendBLE(
      "MOTOR:STOP"
    );


    return;
  }


  Serial.println(
    "[BLE] Unknown command"
  );
}


// =====================================================
// BLE RX Callback
// =====================================================

class RxCallback
    : public BLECharacteristicCallbacks {

  void onWrite(
    BLECharacteristic*
        characteristic
  ) override {

    String command =
        characteristic
            ->getValue()
            .c_str();


    handleFlutterCommand(
      command
    );
  }
};


// =====================================================
// BLE 연결 상태
// =====================================================

class ServerCallback
    : public BLEServerCallbacks {

  void onConnect(
    BLEServer*
  ) override {

    bleConnected = true;


    Serial.println();
    Serial.println(
      "BLE CONNECTED"
    );
  }


  void onDisconnect(
    BLEServer* server
  ) override {

    bleConnected = false;


    stopMotors();


    Serial.println();
    Serial.println(
      "BLE DISCONNECTED"
    );


    delay(
      200
    );


    server
        ->getAdvertising()
        ->start();


    Serial.println(
      "BLE advertising restart"
    );
  }
};


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(
    500
  );


  Serial.println();
  Serial.println(
    "=============================="
  );

  Serial.println(
    "SMART CANE START"
  );

  Serial.println(
    "=============================="
  );


  // =================================================
  // Motor
  // =================================================

  pinMode(
    M1_DIR,
    OUTPUT
  );

  pinMode(
    M2_DIR,
    OUTPUT
  );


  digitalWrite(
    M1_DIR,
    LOW
  );

  digitalWrite(
    M2_DIR,
    LOW
  );


  ledcAttach(
    M1_PWM,
    5000,
    8
  );

  ledcAttach(
    M2_PWM,
    5000,
    8
  );


  stopMotors();


  Serial.println(
    "[OK] MDD10A"
  );


  // =================================================
  // TFmini
  // =================================================

  TFSerial.begin(
    115200,
    SERIAL_8N1,
    TF_RX,
    TF_TX
  );


  Serial.println(
    "[OK] TFmini UART2"
  );


  // =================================================
  // VL53L1X LEFT / RIGHT
  // =================================================

  setupVL53L1X();


  // =================================================
  // Raspberry Pi UART
  // =================================================

  PiSerial.begin(
    115200,
    SERIAL_8N1,
    PI_RX,
    PI_TX
  );


  Serial.println(
    "[OK] Raspberry Pi UART1"
  );


  // =================================================
  // BLE
  // =================================================

  BLEDevice::init(
    "SMART_CANE"
  );


  BLEServer* server =
      BLEDevice::createServer();


  server->setCallbacks(
    new ServerCallback()
  );


  BLEService* service =
      server->createService(
        SERVICE_UUID
      );


  // Flutter -> ESP32
  BLECharacteristic*
      rxCharacteristic =
          service
              ->createCharacteristic(
                RX_UUID,

                BLECharacteristic::
                    PROPERTY_WRITE
                |
                BLECharacteristic::
                    PROPERTY_WRITE_NR
              );


  rxCharacteristic
      ->setCallbacks(
        new RxCallback()
      );


  // ESP32 -> Flutter
  txCharacteristic =
      service
          ->createCharacteristic(
            TX_UUID,

            BLECharacteristic::
                PROPERTY_NOTIFY
          );


  txCharacteristic
      ->addDescriptor(
        new BLE2902()
      );


  service->start();


  BLEAdvertising* advertising =
      BLEDevice
          ::getAdvertising();


  advertising
      ->addServiceUUID(
        SERVICE_UUID
      );


  advertising
      ->setScanResponse(
        true
      );


  BLEDevice
      ::startAdvertising();


  Serial.println(
    "[OK] BLE SMART_CANE"
  );


  Serial.println();
  Serial.println(
    "TFmini warning:"
  );

  Serial.println(
    " <= 100 cm : Flutter screen warning"
  );

  Serial.println(
    " <= 40 cm x3 : voice alert + reverse torque x3 + compare LEFT/RIGHT"
  );

  Serial.println(
    " then pull toward open side until front >= 50 cm"
  );

  Serial.println(
    " both side < 30 cm or still blocked : stop"
  );

  Serial.println();
}


// =====================================================
// LOOP
// =====================================================

void loop() {

  // =================================================
  // Raspberry Pi 데이터 확인
  // =================================================

  readRaspberryPi();


  // =================================================
  // LEFT / RIGHT VL53L1X 거리 -> Flutter
  // 300 ms마다 SIDE:왼쪽cm,오른쪽cm 전송
  // 예: SIDE:82,135
  // =================================================

  static unsigned long lastSideDistanceSend = 0;

  if (
    millis() - lastSideDistanceSend >= 300
  ) {

    int leftDistance = readVL53Left();
    int rightDistance = readVL53Right();

    lastLeftDistance = leftDistance;
    lastRightDistance = rightDistance;

    Serial.print("[VL53] LEFT=");
    Serial.print(leftDistance);
    Serial.print(" cm | RIGHT=");
    Serial.print(rightDistance);
    Serial.println(" cm");

    // 읽기 실패 시 -1이 전송되고, Flutter에서는 -- cm로 표시
    sendBLE(
      "SIDE:"
      + String(leftDistance)
      + ","
      + String(rightDistance)
    );

    lastSideDistanceSend = millis();
  }


  // =================================================
  // 장애물 회피 중이면 정면이 트일 때까지 계속 유도
  // =================================================

  if (obstacleTriggered) {
    updateObstacleAvoidance();
  }


  // =================================================
  // 횡단 중이면 Pi 보정 방향으로 계속 유도
  // =================================================

  updateCrosswalkSteering();


  // =================================================
  // TFmini
  // =================================================

  int distance =
      readTFmini();


  if (
    distance > 0
  ) {

    // 너무 빠르게 시리얼 찍지 않도록
    static unsigned long
        lastDistancePrint = 0;


    if (
      millis()
          - lastDistancePrint
      >= 300
    ) {

      Serial.print(
        "[TFmini] "
      );

      Serial.print(
        distance
      );

      Serial.println(
        " cm"
      );


      lastDistancePrint =
          millis();
    }


    // -------------------------------------------------
    // 앱에 거리 전송
    // 300ms마다
    // -------------------------------------------------

    static unsigned long
        lastDistanceSend = 0;


    if (
      millis()
          - lastDistanceSend
      >= 300
    ) {

      sendBLE(
        "DIST:"
        + String(distance)
      );


      lastDistanceSend =
          millis();
    }


    // -------------------------------------------------
    // TFmini 40cm 이하가 3회 연속 들어오면
    // LEFT / RIGHT VL53L1X를 비교해서 회피 방향 결정
    // -------------------------------------------------

    if (
      distance <=
          OBSTACLE_DISTANCE
    ) {

      if (
        obstacleCount <
            OBSTACLE_CONFIRM_COUNT
      ) {

        obstacleCount++;

        Serial.print(
          "[OBSTACLE COUNT] "
        );

        Serial.print(
          obstacleCount
        );

        Serial.print(
          "/"
        );

        Serial.println(
          OBSTACLE_CONFIRM_COUNT
        );
      }
    }
    else {

      obstacleCount = 0;
    }


    if (
      obstacleCount >=
          OBSTACLE_CONFIRM_COUNT
      &&
      !obstacleTriggered
    ) {

      obstacleTriggered =
          true;


      // Flutter에도 전달
      sendBLE(
        "OBSTACLE:"
        + String(distance)
      );


      handleObstacleAvoidance(
        distance
      );
    }


    // -------------------------------------------------
    // 정면이 50cm 이상으로 다시 멀어지면 reset
    // -------------------------------------------------

    if (
      distance >=
          RESET_DISTANCE
      &&
      obstacleTriggered
    ) {

      obstacleTriggered =
          false;

      obstacleCount =
          0;

      // 회피 유도 종료
      stopMotors();
      avoidPulseRunning = false;
      avoidPulseCount = 0;
      avoidState = AVOID_NONE;


      sendBLE(
        "OBSTACLE_CLEAR"
      );

      Serial.println(
        "[TFmini] Obstacle cleared"
      );
    }
  }


  delay(
    10
  );
}
