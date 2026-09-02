#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

// =====================================================
// MDD10A
// =====================================================
#define M1_DIR 25
#define M1_PWM 27
#define M2_DIR 32
#define M2_PWM 14

// =====================================================
// TFmini + Servo
// TFmini TX(초록) -> GPIO16
// TFmini RX(흰색)  -> GPIO17 (안 연결해도 됨)
// Servo Signal     -> GPIO13
// Servo VCC        -> 외부 5V
// Servo GND        -> ESP32 GND 공통
// =====================================================
#define TF_RX 16
#define TF_TX 17
#define SERVO_PIN 13

HardwareSerial TFSerial(2);
Servo tfServo;

const int SERVO_CENTER = 90;
const int SERVO_LEFT   = 140;
const int SERVO_RIGHT  = 40;
const int SERVO_SETTLE_TIME = 500;
const int SCAN_SAMPLE_COUNT = 5;
const int SCAN_TIMEOUT = 1500;
const int SCAN_SAFE_DISTANCE = 50;
const int DIRECTION_MARGIN = 10;

// =====================================================
// Raspberry Pi UART
// Pi TX -> ESP32 GPIO18
// Pi RX <- ESP32 GPIO19
// Pi 없어도 코드 동작함
// =====================================================
#define PI_RX 18
#define PI_TX 19

HardwareSerial PiSerial(1);
String piBuffer = "";

// =====================================================
// BLE UUID
// =====================================================
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLECharacteristic* txCharacteristic = nullptr;
bool bleConnected = false;

// =====================================================
// 장애물 설정
// 30cm 이하 3회 연속 -> 장애물 확정
// 40cm 이상 -> reset
// =====================================================
const int OBSTACLE_DISTANCE = 30;
const int RESET_DISTANCE = 40;

bool obstacleTriggered = false;
int obstacleCount = 0;

// =====================================================
// 모터 설정
// =====================================================
const int GUIDE_PWM = 40;
const int GUIDE_TIME = 200;

const int BRAKE_PWM = 33;
const int BRAKE_ON_TIME = 300;
const int BRAKE_OFF_TIME = 200;
const int BRAKE_COUNT = 3;

// =====================================================
// 제어 모드
// 우선순위:
// 1) TFmini 장애물
// 2) Raspberry Pi 횡단보도
// 3) Flutter 일반 길찾기
// =====================================================
bool crosswalkMode = false;
bool redLightStop = false;

float targetBearing = -1;

// =====================================================
// BLE TX
// =====================================================
void sendBLE(String message) {
  if (!bleConnected || txCharacteristic == nullptr) return;

  txCharacteristic->setValue(message.c_str());
  txCharacteristic->notify();

  Serial.print("[BLE TX] ");
  Serial.println(message);
}

// =====================================================
// 기본 모터
// =====================================================
void stopMotors() {
  ledcWrite(M1_PWM, 0);
  ledcWrite(M2_PWM, 0);
}

// 오른쪽 바퀴 M2 -> 왼쪽 유도
void pulseLeftGuide() {
  ledcWrite(M1_PWM, 0);
  digitalWrite(M2_DIR, LOW);
  ledcWrite(M2_PWM, GUIDE_PWM);
  delay(GUIDE_TIME);
  ledcWrite(M2_PWM, 0);
}

// 왼쪽 바퀴 M1 -> 오른쪽 유도
void pulseRightGuide() {
  ledcWrite(M2_PWM, 0);
  digitalWrite(M1_DIR, LOW);
  ledcWrite(M1_PWM, GUIDE_PWM);
  delay(GUIDE_TIME);
  ledcWrite(M1_PWM, 0);
}

// =====================================================
// Flutter 일반 길안내
// 횡단보도 모드에서는 앱 L/R/F 차단
// =====================================================
void guideLeft(float angle) {
  if (obstacleTriggered) {
    Serial.println("[APP MOTOR] LEFT ignored - obstacle");
    return;
  }
  if (crosswalkMode) {
    Serial.println("[APP MOTOR] LEFT ignored - Pi has control");
    return;
  }

  Serial.print("[APP MOTOR] LEFT angle=");
  Serial.println(angle);
  pulseLeftGuide();
  sendBLE("MOTOR:L");
}

void guideRight(float angle) {
  if (obstacleTriggered) {
    Serial.println("[APP MOTOR] RIGHT ignored - obstacle");
    return;
  }
  if (crosswalkMode) {
    Serial.println("[APP MOTOR] RIGHT ignored - Pi has control");
    return;
  }

  Serial.print("[APP MOTOR] RIGHT angle=");
  Serial.println(angle);
  pulseRightGuide();
  sendBLE("MOTOR:R");
}

void guideForward() {
  if (obstacleTriggered || crosswalkMode) return;

  // 사용자가 직접 앞으로 끌기 때문에 F는 모터 중립
  stopMotors();
  Serial.println("[APP MOTOR] FORWARD / NEUTRAL");
}

// =====================================================
// Raspberry Pi 횡단보도 바퀴 제어
// CROSS_DIR:L / R / F / S
// 빨간불 및 장애물에서는 차단
// =====================================================
bool canUseCrosswalkMotor() {
  if (!crosswalkMode) {
    Serial.println("[PI MOTOR] ignored - not crosswalk mode");
    return false;
  }

  if (obstacleTriggered) {
    Serial.println("[PI MOTOR] ignored - obstacle priority");
    stopMotors();
    return false;
  }

  if (redLightStop) {
    Serial.println("[PI MOTOR] ignored - RED LIGHT");
    stopMotors();
    return false;
  }

  return true;
}

void crosswalkLeft() {
  if (!canUseCrosswalkMotor()) return;
  Serial.println("[PI MOTOR] CROSSWALK LEFT");
  pulseLeftGuide();
  sendBLE("CROSS_MOTOR:L");
}

void crosswalkRight() {
  if (!canUseCrosswalkMotor()) return;
  Serial.println("[PI MOTOR] CROSSWALK RIGHT");
  pulseRightGuide();
  sendBLE("CROSS_MOTOR:R");
}

void crosswalkForward() {
  if (!canUseCrosswalkMotor()) return;
  stopMotors();
  Serial.println("[PI MOTOR] CROSSWALK FORWARD / NEUTRAL");
  sendBLE("CROSS_MOTOR:F");
}

void crosswalkStop() {
  stopMotors();
  Serial.println("[PI MOTOR] CROSSWALK STOP");
  sendBLE("CROSS_MOTOR:STOP");
}

// =====================================================
// 역토크 3회
// =====================================================
void obstacleWarning() {
  Serial.println("*** OBSTACLE WARNING ***");

  digitalWrite(M1_DIR, HIGH);
  digitalWrite(M2_DIR, HIGH);

  for (int i = 0; i < BRAKE_COUNT; i++) {
    Serial.print("[BRAKE] ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.println(BRAKE_COUNT);

    ledcWrite(M1_PWM, BRAKE_PWM);
    ledcWrite(M2_PWM, BRAKE_PWM);
    delay(BRAKE_ON_TIME);

    stopMotors();
    delay(BRAKE_OFF_TIME);
  }

  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);
  stopMotors();

  Serial.println("*** BRAKE END ***");
}

// =====================================================
// TFmini
// =====================================================
int readTFmini() {
  static uint8_t buf[9];

  while (TFSerial.available() >= 9) {
    if (TFSerial.read() != 0x59) continue;
    if (TFSerial.read() != 0x59) continue;

    buf[0] = 0x59;
    buf[1] = 0x59;

    for (int i = 2; i < 9; i++) {
      unsigned long start = millis();
      while (!TFSerial.available()) {
        if (millis() - start > 10) return -1;
      }
      buf[i] = TFSerial.read();
    }

    uint16_t checksum = 0;
    for (int i = 0; i < 8; i++) checksum += buf[i];
    checksum &= 0xFF;

    if (checksum != buf[8]) {
      Serial.println("[TFmini] checksum error");
      return -1;
    }

    return buf[2] | (buf[3] << 8);
  }

  return -1;
}

void clearTFminiBuffer() {
  while (TFSerial.available()) TFSerial.read();
}

// =====================================================
// 서보 한 방향 평균 거리
// =====================================================
int measureDirection(int servoAngle) {
  tfServo.write(servoAngle);
  delay(SERVO_SETTLE_TIME);

  clearTFminiBuffer();

  long sum = 0;
  int validCount = 0;
  unsigned long startTime = millis();

  while (
    validCount < SCAN_SAMPLE_COUNT &&
    millis() - startTime < SCAN_TIMEOUT
  ) {
    int distance = readTFmini();

    if (distance > 0) {
      sum += distance;
      validCount++;

      Serial.print("[SCAN SAMPLE] ");
      Serial.print(validCount);
      Serial.print("/");
      Serial.print(SCAN_SAMPLE_COUNT);
      Serial.print(" = ");
      Serial.print(distance);
      Serial.println(" cm");

      delay(30);
    }
  }

  if (validCount == 0) return -1;
  return sum / validCount;
}

// =====================================================
// 장애물 회피 전용 모터
// 빨간불이면 절대 움직이지 않음
// =====================================================
void obstacleAvoidLeft() {
  if (redLightStop) {
    Serial.println("[AVOID] LEFT blocked - RED LIGHT");
    stopMotors();
    sendBLE("AVOID:STOP");
    return;
  }

  Serial.println("[AVOID MOTOR] LEFT");
  pulseLeftGuide();
  sendBLE("AVOID:LEFT");
}

void obstacleAvoidRight() {
  if (redLightStop) {
    Serial.println("[AVOID] RIGHT blocked - RED LIGHT");
    stopMotors();
    sendBLE("AVOID:STOP");
    return;
  }

  Serial.println("[AVOID MOTOR] RIGHT");
  pulseRightGuide();
  sendBLE("AVOID:RIGHT");
}

// =====================================================
// 장애물 좌/우 스캔
// =====================================================
void scanObstacleDirection() {
  Serial.println();
  Serial.println("==============================");
  Serial.println("OBSTACLE LEFT / RIGHT SCAN");
  Serial.println("==============================");

  stopMotors();

  Serial.println("[SCAN] LEFT START");
  int leftDistance = measureDirection(SERVO_LEFT);
  Serial.print("[SCAN] LEFT AVERAGE = ");
  if (leftDistance > 0) {
    Serial.print(leftDistance);
    Serial.println(" cm");
  } else {
    Serial.println("FAILED");
  }
  sendBLE("SCAN_LEFT:" + String(leftDistance));

  Serial.println("[SCAN] RIGHT START");
  int rightDistance = measureDirection(SERVO_RIGHT);
  Serial.print("[SCAN] RIGHT AVERAGE = ");
  if (rightDistance > 0) {
    Serial.print(rightDistance);
    Serial.println(" cm");
  } else {
    Serial.println("FAILED");
  }
  sendBLE("SCAN_RIGHT:" + String(rightDistance));

  // 정면 복귀
  tfServo.write(SERVO_CENTER);
  delay(SERVO_SETTLE_TIME);
  clearTFminiBuffer();

  // 빨간불이면 스캔만 하고 정지
  if (redLightStop) {
    Serial.println("[AVOID] RED LIGHT -> STOP");
    stopMotors();
    sendBLE("AVOID:STOP");
    return;
  }

  // 둘 다 실패
  if (leftDistance < 0 && rightDistance < 0) {
    Serial.println("[AVOID] BOTH SCAN FAILED");
    stopMotors();
    sendBLE("AVOID:STOP");
    return;
  }

  // 한쪽만 정상
  if (leftDistance > 0 && rightDistance < 0) {
    if (leftDistance >= SCAN_SAFE_DISTANCE) obstacleAvoidLeft();
    else {
      Serial.println("[AVOID] LEFT too narrow -> STOP");
      stopMotors();
      sendBLE("AVOID:STOP");
    }
    return;
  }

  if (rightDistance > 0 && leftDistance < 0) {
    if (rightDistance >= SCAN_SAFE_DISTANCE) obstacleAvoidRight();
    else {
      Serial.println("[AVOID] RIGHT too narrow -> STOP");
      stopMotors();
      sendBLE("AVOID:STOP");
    }
    return;
  }

  // 둘 다 좁음
  if (
    leftDistance < SCAN_SAFE_DISTANCE &&
    rightDistance < SCAN_SAFE_DISTANCE
  ) {
    Serial.println("[AVOID] BOTH BLOCKED");
    stopMotors();
    sendBLE("AVOID:STOP");
    return;
  }

  // 한쪽만 안전
  if (
    leftDistance >= SCAN_SAFE_DISTANCE &&
    rightDistance < SCAN_SAFE_DISTANCE
  ) {
    obstacleAvoidLeft();
    return;
  }

  if (
    rightDistance >= SCAN_SAFE_DISTANCE &&
    leftDistance < SCAN_SAFE_DISTANCE
  ) {
    obstacleAvoidRight();
    return;
  }

  // 둘 다 안전 -> 더 넓은 쪽 비교
  int difference = abs(leftDistance - rightDistance);

  Serial.print("[SCAN] DIFFERENCE = ");
  Serial.print(difference);
  Serial.println(" cm");

  if (difference < DIRECTION_MARGIN) {
    Serial.println("[AVOID] LEFT / RIGHT almost same -> STOP");
    stopMotors();
    sendBLE("AVOID:STOP");
    return;
  }

  if (leftDistance > rightDistance) obstacleAvoidLeft();
  else obstacleAvoidRight();
}

// =====================================================
// Raspberry Pi 메시지
//
// CROSSWALK:1 / CROSSWALK:0
// LIGHT:RED / LIGHT:GREEN / LIGHT:NONE
// CROSS_DIR:L / CROSS_DIR:R / CROSS_DIR:F / CROSS_DIR:S
// CROSSING_START / CROSSING_END
// =====================================================
void handlePiMessage(String message) {
  message.trim();
  if (message.length() == 0) return;

  Serial.print("[PI RX] ");
  Serial.println(message);

  // 횡단보도 감지 -> Pi가 제어권 가져감
  if (message == "CROSSWALK:1") {
    crosswalkMode = true;
    stopMotors();
    sendBLE("CROSSWALK:1");
    Serial.println("[MODE] Pi crosswalk control ON");
    return;
  }

  // 횡단보도 종료 -> Flutter 제어 복귀
  if (message == "CROSSWALK:0") {
    crosswalkMode = false;
    redLightStop = false;
    stopMotors();
    sendBLE("CROSSWALK:0");
    Serial.println("[MODE] Flutter navigation control restored");
    return;
  }

  // 빨간불 -> 무조건 정지
  if (
    message == "LIGHT:RED" ||
    message == "SIGNAL_RED" ||
    message == "WAIT_SIGNAL"
  ) {
    crosswalkMode = true;
    redLightStop = true;
    stopMotors();
    sendBLE("LIGHT:RED");
    Serial.println("[TRAFFIC] RED -> MOTOR STOP");
    return;
  }

  // 초록불 -> Pi 방향 명령 허용
  if (
    message == "LIGHT:GREEN" ||
    message == "SIGNAL_GREEN"
  ) {
    crosswalkMode = true;
    redLightStop = false;
    stopMotors();
    sendBLE("LIGHT:GREEN");
    Serial.println("[TRAFFIC] GREEN -> waiting CROSS_DIR");
    return;
  }

  // 미인식은 현재 red/green 상태를 유지하고 앱에만 전달
  if (message == "LIGHT:NONE") {
    sendBLE("LIGHT:NONE");
    return;
  }

  if (message == "CROSSING_START") {
    crosswalkMode = true;
    redLightStop = false;
    stopMotors();
    sendBLE("CROSSING_START");
    Serial.println("[MODE] CROSSING START");
    return;
  }

  if (message == "CROSSING_END") {
    crosswalkMode = false;
    redLightStop = false;
    stopMotors();
    sendBLE("CROSSING_END");
    Serial.println("[MODE] CROSSING END -> Flutter control restored");
    return;
  }

  // 횡단보도 방향 명령
  if (message == "CROSS_DIR:L") {
    crosswalkLeft();
    return;
  }

  if (message == "CROSS_DIR:R") {
    crosswalkRight();
    return;
  }

  if (message == "CROSS_DIR:F") {
    crosswalkForward();
    return;
  }

  if (message == "CROSS_DIR:S") {
    crosswalkStop();
    return;
  }

  // 그 외 Pi 메시지는 앱으로 전달
  sendBLE(message);
}

// =====================================================
// Raspberry Pi UART 읽기
// =====================================================
void readRaspberryPi() {
  while (PiSerial.available()) {
    char c = PiSerial.read();

    if (c == '\n') {
      handlePiMessage(piBuffer);
      piBuffer = "";
    }
    else if (c != '\r') {
      piBuffer += c;
      if (piBuffer.length() > 100) piBuffer = "";
    }
  }
}

// =====================================================
// Flutter BLE 명령
// B:90
// F:3.5
// L:45
// R:32
// S:0
// =====================================================
void handleFlutterCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  Serial.print("[BLE RX] ");
  Serial.println(command);

  // 목표 방위각은 로그용으로 계속 받음
  if (command.startsWith("B:")) {
    targetBearing = command.substring(2).toFloat();
    Serial.print("[NAV] Target bearing = ");
    Serial.println(targetBearing);
    return;
  }

  // STOP은 항상 허용
  if (command == "S" || command.startsWith("S:")) {
    stopMotors();
    sendBLE("MOTOR:STOP");
    return;
  }

  // 횡단보도 모드면 앱 방향 명령 차단
  if (crosswalkMode) {
    Serial.println("[BLE] Direction ignored - Pi has control");
    return;
  }

  if (command == "L" || command.startsWith("L:")) {
    float angle = 0;
    int colon = command.indexOf(':');
    if (colon >= 0) angle = command.substring(colon + 1).toFloat();
    guideLeft(angle);
    return;
  }

  if (command == "R" || command.startsWith("R:")) {
    float angle = 0;
    int colon = command.indexOf(':');
    if (colon >= 0) angle = command.substring(colon + 1).toFloat();
    guideRight(angle);
    return;
  }

  if (command == "F" || command.startsWith("F:")) {
    guideForward();
    return;
  }

  Serial.println("[BLE] Unknown command");
}

// =====================================================
// BLE Callback
// =====================================================
class RxCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String command = characteristic->getValue().c_str();
    handleFlutterCommand(command);
  }
};

class ServerCallback : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    Serial.println("BLE CONNECTED");
  }

  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    stopMotors();

    Serial.println("BLE DISCONNECTED");

    delay(200);
    server->getAdvertising()->start();

    Serial.println("BLE advertising restart");
  }
};

// =====================================================
// SETUP
// =====================================================
void setup() {

  // =============================================
  // 가장 먼저 모터 OFF
  // =============================================
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);

  digitalWrite(M1_DIR, LOW);
  digitalWrite(M2_DIR, LOW);

  digitalWrite(M1_PWM, LOW);
  digitalWrite(M2_PWM, LOW);

  // PWM 연결
  ledcAttach(M1_PWM, 5000, 8);
  ledcAttach(M2_PWM, 5000, 8);

  ledcWrite(M1_PWM, 0);
  ledcWrite(M2_PWM, 0);

  // 모터 OFF 후 Serial 시작
  Serial.begin(115200);

  Serial.println();
  Serial.println("==================================");
  Serial.println("SMART CANE START");
  Serial.println("==================================");

  Serial.println("[OK] MOTOR SAFE START");


  // =============================================
  // TFmini
  // =============================================
  TFSerial.begin(
    115200,
    SERIAL_8N1,
    TF_RX,
    TF_TX
  );

  Serial.println("[OK] TFmini UART2");


  // =============================================
  // Servo
  // =============================================
  tfServo.setPeriodHertz(50);
  tfServo.attach(SERVO_PIN, 500, 2400);
  tfServo.write(SERVO_CENTER);

  delay(700);

  Serial.println("[OK] TFmini Servo GPIO13");


  // =============================================
  // Raspberry Pi
  // =============================================
  PiSerial.begin(
    115200,
    SERIAL_8N1,
    PI_RX,
    PI_TX
  );

  Serial.println("[OK] Raspberry Pi UART1");


  // =============================================
  // BLE
  // =============================================
  BLEDevice::init("SMART_CANE");

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallback());

  BLEService* service =
      server->createService(SERVICE_UUID);

  BLECharacteristic* rxCharacteristic =
      service->createCharacteristic(
        RX_UUID,
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_WRITE_NR
      );

  rxCharacteristic->setCallbacks(new RxCallback());

  txCharacteristic =
      service->createCharacteristic(
        TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
      );

  txCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising* advertising =
      BLEDevice::getAdvertising();

  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);

  BLEDevice::startAdvertising();

  // 마지막으로 한번 더 정지
  stopMotors();

  Serial.println("[OK] BLE SMART_CANE");
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  // Pi 없어도 available()=0이라 그냥 넘어감
  readRaspberryPi();

  int distance = readTFmini();

  if (distance > 0) {
    static unsigned long lastDistancePrint = 0;
    static unsigned long lastDistanceSend = 0;

    if (millis() - lastDistancePrint >= 300) {
      Serial.print("[TFmini CENTER] ");
      Serial.print(distance);
      Serial.println(" cm");
      lastDistancePrint = millis();
    }

    if (millis() - lastDistanceSend >= 300) {
      sendBLE("DIST:" + String(distance));
      lastDistanceSend = millis();
    }

    // 30cm 이하 3회 연속
    if (distance <= OBSTACLE_DISTANCE) {
      obstacleCount++;

      Serial.print("[TFmini] obstacle count = ");
      Serial.println(obstacleCount);

      if (obstacleCount >= 3 && !obstacleTriggered) {
        obstacleTriggered = true;
        obstacleCount = 0;

        Serial.println();
        Serial.println("################################");
        Serial.println("OBSTACLE CONFIRMED");
        Serial.println("################################");

        sendBLE("OBSTACLE:" + String(distance));

        // 1. 정지
        stopMotors();

        // 2. 역토크 3회
        obstacleWarning();

        // 3. 서보 좌/우 스캔 + 방향 결정
        scanObstacleDirection();

        // 4. 정면 복귀 보장
        tfServo.write(SERVO_CENTER);
        delay(SERVO_SETTLE_TIME);
        clearTFminiBuffer();
      }
    }
    else {
      obstacleCount = 0;
    }

    // 40cm 이상 -> 새 장애물 감지 가능
    if (distance >= RESET_DISTANCE && obstacleTriggered) {
      obstacleTriggered = false;
      obstacleCount = 0;

      sendBLE("OBSTACLE_CLEAR");
      Serial.println("[TFmini] Obstacle cleared");
    }
  }

  delay(10);
}
