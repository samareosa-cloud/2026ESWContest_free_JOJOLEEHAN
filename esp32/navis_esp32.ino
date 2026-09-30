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

#define SERVICE_UUID \
"6e400001-b5a3-f393-e0a9-e50e24dcca9e"

#define RX_UUID \
"6e400002-b5a3-f393-e0a9-e50e24dcca9e"

#define TX_UUID \
"6e400003-b5a3-f393-e0a9-e50e24dcca9e"


BLECharacteristic* txCharacteristic = nullptr;

bool bleConnected = false;


// =====================================================
// 장애물 설정
//
// 앱:
// 100cm 이하 -> 화면에 "장애물 감지"
//
// ESP32:
// 30cm 이하 -> 역토크 3회
// 40cm 이상 -> 다시 장애물 감지 가능
// =====================================================

const int OBSTACLE_DISTANCE = 30;
const int RESET_DISTANCE = 40;

// 저번 장애물 회피 로직
// TFmini 30cm 이하가 3회 연속 들어와야 실제 장애물로 판단
const int OBSTACLE_CONFIRM_COUNT = 3;

// 좌/우 VL53L1X 판단 기준
const int SIDE_BLOCK_DISTANCE = 30;
const int DIRECTION_MARGIN = 20;

int obstacleCount = 0;
bool obstacleTriggered = false;


// =====================================================
// 방향 유도 모터 설정
// =====================================================

const int GUIDE_PWM = 55;
const int GUIDE_TIME = 250;


// =====================================================
// 역토크 설정
// =====================================================

const int BRAKE_PWM = 45;

const int BRAKE_ON_TIME = 300;
const int BRAKE_OFF_TIME = 200;

const int BRAKE_COUNT = 3;


// =====================================================
// 횡단보도 모드
//
// true:
// 앱에서 오는 일반 L/R 유도 명령 무시
// =====================================================

bool crosswalkMode = false;
bool trafficRed = false;


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
  ledcWrite(
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
  ledcWrite(
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
// 횡단보도 LEFT 보정
// Raspberry Pi -> CROSS_MOTOR:L
// =====================================================
void crosswalkLeft() {

  if (
    !crosswalkMode ||
    trafficRed ||
    obstacleTriggered
  ) {
    stopMotors();
    return;
  }

  Serial.println("[CROSS MOTOR] LEFT");

  // M1 정지
  ledcWrite(M1_PWM, 0);

  // M2를 이용해 왼쪽 유도
  digitalWrite(M2_DIR, LOW);
  ledcWrite(M2_PWM, GUIDE_PWM);

  delay(GUIDE_TIME);

  ledcWrite(M2_PWM, 0);

  sendBLE("CROSS_MOTOR:L");
}


// =====================================================
// 횡단보도 RIGHT 보정
// Raspberry Pi -> CROSS_MOTOR:R
// =====================================================
void crosswalkRight() {

  if (
    !crosswalkMode ||
    trafficRed ||
    obstacleTriggered
  ) {
    stopMotors();
    return;
  }

  Serial.println("[CROSS MOTOR] RIGHT");

  // M2 정지
  ledcWrite(M2_PWM, 0);

  // M1을 이용해 오른쪽 유도
  digitalWrite(M1_DIR, LOW);
  ledcWrite(M1_PWM, GUIDE_PWM);

  delay(GUIDE_TIME);

  ledcWrite(M1_PWM, 0);

  sendBLE("CROSS_MOTOR:R");
}


// =====================================================
// 횡단보도 중앙 유지
// Raspberry Pi -> CROSS_MOTOR:CENTER
// =====================================================
void crosswalkCenter() {

  stopMotors();

  Serial.println("[CROSS MOTOR] CENTER");

  sendBLE("CROSS_MOTOR:CENTER");
}

// =====================================================
// 장애물 역토크 3회
// =====================================================

void obstacleWarning() {

  Serial.println();
  Serial.println(
    "*** OBSTACLE WARNING ***"
  );


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
    i < BRAKE_COUNT;
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
      BRAKE_COUNT
    );


    // 역토크 ON
    ledcWrite(
      M1_PWM,
      BRAKE_PWM
    );

    ledcWrite(
      M2_PWM,
      BRAKE_PWM
    );


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
// 장애물 회피용 LEFT 유도
//
// 기존 guideLeft()는 obstacleTriggered=true일 때 무시하므로,
// 장애물 회피 전용 함수는 별도로 둔다.
// 오른쪽 바퀴 M2를 짧게 구동해서 왼쪽으로 유도.
// =====================================================

void avoidLeft() {

  Serial.println("[AVOID] TURN LEFT");

  ledcWrite(M1_PWM, 0);

  digitalWrite(M2_DIR, LOW);
  ledcWrite(M2_PWM, GUIDE_PWM);

  delay(GUIDE_TIME);

  ledcWrite(M2_PWM, 0);

  sendBLE("MOTOR:L");
}


// =====================================================
// 장애물 회피용 RIGHT 유도
//
// 왼쪽 바퀴 M1을 짧게 구동해서 오른쪽으로 유도.
// =====================================================

void avoidRight() {

  Serial.println("[AVOID] TURN RIGHT");

  ledcWrite(M2_PWM, 0);

  digitalWrite(M1_DIR, LOW);
  ledcWrite(M1_PWM, GUIDE_PWM);

  delay(GUIDE_TIME);

  ledcWrite(M1_PWM, 0);

  sendBLE("MOTOR:R");
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
// 장애물 회피 방향 결정
//
// 1) 양쪽 모두 30cm 미만 -> 회피 공간 없음 -> 역토크 경고
// 2) LEFT가 RIGHT보다 20cm 초과 더 멂 -> LEFT 유도
// 3) RIGHT가 LEFT보다 20cm 초과 더 멂 -> RIGHT 유도
// 4) 차이가 20cm 이하 -> 별도 좌/우 유도 없이 정지
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


  // 센서 하나라도 읽기 실패하면 방향 유도하지 않고 정지
  // 역토크는 위에서 이미 실행했으므로 다시 실행하지 않음.
  if (
    leftDistance < 0
    ||
    rightDistance < 0
  ) {

    Serial.println("[AVOID] VL53 read error -> STOP");
    stopMotors();
    return;
  }


  // 양쪽 모두 막혀 있으면 정지
  // 역토크는 위에서 이미 실행됨.
  if (
    leftDistance < SIDE_BLOCK_DISTANCE
    &&
    rightDistance < SIDE_BLOCK_DISTANCE
  ) {

    Serial.println("[AVOID] BOTH SIDES BLOCKED -> STOP");
    stopMotors();
    return;
  }


  // 왼쪽이 확실히 더 넓음
  if (
    leftDistance >
        rightDistance + DIRECTION_MARGIN
  ) {

    avoidLeft();
    return;
  }


  // 오른쪽이 확실히 더 넓음
  if (
    rightDistance >
        leftDistance + DIRECTION_MARGIN
  ) {

    avoidRight();
    return;
  }


  // 좌우 차이가 기준 이하이면 확실한 회피 방향이 아니므로 중립
  stopMotors();

  Serial.println(
    "[AVOID] LEFT/RIGHT difference <= 20 cm -> NEUTRAL"
  );
}


// =====================================================
// TFmini 읽기
// =====================================================

int readTFmini() {

  static uint8_t buf[9];


  while (
    TFSerial.available() >= 9
  ) {

    // 첫 번째 헤더
    if (
      TFSerial.read() != 0x59
    ) {
      continue;
    }


    // 두 번째 헤더
    if (
      TFSerial.read() != 0x59
    ) {
      continue;
    }


    buf[0] = 0x59;
    buf[1] = 0x59;


    for (
      int i = 2;
      i < 9;
      i++
    ) {

      unsigned long start =
          millis();


      while (
        !TFSerial.available()
      ) {

        if (
          millis() - start > 10
        ) {

          return -1;
        }
      }


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

      return -1;
    }


    // 거리 cm
    uint16_t distance =
        buf[2] |
        (buf[3] << 8);


    return distance;
  }


  return -1;
}


// =====================================================
// Raspberry Pi 메시지 처리
//
// Raspberry Pi에서 아래처럼 보내면 됨.
//
// CROSSWALK:1
// CROSSWALK:0
//
// LIGHT:RED
// LIGHT:GREEN
// LIGHT:NONE
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

    stopMotors();


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

    stopMotors();


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

    crosswalkMode = true;
    trafficRed = true;

    stopMotors();

    sendBLE(
      "LIGHT:RED"
    );

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

    stopMotors();

    sendBLE(
      "LIGHT:GREEN"
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

    stopMotors();

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

    stopMotors();

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
    " <= 30 cm x3 : voice alert + reverse torque x3 + compare LEFT/RIGHT"
  );

  Serial.println(
    " both side < 30 cm : stop after reverse torque"
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
    // TFmini 30cm 이하가 3회 연속 들어오면
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
    // 정면이 40cm 이상으로 다시 멀어지면 reset
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
