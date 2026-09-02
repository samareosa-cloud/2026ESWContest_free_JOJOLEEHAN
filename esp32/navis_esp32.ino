#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


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

bool obstacleTriggered = false;


// =====================================================
// 방향 유도 모터 설정
// =====================================================

const int GUIDE_PWM = 55;
const int GUIDE_TIME = 250;


// =====================================================
// 역토크 설정
// =====================================================

const int BRAKE_PWM = 33;

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
    " <= 30 cm  : reverse torque x3"
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
    // 30cm 이하
    //
    // 역토크는 한 장애물당 한 번만
    // -------------------------------------------------

    if (
      distance <=
          OBSTACLE_DISTANCE
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


      stopMotors();


      obstacleWarning();
    }


    // -------------------------------------------------
    // 40cm 이상으로 다시 멀어지면 reset
    // -------------------------------------------------

    if (
      distance >=
          RESET_DISTANCE
      &&
      obstacleTriggered
    ) {

      obstacleTriggered =
          false;


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
