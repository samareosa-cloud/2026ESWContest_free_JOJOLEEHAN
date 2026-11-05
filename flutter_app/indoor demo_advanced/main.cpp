import 'dart:async';
import 'dart:convert';
import 'dart:io' show Platform;
import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/semantics.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';
import 'package:flutter_compass/flutter_compass.dart';
import 'package:flutter_map/flutter_map.dart';
import 'package:flutter_tts/flutter_tts.dart';
import 'package:geolocator/geolocator.dart';
import 'package:http/http.dart' as http;
import 'package:latlong2/latlong.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:speech_to_text/speech_to_text.dart' as stt;

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const SmartCaneApp());
}

class SmartCaneApp extends StatelessWidget {
  const SmartCaneApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Smart Cane',
      theme: ThemeData(
        useMaterial3: true,
        colorSchemeSeed: Colors.deepPurple,
        scaffoldBackgroundColor: const Color(0xFFF7F3F8),
      ),
      home: const SmartCaneHomePage(),
    );
  }
}

class SmartCaneHomePage extends StatefulWidget {
  const SmartCaneHomePage({super.key});

  @override
  State<SmartCaneHomePage> createState() => _SmartCaneHomePageState();
}

class _SmartCaneHomePageState extends State<SmartCaneHomePage> {
  // ===========================================================================
  // TMAP
  // 실행:
  // flutter run --dart-define=TMAP_APP_KEY=실제_앱키
  // ===========================================================================
  static const String tmapAppKey = String.fromEnvironment(
    'TMAP_APP_KEY',
    defaultValue: '',
  );

  static const String tmapHost = 'apis.openapi.sk.com';
  static const String tmapPoiPath = '/tmap/pois';
  static const String tmapPedestrianPath = '/tmap/routes/pedestrian';

  // ===========================================================================
  // ESP32 BLE - Nordic UART Service
  // ===========================================================================
  static const String targetDeviceName = 'SMART_CANE';

  static const String serviceUuid =
      '6e400001-b5a3-f393-e0a9-e50e24dcca9e';

  // Flutter -> ESP32
  static const String rxUuid =
      '6e400002-b5a3-f393-e0a9-e50e24dcca9e';

  // ESP32 -> Flutter
  static const String txUuid =
      '6e400003-b5a3-f393-e0a9-e50e24dcca9e';

  // TFmini: 40cm 이내일 때 "장애물 감지"를 화면에 표시.
  // 음성은 ESP32가 OBSTACLE_BRAKE를 보낸 실제 역토크 시작 시점에만 출력.
  static const double obstacleWarningCm = 40.0;

  // ===========================================================================
  // 실제 도로 주행용 경로 허용값
  // ===========================================================================
  // TMAP 경로선(지도 위 초록색 선)에서 좌우 이 거리까지는 경로 위로 간주한다.
  // 이 폭 안에서는 선 위로 끌어당기지 않고 경로와 나란히 걷도록만 안내한다.
  // (인도 폭 + 스마트폰 GPS 오차 3~5 m 감안)
  static const double routeToleranceMeters = 5.0;

  // GPS 정확도(accuracy)가 나쁘면 그 오차만큼 허용 폭을 넓힌다. 최대 이 값까지.
  static const double maxRouteToleranceMeters = 10.0;

  // 경로 진행 방향은 경로를 따라 이 거리만큼 앞 지점을 기준으로 계산한다.
  // 허용 폭을 벗어났을 때도 이 거리만큼 앞에서 완만하게 경로에 합류시킨다.
  static const double routeLookAheadMeters = 8.0;

  // ===========================================================================
  // 나침반 민감도 (직진 / 왼쪽 / 오른쪽 판단)
  // ===========================================================================
  // 목표 방위각과 나침반 방위각 오차가 ±이 값을 넘어야 좌/우 유도를 시작한다.
  static const double headingToleranceDegrees = 25.0;

  // 좌/우 유도 중에는 오차가 ±이 값 안으로 들어와야 다시 직진으로 바뀐다.
  // (경계값 근처에서 직진 ↔ 좌/우가 계속 바뀌지 않도록 하는 히스테리시스)
  static const double headingReleaseDegrees = 15.0;

  // 나침반 값 평활화 시간(초). 클수록 흔들림에 둔감하지만 반응은 느려진다.
  static const double compassSmoothingSeconds = 0.7;

  // 좌/우 판단이 이 시간 이상 계속 유지되어야 실제 모터 명령으로 보낸다.
  static const Duration steeringHoldDuration = Duration(milliseconds: 700);

  // 같은 방향 명령(F/L/R)을 다시 보내는 최소 간격
  static const Duration motorRepeatInterval = Duration(milliseconds: 1500);

  // 자기 편각 보정. Android 나침반은 자북 기준이고 TMAP 경로는 진북 기준이라
  // 한국에서는 약 9° 차이가 난다. (WMM-2025: 서울 -9.0°, 부산 -8.5°, 제주 -7.8°)
  // iOS(flutter_compass)는 이미 진북(trueHeading)을 주므로 Android에서만 적용한다.
  static const double magneticDeclinationDegrees = -9.0;

  // 장애물을 피한 직후에는 잠깐 방향 유도를 쉬어서
  // 바로 장애물 쪽으로 다시 끌어당기지 않게 한다.
  static const Duration obstacleClearPause = Duration(milliseconds: 2500);

  // 횡단을 마친 직후에도 잠깐 방향 유도를 쉰다.
  // (Pi가 횡단 종료를 일찍 판정했을 때 도로 위에서 바로 끌어당기지 않게)
  static const Duration crossingClearPause = Duration(seconds: 3);

  // ESP32 CROSSING_RED_GRACE_MS와 같은 값.
  // CROSSING_END 뒤 이 시간 안에는 횡단보도 재인식 / GREEN_WAIT 안내를 하지 않는다.
  // 빨간불은 앱이 따로 판단하지 않고 ESP32를 따른다.
  // (ESP32가 유예 중이면 CROSSING_RED, 유예가 끝나면 LIGHT:RED를 보냄)
  static const Duration crossingGrace = Duration(seconds: 3);

  // ===========================================================================
  // ㄱ자 실내 시연 모드
  // ===========================================================================
  // 회전 단계: 새 방향과의 오차가 이 값 안으로 이 시간 동안 유지되면 다음 단계
  static const double demoTurnDoneDegrees = 15.0;
  static const Duration demoTurnHoldDuration = Duration(milliseconds: 2500);

  // 장애물 구간: 장애물을 만난 뒤 풀리고 이 시간이 지나면 도착
  static const Duration demoArriveAfterObstacle = Duration(seconds: 3);

  // ===========================================================================
  // 음성 안내 시점
  // ===========================================================================
  // 남은 거리가 이 값을 처음 지날 때 한 번씩 알려 준다. (m)
  static const List<double> progressMilestonesMeters = [
    1000,
    500,
    300,
    200,
    100,
    50,
  ];

  // 경로 허용 폭을 이 시간 이상 계속 벗어나야 "경로 이탈" 안내
  // (GPS가 잠깐 튀는 것으로 안내하지 않게)
  static const Duration offRouteAnnounceDelay = Duration(seconds: 8);
  static const Duration offRouteRepeatInterval = Duration(seconds: 30);

  // 지팡이 연결이 끊긴 채 길 안내 중이면 이 간격으로 다시 알려 준다.
  static const Duration disconnectReminderInterval = Duration(seconds: 30);
  static const Duration reconnectInterval = Duration(seconds: 5);

  // 방향 유도에 쓰는 GPS / 나침반 값의 유효 조건.
  // 하나라도 벗어나면 L/R 유도를 멈추고(F) 사용자에게 알린다.
  static const Duration gpsStaleAfter = Duration(seconds: 5);
  static const double gpsMaxAccuracyMeters = 20.0;
  static const Duration compassStaleAfter = Duration(milliseconds: 1500);

  // 경로 진행 위치 검색 범위 (지금 진행 위치 기준, 경로를 따라 잰 거리)
  static const double progressSearchBackMeters = 30.0;
  static const double progressSearchAheadMeters = 60.0;
  // 진행 위치가 실제 이동 거리보다 이만큼 더 크게 바뀌면 GPS 튐으로 본다.
  static const double progressJumpMarginMeters = 10.0;
  // 튐으로 본 위치가 이 횟수만큼 연속으로 같은 곳을 가리키면 받아들인다.
  static const int progressJumpConfirmCount = 3;

  // 도착 판정: 직선거리 + 경로상 남은 거리 + GPS 정확도를 모두 만족하는 상태가
  // arrivalConfirmDuration 동안 이어져야 도착으로 본다.
  static const double arrivalDistanceMeters = 10.0;
  static const double arrivalRouteRemainingMeters = 15.0;
  static const double arrivalMaxAccuracyMeters = 15.0;
  static const Duration arrivalConfirmDuration = Duration(seconds: 2);

  // BLE 전송 실패 후 같은 명령을 다시 보내기까지 최소 간격 / 정지 명령 재시도
  static const Duration motorRetryInterval = Duration(milliseconds: 300);
  static const int stopCommandRetries = 3;

  // 사용자 화면 제스처
  // 마지막 탭 뒤 이 시간 안에 다시 누르면 같은 동작의 탭 횟수로 센다.
  static const Duration multiTapWindow = Duration(milliseconds: 300);
  // 주머니 속 오작동을 줄이려고 기본(0.5초)보다 길게
  static const Duration longPressDuration = Duration(milliseconds: 800);

  static const String gestureGuideText =
      '화면 아무 곳이나 한 번 누르면 지금 상황을 알려 드립니다. '
      '길게 누르면 말하기입니다. 목적지나 명령을 말씀하세요. '
      '두 번 누르면 길 안내를 멈춥니다.';

  // ===========================================================================
  // 객체 / 스트림
  // ===========================================================================
  final MapController _mapController = MapController();
  final FlutterTts _tts = FlutterTts();
  final stt.SpeechToText _speech = stt.SpeechToText();

  StreamSubscription<Position>? _positionSubscription;
  StreamSubscription<CompassEvent>? _compassSubscription;
  StreamSubscription<BluetoothConnectionState>? _bleConnectionSubscription;
  StreamSubscription<List<int>>? _bleNotifySubscription;

  Timer? _guidanceTimer;
  Timer? _listenRetryTimer;
  Timer? _reconnectTimer;

  // 길 안내 중 GPS / 나침반이 끊겼는지 1초마다 확인 (나침반 이벤트가 멈추면
  // _updateSteering()도 불리지 않으므로 따로 확인한다)
  Timer? _navInputWatchdog;

  // 횡단 종료 직후 다시 보인 횡단보도 안내를 유예가 끝날 때까지 미룬다.
  Timer? _deferredCrosswalkTimer;

  // ===========================================================================
  // 음성 인식 / TTS
  // ===========================================================================
  bool _speechAvailable = false;
  bool _isListening = false;
  bool _isSpeaking = false;
  bool _isHandlingVoiceCommand = false;
  bool _gotFinalSpeechResult = false;
  bool _isProcessingSpeechEndResult = false;

  String _lastRecognizedWords = '';
  String _speechStatus = '음성 시스템 준비 중...';
  String _recognizedDestinationText = '';

  // 음성 안내 큐 (_announce 참고)
  // 급한 안내(장애물/빨간불/연결 끊김)는 맨 앞으로 들어가고,
  // 덜 급한 안내를 말하는 중이면 끊고 먼저 말한다.
  final List<_SpeechRequest> _speechQueue = [];
  _SpeechRequest? _currentSpeech;
  _SpeechRequest? _interruptedSpeech;
  Completer<void>? _speechInterrupt;
  bool _isSpeechLoopRunning = false;

  // 안내 때문에 목적지 듣기를 끊었으면 안내 후 다시 듣는다.
  bool _resumeListeningAfterSpeech = false;

  // ===========================================================================
  // 지도 / 경로 / 내비게이션
  // ===========================================================================
  LatLng? _currentLatLng;
  Destination? _destination;
  List<RoutePoint> _routePoints = [];

  // _routeCumulativeMeters[i]: 경로 시작점부터 i번째 점까지의 경로 길이
  List<double> _routeCumulativeMeters = [];
  double? _remainingRouteMeters;

  // 경로 진행 위치 (경로 시작점부터 경로를 따라 잰 거리)와 그때의 GPS 위치.
  // GPS가 튀어서 진행 위치가 멀리 뛰면 바로 받아들이지 않는다.
  double _routeProgressMeters = 0.0;
  Position? _lastProgressPosition;
  double? _progressJumpCandidateMeters;
  int _progressJumpCount = 0;

  // 도착 조건을 처음 만족한 시각
  DateTime? _arrivalCandidateSince;

  // 방향 유도에 쓸 수 있는 마지막 GPS 위치 시각 / 정확도
  DateTime? _lastGoodFixAt;
  double? _lastFixAccuracy;

  // GPS / 나침반 문제로 방향 유도를 멈춘 이유 ('gps' / 'compass' / null)
  String? _navInputProblem;

  // 남은 거리 안내 / 경로 이탈 안내 상태
  double? _lastProgressMilestone;
  DateTime? _offRouteSince;
  DateTime? _lastOffRouteAnnouncedAt;
  bool _offRouteAnnounced = false;

  bool _isSearchingRoute = false;
  bool _isNavigationActive = false;
  bool _isGuidanceUpdating = false;

  double? _phoneHeading;
  double? _targetBearing;
  double? _steeringError;
  double? _distanceToDestinationMeters;
  double? _distanceToNextPointMeters;

  // 경로선과의 거리 / 지금 적용 중인 허용 폭 (화면 표시용)
  double? _routeDeviationMeters;
  double? _currentRouteToleranceMeters;

  // 나침반 평활화 상태. 359° → 0° 경계에서 튀지 않도록 sin/cos로 평균한다.
  double _headingSin = 0.0;
  double _headingCos = 1.0;
  DateTime? _lastCompassEventAt;

  List<LatLng> get _routeLatLngs => _routePoints
      .map((point) => LatLng(point.latitude, point.longitude))
      .toList();

  // ===========================================================================
  // BLE
  // ===========================================================================
  BluetoothDevice? _device;
  BluetoothCharacteristic? _writeCharacteristic;
  BluetoothCharacteristic? _notifyCharacteristic;

  bool _isBleConnected = false;
  bool _isConnecting = false;
  bool _writeWithoutResponse = false;
  String _bleStatus = '연결 대기';

  // 한 번이라도 연결됐다가 끊겼는지 (다시 연결되면 알려 주기 위해)
  bool _bleWasLost = false;

  // 연결 직후 요청한 ESP32 상태(ST:) 응답을 받았는지
  bool _caneStateReceived = false;
  DateTime? _lastDisconnectReminderAt;

  int? _lastSentBearing;
  DateTime? _lastBearingSentAt;

  // 마지막으로 "전송에 성공한" 모터 명령과 시각
  String? _lastMotorCommand;
  DateTime? _lastMotorSentAt;
  DateTime? _lastMotorFailedAt;
  bool _motorSendInFlight = false;

  // 지금 실제로 유도 중인 방향(F/L/R)과, 유지 시간을 기다리는 좌/우 판단
  String _steeringCommand = 'F';
  String? _pendingSteeringCommand;
  DateTime? _pendingSteeringSince;

  bool _isCrosswalkMode = false;

  // ESP32 장애물 처리 상태
  bool _obstacleConfirmed = false;

  // ESP32가 알려준 회피 방향: LEFT / RIGHT / BLOCKED / NONE
  String _avoidDirection = 'NONE';
  DateTime? _obstacleClearedAt;

  // ===========================================================================
  // 거리 센서 / Raspberry Pi 인식 결과
  // ===========================================================================
  // 정면: TFmini
  double? _distanceCm;

  // 좌/우: VL53L1X
  double? _leftDistanceCm;
  double? _rightDistanceCm;

  // ESP32가 SENSOR:FRONT_LOST를 보냄 (TFmini 프레임이 안 옴)
  bool _frontSensorLost = false;

  bool _crosswalkDetected = false;
  String _trafficLight = 'NONE'; // RED / GREEN / GREEN_WAIT / CROSSING_RED / NONE

  // CROSSING_START ~ CROSSING_END: 실제로 도로 위를 건너는 중
  bool _isCrossing = false;
  bool _crossingRedAnnounced = false;
  DateTime? _crossingEndedAt;

  // 횡단보도 주행 중 Raspberry Pi → ESP32 모터 보정 상태
  // LEFT / RIGHT / CENTER / NONE
  String _crosswalkSteering = 'NONE';

  // 화면 종류. 기능은 같고 보여 주는 방식만 다르다.
  //  false: 사용자(시각장애인) 화면 - 버튼 없이 화면 전체 제스처, 고대비
  //  true : 보호자 화면 - 지도, 경로, 센서 값, 인식 결과 등 자세한 정보
  bool _isGuardianView = false;

  // 사용자 화면 제스처: 탭 횟수 세기 / 방금 한 동작을 화면에 잠깐 표시
  int _tapCount = 0;
  Timer? _tapTimer;
  String? _gestureFeedback;
  Timer? _gestureFeedbackTimer;

  // ===========================================================================
  // ㄱ자 실내 시연 모드
  // 단계 0: 대기, 1: 직진(횡단보도), 2: 회전, 3: 직진(장애물), 4: 도착
  // ===========================================================================
  bool _isDemoMode = false;
  bool _demoTurnRight = true;
  int _demoStep = 0;
  double? _demoFrontBearing;
  DateTime? _demoTurnHeldSince;
  bool _demoObstacleSeen = false;
  Timer? _demoTimer;

  @override
  void initState() {
    super.initState();

    _startCompass();

    // 앱 실행 직후 SMART_CANE BLE 자동 연결 시도
    // 실패하면 한 번 알려 주고, 이후에는 조용히 계속 다시 시도한다.
    Future<void>.delayed(const Duration(milliseconds: 500), () async {
      if (mounted && !_isBleConnected && !_isConnecting) {
        await _connectSmartCane(silent: true, announceFailure: true);

        if (mounted && !_isBleConnected) {
          _scheduleReconnect();
        }
      }
    });

    // BLE 연결 시도와 별개로 음성 시스템 초기화
    Future<void>.delayed(const Duration(milliseconds: 800), () {
      if (mounted) {
        _initializeVoiceSystem();
      }
    });
  }

  @override
  void dispose() {
    _positionSubscription?.cancel();
    _compassSubscription?.cancel();
    _bleConnectionSubscription?.cancel();
    _bleNotifySubscription?.cancel();

    _guidanceTimer?.cancel();
    _listenRetryTimer?.cancel();
    _reconnectTimer?.cancel();
    _navInputWatchdog?.cancel();
    _deferredCrosswalkTimer?.cancel();
    _tapTimer?.cancel();
    _gestureFeedbackTimer?.cancel();
    _demoTimer?.cancel();

    _speech.cancel();
    _tts.stop();
    _device?.disconnect();

    super.dispose();
  }

  // ===========================================================================
  // TTS / 음성 인식
  // ===========================================================================

  Future<void> _initializeVoiceSystem() async {
    try {
      await _setupTts();
      await _setupSpeechToText();

      if (tmapAppKey.isEmpty) {
        _setSpeechStatus('TMAP 앱키가 필요합니다.');
        await _speak(
          'TMAP 앱키가 설정되지 않았습니다. 앱키를 넣고 다시 실행해 주세요.',
        );
        return;
      }

      // 앱 실행 후 사용법을 한 번 알려 주고, 바로 목적지를 묻고 듣는다.
      await _speakAndListen(
        '한 번 누르면 상황 듣기, 길게 누르면 말하기, 두 번 누르면 안내 중지입니다. '
            '목적지를 말씀해주세요.',
      );
    } catch (e) {
      _setSpeechStatus('음성 시스템 초기화 실패');
    }
  }

  Future<void> _setupTts() async {
    await _tts.setLanguage('ko-KR');
    await _tts.setSpeechRate(0.45);
    await _tts.setPitch(1.0);
    await _tts.setVolume(1.0);
    await _tts.awaitSpeakCompletion(true);

    if (Platform.isAndroid) {
      try {
        await _tts.setAudioAttributesForNavigation();
      } catch (_) {}
    }

    // iOS 기본 설정(ambient)은 무음 스위치를 켜 두면 안내가 아예 안 나온다.
    // 시각장애인은 화면을 볼 수 없으므로 무음 모드에서도 반드시 들려야 한다.
    // 음성 인식(마이크)과 같이 쓰므로 playAndRecord, 다른 소리는 잠깐 줄인다.
    if (Platform.isIOS) {
      try {
        await _tts.setSharedInstance(true);
        await _tts.setIosAudioCategory(
          IosTextToSpeechAudioCategory.playAndRecord,
          [
            IosTextToSpeechAudioCategoryOptions.defaultToSpeaker,
            IosTextToSpeechAudioCategoryOptions.allowBluetooth,
            IosTextToSpeechAudioCategoryOptions.allowBluetoothA2DP,
            IosTextToSpeechAudioCategoryOptions.duckOthers,
          ],
          IosTextToSpeechAudioMode.voicePrompt,
        );
      } catch (_) {}
    }
  }

  Future<void> _setupSpeechToText() async {
    _speechAvailable = await _speech.initialize(
      onStatus: (String status) {
        if (!mounted) return;

        if (status == 'listening') {
          setState(() {
            _isListening = true;
            _speechStatus = '듣는 중...';
          });
          return;
        }

        if (status == 'done' || status == 'notListening') {
          if (mounted) {
            setState(() {
              _isListening = false;
            });
          }

          // 일부 기기는 finalResult 없이 partial만 남기고 done이 온다.
          // 짧게 기다렸다가 남아 있는 단어가 있으면 그 단어를 처리하고,
          // 아무 단어도 없으면 자동으로 다시 묻는다.
          _listenRetryTimer?.cancel();
          _listenRetryTimer = Timer(
            const Duration(milliseconds: 350),
                () {
              if (!mounted ||
                  _gotFinalSpeechResult ||
                  _isHandlingVoiceCommand ||
                  _isSearchingRoute ||
                  _isNavigationActive ||
                  _isSpeaking) {
                return;
              }

              final String partial = _lastRecognizedWords.trim();

              if (partial.isNotEmpty && !_isProcessingSpeechEndResult) {
                _isProcessingSpeechEndResult = true;
                unawaited(_processSpeechEndResult(partial));
              } else if (partial.isEmpty) {
                unawaited(
                  _retryDestinationInput(
                    '잘 듣지 못했습니다. 목적지를 다시 말씀해 주세요.',
                  ),
                );
              }
            },
          );
        }
      },
      onError: (error) {
        if (!mounted) return;

        setState(() {
          _isListening = false;
          _speechStatus = '음성 인식 실패';
        });

        if (!_isNavigationActive &&
            !_isSearchingRoute &&
            !_isHandlingVoiceCommand) {
          unawaited(
            _retryDestinationInput(
              '잘 듣지 못했습니다. 목적지를 다시 말씀해 주세요.',
            ),
          );
        }
      },
    );

    if (!mounted) return;

    setState(() {
      _speechStatus = _speechAvailable
          ? '목적지를 말씀해주세요.'
          : '이 기기에서 음성 인식을 사용할 수 없습니다.';
    });
  }

  // ===========================================================================
  // 음성 안내 큐
  //
  // 모든 음성은 _announce()를 거친다. (대화형 질문은 _speak())
  //
  // urgent   : 장애물, 빨간불 정지, 지팡이 연결 끊김, 횡단 중 신호 변경
  //            -> 큐 맨 앞. 덜 급한 안내를 말하는 중이면 끊고 바로 말한다.
  // important: 목적지 질문/응답, 횡단보도·초록불, 경로 이탈
  //            -> urgent 다음. urgent에 끊기면 urgent 뒤에 다시 말한다.
  // info     : 남은 거리, 연결됨 같은 참고 안내
  //
  // key  : 같은 key의 안내가 대기 중이면 최신 것으로 바꾼다.
  //        (예: "왼쪽으로 피하세요"가 밀려 있는데 오른쪽으로 바뀐 경우)
  // maxAge: 이 시간 안에 말하지 못하면 버린다. 상황이 이미 바뀌었을 수 있다.
  // ===========================================================================

  Future<void> _announce(
      String text, {
        _SpeechPriority priority = _SpeechPriority.info,
        String? key,
        Duration? maxAge,
        bool vibrate = false,
      }) {
    final String trimmed = text.trim();

    if (trimmed.isEmpty || !mounted) return Future<void>.value();

    if (vibrate) {
      unawaited(HapticFeedback.vibrate());
    }

    if (key != null) {
      _dropQueuedSpeech((r) => r.key == key);

      // 같은 안내를 지금 말하고 있으면 다시 넣지 않는다.
      final _SpeechRequest? current = _currentSpeech;
      if (current != null && current.key == key && current.text == trimmed) {
        return current.done.future;
      }
    }

    final _SpeechRequest request = _SpeechRequest(
      text: trimmed,
      priority: priority,
      key: key,
      maxAge: maxAge,
    );

    _insertSpeech(request);

    // 급한 안내는 덜 급한 안내를 끊는다.
    final _SpeechRequest? current = _currentSpeech;
    if (priority == _SpeechPriority.urgent &&
        current != null &&
        current.priority != _SpeechPriority.urgent) {
      _interruptedSpeech = current;

      final Completer<void>? interrupt = _speechInterrupt;
      if (interrupt != null && !interrupt.isCompleted) {
        interrupt.complete();
      }

      unawaited(_tts.stop());
    }

    unawaited(_runSpeechLoop());

    return request.done.future;
  }

  // 우선순위가 높은 것부터, 같은 우선순위는 들어온 순서대로
  void _insertSpeech(_SpeechRequest request) {
    int index = _speechQueue.indexWhere(
          (r) => r.priority.index < request.priority.index,
    );

    if (index < 0) index = _speechQueue.length;

    _speechQueue.insert(index, request);
  }

  void _dropQueuedSpeech(bool Function(_SpeechRequest request) test) {
    _speechQueue.removeWhere((r) {
      if (!test(r)) return false;
      r.finish();
      return true;
    });
  }

  Future<void> _runSpeechLoop() async {
    if (_isSpeechLoopRunning) return;

    _isSpeechLoopRunning = true;

    try {
      while (mounted && _speechQueue.isNotEmpty) {
        // 사용자가 말하는 중(음성 인식 중)에는 급하지 않은 안내로 끊지 않는다.
        // 듣기가 끝나면 이어서 말한다. 급한 안내는 바로 끊고 말한다.
        if (_speechQueue.first.priority != _SpeechPriority.urgent &&
            _speech.isListening) {
          await Future<void>.delayed(const Duration(milliseconds: 250));
          _dropQueuedSpeech((r) => r.isStale);
          continue;
        }

        final _SpeechRequest request = _speechQueue.removeAt(0);

        if (request.isStale) {
          debugPrint('[TTS] skip stale: ${request.text}');
          request.finish();
          continue;
        }

        await _stopListeningForSpeech();

        _currentSpeech = request;
        _setSpeaking(true);

        final Completer<void> interrupt = Completer<void>();
        _speechInterrupt = interrupt;

        try {
          // iOS는 stop() 해도 speak()가 끝나지 않을 수 있어서
          // 끊김 신호와 시간 상한을 같이 기다린다.
          await Future.any<void>([
            _tts.speak(request.text).then((_) {}),
            interrupt.future,
          ]).timeout(_speechTimeoutFor(request.text));
        } catch (e) {
          debugPrint('[TTS ERROR] $e');
        } finally {
          _speechInterrupt = null;
          _currentSpeech = null;
          _setSpeaking(false);
        }

        final bool wasInterrupted = identical(_interruptedSpeech, request);
        if (wasInterrupted) {
          _interruptedSpeech = null;
        }

        if (wasInterrupted &&
            request.priority == _SpeechPriority.important &&
            !request.isStale) {
          // 급한 안내가 끝난 뒤 다시 말한다.
          _insertSpeech(request);
        } else {
          request.finish();
        }

        await Future<void>.delayed(const Duration(milliseconds: 150));
      }
    } finally {
      _isSpeechLoopRunning = false;
    }

    _resumeListeningIfNeeded();
  }

  Duration _speechTimeoutFor(String text) {
    // 한국어 TTS(속도 0.45)는 대략 초당 4~5글자
    return Duration(milliseconds: 3000 + text.length * 350);
  }

  void _setSpeaking(bool speaking) {
    _isSpeaking = speaking;

    if (mounted) {
      setState(() {
        if (speaking) _isListening = false;
      });
    }
  }

  Future<void> _stopListeningForSpeech() async {
    if (!_speech.isListening) return;

    // 목적지를 듣던 중에 끊었으면 안내가 끝난 뒤 다시 듣는다.
    if (!_isNavigationActive && !_isSearchingRoute) {
      _resumeListeningAfterSpeech = true;
    }

    try {
      await _speech.stop();
    } catch (_) {}
  }

  void _resumeListeningIfNeeded() {
    if (!_resumeListeningAfterSpeech) return;

    _resumeListeningAfterSpeech = false;

    Future<void>.delayed(const Duration(milliseconds: 400), () {
      if (!mounted ||
          _isNavigationActive ||
          _isSearchingRoute ||
          _isHandlingVoiceCommand ||
          _isSpeaking) {
        return;
      }

      unawaited(_startListening());
    });
  }

  // 대화형 안내 (질문, 응답). 말이 끝날 때까지 기다린다.
  Future<void> _speak(String text) {
    return _announce(text, priority: _SpeechPriority.important);
  }

  Future<void> _speakAndListen(String message) async {
    await _speak(message);
    await Future<void>.delayed(const Duration(milliseconds: 500));

    if (!mounted) return;
    await _startListening();
  }

  Future<void> _startListening() async {
    // 시연 중에는 음성 인식을 켜지 않는다. (주변 소리로 목적지 검색이 시작되지 않게)
    if (!_speechAvailable ||
        _isDemoMode ||
        _speech.isListening ||
        _isSpeaking ||
        _isHandlingVoiceCommand ||
        _isSearchingRoute) {
      return;
    }

    _listenRetryTimer?.cancel();

    _lastRecognizedWords = '';
    _gotFinalSpeechResult = false;
    _isProcessingSpeechEndResult = false;

    if (mounted) {
      setState(() {
        _isListening = true;
        _speechStatus = _isNavigationActive
            ? '명령을 말씀해주세요.'
            : '목적지를 말씀해주세요.';
      });
    }

    await _speech.listen(
      onResult: (result) {
        final String words = result.recognizedWords.toString().trim();

        if (!mounted) return;

        if (words.isNotEmpty) {
          _listenRetryTimer?.cancel();

          setState(() {
            _lastRecognizedWords = words;
            _speechStatus = '인식 중: $words';
          });
        }

        if (result.finalResult == true) {
          _gotFinalSpeechResult = true;
          _listenRetryTimer?.cancel();

          if (mounted) {
            setState(() {
              _isListening = false;
            });
          }

          if (words.isEmpty) {
            unawaited(
              _retryDestinationInput(
                '잘 듣지 못했습니다. 목적지를 다시 말씀해 주세요.',
              ),
            );
            return;
          }

          unawaited(_handleFinalSpeechResult(words));
        }
      },
      listenOptions: stt.SpeechListenOptions(
        localeId: 'ko_KR',
        listenFor: const Duration(seconds: 30),
        pauseFor: const Duration(seconds: 4),
        partialResults: true,
        cancelOnError: true,
        listenMode: stt.ListenMode.confirmation,
      ),
    );
  }

  Future<void> _processSpeechEndResult(String words) async {
    try {
      await _handleFinalSpeechResult(words);
    } finally {
      _isProcessingSpeechEndResult = false;
    }
  }

  Future<void> _handleFinalSpeechResult(String words) async {
    if (_isHandlingVoiceCommand) return;

    _isHandlingVoiceCommand = true;

    try {
      if (_speech.isListening) {
        try {
          await _speech.stop();
        } catch (_) {}
      }

      await _handleVoiceCommand(words);
    } finally {
      _isHandlingVoiceCommand = false;
    }
  }

  Future<void> _handleVoiceCommand(String rawCommand) async {
    final String command = rawCommand.trim();

    if (command.isEmpty) {
      await _retryDestinationInput(
        '잘 듣지 못했습니다. 목적지를 다시 말씀해 주세요.',
      );
      return;
    }

    // 길 안내 중 음성 명령
    if (_containsAny(command, ['중지', '취소', '그만', '종료', '멈춰'])) {
      await _stopNavigationByUser();
      return;
    }

    if (_containsAny(command, ['사용법', '도움말', '사용 방법', '어떻게 써'])) {
      await _speak(gestureGuideText);
      return;
    }

    if (_containsAny(command, [
      '다시 안내',
      '현재 안내',
      '반복',
      '상황',
      '남은 거리',
      '얼마나 남',
      '지금 어디',
    ])) {
      await _speakCurrentGuidance();
      return;
    }

    final String keyword = _extractDestinationKeyword(command);

    if (keyword.isEmpty) {
      _scheduleDestinationRetry(
        '목적지를 이해하지 못했습니다. 다시 말씀해 주세요.',
      );
      return;
    }

    if (mounted) {
      setState(() {
        _recognizedDestinationText = keyword;
        _speechStatus = '목적지 인식 완료';
      });
    }

    // 예전 앱과 동일하게 인식된 장소를 먼저 음성으로 확인한다.
    await _speak('$keyword에 대해 안내합니다.');
    await Future<void>.delayed(const Duration(milliseconds: 400));

    await _startNavigationByVoiceKeyword(keyword);
  }

  void _scheduleDestinationRetry(
      String guideMessage, {
        Duration delay = const Duration(milliseconds: 350),
      }) {
    _listenRetryTimer?.cancel();

    _listenRetryTimer = Timer(
      delay,
          () {
        if (!mounted || _isNavigationActive) {
          return;
        }

        // 현재 TTS/검색/음성 명령 처리가 끝나지 않았다면
        // 잠깐 기다린 뒤 다시 확인한다.
        if (_isSearchingRoute ||
            _isSpeaking ||
            _isHandlingVoiceCommand ||
            _speech.isListening) {
          _scheduleDestinationRetry(
            guideMessage,
            delay: const Duration(milliseconds: 350),
          );
          return;
        }

        unawaited(_retryDestinationInput(guideMessage));
      },
    );
  }

  Future<void> _retryDestinationInput(String guideMessage) async {
    if (!mounted ||
        _isNavigationActive ||
        _isSearchingRoute ||
        _isSpeaking ||
        _isHandlingVoiceCommand) {
      return;
    }

    _listenRetryTimer?.cancel();

    _lastRecognizedWords = '';
    _gotFinalSpeechResult = false;
    _isProcessingSpeechEndResult = false;

    if (mounted) {
      setState(() {
        _recognizedDestinationText = '';
        _speechStatus = '다시 검색합니다.';
      });
    }

    await _speak(guideMessage);
    await Future<void>.delayed(const Duration(milliseconds: 600));

    if (!mounted || _isNavigationActive || _isSearchingRoute) return;

    await _startListening();
  }

  bool _containsAny(String text, List<String> words) {
    for (final String word in words) {
      if (text.contains(word)) return true;
    }
    return false;
  }

  String _extractDestinationKeyword(String command) {
    String cleaned = command;

    const List<String> removableWords = [
      '목적지',
      '길 안내',
      '길안내',
      '안내',
      '검색',
      '찾아줘',
      '찾아 줘',
      '가자',
      '가줘',
      '가 줘',
      '이동',
      '경로',
      '네비',
      '내비',
    ];

    for (final String word in removableWords) {
      cleaned = cleaned.replaceAll(word, ' ');
    }

    cleaned = cleaned.replaceFirst(RegExp(r'(으로|로|까지|에)$'), '');
    cleaned = cleaned.replaceAll(RegExp(r'\s+'), ' ').trim();

    return cleaned;
  }

  void _setSpeechStatus(String message) {
    if (!mounted) return;
    setState(() {
      _speechStatus = message;
    });
  }

  // ===========================================================================
  // 목적지 검색 / TMAP 보행자 경로
  // ===========================================================================

  Future<void> _startNavigationByVoiceKeyword(String keyword) async {
    if (_isSearchingRoute) return;

    if (tmapAppKey.isEmpty) {
      _setSpeechStatus('TMAP 앱키가 필요합니다.');
      await _speak('TMAP 앱키가 설정되지 않았습니다.');
      return;
    }

    if (mounted) {
      setState(() {
        _isSearchingRoute = true;
        _speechStatus = '현재 위치 확인 중...';
      });
    }

    bool success = false;
    String failureMessage = '';

    try {
      final Position currentPosition = await _getCurrentGpsPosition();
      _updateCurrentLocationOnMap(currentPosition);

      if (mounted) {
        setState(() {
          _speechStatus = 'TMAP에서 $keyword 검색 중...';
        });
      }

      final Destination destination = await _searchDestinationByKeyword(
        keyword: keyword,
        currentPosition: currentPosition,
      );

      if (mounted) {
        setState(() {
          _speechStatus = '${destination.name} 보행 경로 검색 중...';
        });
      }

      final List<RoutePoint> route = await _requestTmapPedestrianRoute(
        currentPosition: currentPosition,
        destination: destination,
      );

      if (route.isEmpty) {
        throw Exception('보행 경로 좌표가 없습니다.');
      }

      if (!mounted) return;

      setState(() {
        _destination = destination;
        _recognizedDestinationText = destination.name;
        _routePoints = route;
        _routeCumulativeMeters = _buildRouteCumulativeMeters(route);
        _remainingRouteMeters = _routeCumulativeMeters.last;
        _routeProgressMeters = 0.0;
        _lastProgressPosition = null;
        _progressJumpCandidateMeters = null;
        _progressJumpCount = 0;
        _arrivalCandidateSince = null;
        _lastProgressMilestone =
            _initialProgressMilestone(_routeCumulativeMeters.last);
        _offRouteSince = null;
        _lastOffRouteAnnouncedAt = null;
        _offRouteAnnounced = false;
        _isNavigationActive = true;
        _targetBearing = null;
        _steeringError = null;
        _distanceToDestinationMeters = null;
        _distanceToNextPointMeters = null;
        _routeDeviationMeters = null;
        _currentRouteToleranceMeters = null;
        _speechStatus = '${destination.name} 경로 검색 완료';
      });

      _resetMotorCommandCache();
      _moveMapToCurrentLocation();

      await _startLocationTracking();
      _startGuidanceTimer();

      // 목적지 입력이 끝난 뒤 BLE 자동 연결 시도.
      // 실패해도 지도/길찾기는 계속 동작한다.
      if (!_isBleConnected && !_isConnecting) {
        _scheduleReconnect(Duration.zero);
      }

      await _updateGuidanceByPosition(
        currentPosition,
        forceSpeak: true,
      );

      final double routeLength =
          _remainingRouteMeters ?? _routeCumulativeMeters.last;

      unawaited(
        _speak(
          '경로를 찾았습니다. ${destination.name}까지 약 '
              '${_spokenDistance(routeLength)}입니다. '
              '지팡이가 당기는 방향을 따라 걸어 주세요.'
              '${_isBleConnected ? '' : ' 지금은 지팡이가 연결되어 있지 않습니다.'}',
        ),
      );

      success = true;
    } catch (e) {
      failureMessage = e.toString();

      if (mounted) {
        setState(() {
          _isNavigationActive = false;
          _destination = null;
          _routePoints = [];
          _routeProgressMeters = 0.0;
          _lastProgressPosition = null;
          _progressJumpCandidateMeters = null;
          _progressJumpCount = 0;
          _arrivalCandidateSince = null;
          _speechStatus = '목적지를 찾지 못했습니다.';
        });
      }
    } finally {
      if (mounted) {
        setState(() {
          _isSearchingRoute = false;
        });
      }
    }

    if (!success && mounted) {
      debugPrint('[TMAP ERROR] $failureMessage');

      // 목적지 인식 또는 TMAP 장소/보행 경로 검색에 실패하면
      // 현재 음성 명령 처리가 완전히 끝난 뒤 자동으로 다시 질문하고 듣는다.
      _scheduleDestinationRetry(
        '목적지를 찾지 못했습니다. 다시 말씀해 주세요.',
        delay: const Duration(milliseconds: 450),
      );
    }
  }

  Future<Destination> _searchDestinationByKeyword({
    required String keyword,
    required Position currentPosition,
  }) async {
    final Uri uri = Uri.https(
      tmapHost,
      tmapPoiPath,
      {
        'version': '1',
        'searchKeyword': keyword,
        'resCoordType': 'WGS84GEO',
        'reqCoordType': 'WGS84GEO',
        'count': '10',
        'centerLat': currentPosition.latitude.toString(),
        'centerLon': currentPosition.longitude.toString(),
        'radius': '33',
      },
    );

    final http.Response response = await http.get(
      uri,
      headers: {
        'accept': 'application/json',
        'appKey': tmapAppKey,
      },
    ).timeout(const Duration(seconds: 12));

    if (response.statusCode != 200) {
      throw Exception(
        'TMAP 장소 검색 실패: HTTP ${response.statusCode}',
      );
    }

    final Map<String, dynamic> data =
    jsonDecode(utf8.decode(response.bodyBytes)) as Map<String, dynamic>;

    final List<dynamic> pois =
        data['searchPoiInfo']?['pois']?['poi'] as List<dynamic>? ?? [];

    if (pois.isEmpty) {
      throw Exception('검색 결과가 없습니다.');
    }

    Map<String, dynamic>? bestPoi;
    double? bestLatitude;
    double? bestLongitude;
    int bestNameRank = 999;
    double bestDistance = double.infinity;

    final String normalizedKeyword = _normalizePlaceName(keyword);

    for (final dynamic raw in pois) {
      if (raw is! Map<String, dynamic>) continue;

      final String name = raw['name']?.toString().trim() ?? '';
      final String normalizedName = _normalizePlaceName(name);

      final String? latText =
      raw['frontLat']?.toString().trim().isNotEmpty == true
          ? raw['frontLat']?.toString()
          : raw['noorLat']?.toString();

      final String? lonText =
      raw['frontLon']?.toString().trim().isNotEmpty == true
          ? raw['frontLon']?.toString()
          : raw['noorLon']?.toString();

      final double? lat = double.tryParse(latText ?? '');
      final double? lon = double.tryParse(lonText ?? '');

      if (lat == null || lon == null) continue;

      int nameRank;

      if (normalizedName == normalizedKeyword) {
        nameRank = 0;
      } else if (normalizedName.contains(normalizedKeyword)) {
        nameRank = 1;
      } else if (normalizedKeyword.contains(normalizedName) &&
          normalizedName.isNotEmpty) {
        nameRank = 2;
      } else {
        nameRank = 3;
      }

      final double distance = Geolocator.distanceBetween(
        currentPosition.latitude,
        currentPosition.longitude,
        lat,
        lon,
      );

      if (nameRank < bestNameRank ||
          (nameRank == bestNameRank && distance < bestDistance)) {
        bestPoi = raw;
        bestLatitude = lat;
        bestLongitude = lon;
        bestNameRank = nameRank;
        bestDistance = distance;
      }
    }

    if (bestPoi == null ||
        bestLatitude == null ||
        bestLongitude == null) {
      throw Exception('검색 결과의 좌표를 읽을 수 없습니다.');
    }

    return Destination(
      name: bestPoi['name']?.toString() ?? keyword,
      latitude: bestLatitude,
      longitude: bestLongitude,
    );
  }

  String _normalizePlaceName(String value) {
    return value
        .toLowerCase()
        .replaceAll(RegExp(r'[\s\-\(\)\[\]·.]'), '');
  }

  Future<List<RoutePoint>> _requestTmapPedestrianRoute({
    required Position currentPosition,
    required Destination destination,
  }) async {
    final Uri uri = Uri.https(
      tmapHost,
      tmapPedestrianPath,
      {'version': '1'},
    );

    final Map<String, dynamic> body = {
      'startX': currentPosition.longitude.toString(),
      'startY': currentPosition.latitude.toString(),
      'endX': destination.longitude.toString(),
      'endY': destination.latitude.toString(),
      'reqCoordType': 'WGS84GEO',
      'resCoordType': 'WGS84GEO',
      'startName': '현재 위치',
      'endName': destination.name,
      'searchOption': '0',
    };

    final http.Response response = await http
        .post(
      uri,
      headers: {
        'accept': 'application/json',
        'content-type': 'application/json',
        'appKey': tmapAppKey,
      },
      body: jsonEncode(body),
    )
        .timeout(const Duration(seconds: 15));

    if (response.statusCode != 200) {
      final String errorBody = utf8.decode(response.bodyBytes);

      throw Exception(
        'TMAP 보행자 경로 요청 실패: '
            'HTTP ${response.statusCode} $errorBody',
      );
    }

    final Map<String, dynamic> data =
    jsonDecode(utf8.decode(response.bodyBytes)) as Map<String, dynamic>;

    final List<dynamic> features =
        data['features'] as List<dynamic>? ?? [];

    final List<RoutePoint> points = [];

    for (final dynamic feature in features) {
      if (feature is! Map<String, dynamic>) continue;

      final Map<String, dynamic>? geometry =
      feature['geometry'] as Map<String, dynamic>?;

      if (geometry == null) continue;

      _appendRoutePointsFromGeometry(
        geometry,
        points,
      );
    }

    if (points.isEmpty) {
      throw Exception('TMAP 응답에서 경로 좌표를 찾지 못했습니다.');
    }

    return _removeTooCloseDuplicatePoints(points);
  }

  void _appendRoutePointsFromGeometry(
      Map<String, dynamic> geometry,
      List<RoutePoint> output,
      ) {
    final String type = geometry['type']?.toString() ?? '';
    final dynamic coordinates = geometry['coordinates'];

    if (type == 'LineString' && coordinates is List) {
      for (final dynamic coordinate in coordinates) {
        final RoutePoint? point = _routePointFromCoordinate(coordinate);
        if (point != null) {
          output.add(point);
        }
      }
      return;
    }

    if (type == 'Point') {
      final RoutePoint? point = _routePointFromCoordinate(coordinates);
      if (point != null) {
        output.add(point);
      }
    }
  }

  RoutePoint? _routePointFromCoordinate(dynamic coordinate) {
    if (coordinate is! List || coordinate.length < 2) {
      return null;
    }

    final double? lon = _toDouble(coordinate[0]);
    final double? lat = _toDouble(coordinate[1]);

    if (lat == null || lon == null) {
      return null;
    }

    return RoutePoint(
      latitude: lat,
      longitude: lon,
    );
  }

  double? _toDouble(dynamic value) {
    if (value is num) return value.toDouble();
    if (value is String) return double.tryParse(value);
    return null;
  }

  List<double> _buildRouteCumulativeMeters(List<RoutePoint> points) {
    final List<double> cumulative = [0.0];

    for (int i = 1; i < points.length; i++) {
      cumulative.add(
        cumulative.last +
            Geolocator.distanceBetween(
              points[i - 1].latitude,
              points[i - 1].longitude,
              points[i].latitude,
              points[i].longitude,
            ),
      );
    }

    return cumulative;
  }

  // 경로를 따라 남은 거리 (직선거리가 아니라 실제로 걸어야 하는 거리)
  double _remainingRouteDistance(_RouteProjection projection) {
    final List<double> cumulative = _routeCumulativeMeters;

    if (cumulative.isEmpty) {
      return 0.0;
    }

    return math.max(0.0, cumulative.last - projection.alongMeters);
  }

  // 시작 시점보다 먼 기준점은 안내하지 않도록, 시작 거리 이상인
  // 가장 작은 기준점을 "이미 지난 것"으로 둔다.
  double? _initialProgressMilestone(double routeLength) {
    double? result;

    for (final double milestone in progressMilestonesMeters) {
      if (milestone >= routeLength) {
        result = result == null ? milestone : math.min(result, milestone);
      }
    }

    return result;
  }

  // TTS로 읽기 좋은 거리 표현
  String _spokenDistance(double meters) {
    if (meters >= 1000) {
      final double km = meters / 1000.0;
      return '${km.toStringAsFixed(km >= 10 ? 0 : 1)}킬로미터';
    }

    if (meters >= 100) {
      return '${(meters / 50).round() * 50}미터';
    }

    return '${math.max(10, (meters / 10).round() * 10)}미터';
  }

  List<RoutePoint> _removeTooCloseDuplicatePoints(
      List<RoutePoint> input,
      ) {
    if (input.isEmpty) return [];

    final List<RoutePoint> result = [input.first];

    for (int i = 1; i < input.length; i++) {
      final RoutePoint previous = result.last;
      final RoutePoint current = input[i];

      final double distance = Geolocator.distanceBetween(
        previous.latitude,
        previous.longitude,
        current.latitude,
        current.longitude,
      );

      if (distance >= 1.0) {
        result.add(current);
      }
    }

    return result;
  }

  // ===========================================================================
  // GPS / 내비게이션
  // ===========================================================================

  Future<Position> _getCurrentGpsPosition() async {
    final bool serviceEnabled = await Geolocator.isLocationServiceEnabled();

    if (!serviceEnabled) {
      throw Exception('휴대폰 위치 서비스가 꺼져 있습니다.');
    }

    LocationPermission permission = await Geolocator.checkPermission();if (permission == LocationPermission.denied) {
      permission = await Geolocator.requestPermission();
    }

    if (permission == LocationPermission.denied) {
      throw Exception('위치 권한이 거부되었습니다.');
    }

    if (permission == LocationPermission.deniedForever) {
      throw Exception(
        '위치 권한이 영구적으로 거부되었습니다. '
            '설정에서 위치 권한을 허용해 주세요.',
      );
    }

    const LocationSettings settings = LocationSettings(
      accuracy: LocationAccuracy.bestForNavigation,
    );

    return Geolocator.getCurrentPosition(
      locationSettings: settings,
    );
  }

  void _updateCurrentLocationOnMap(Position position) {
    if (!mounted) return;

    setState(() {
      _currentLatLng = LatLng(
        position.latitude,
        position.longitude,
      );
    });
  }

  void _moveMapToCurrentLocation() {
    final LatLng? current = _currentLatLng;
    if (current == null) return;

    try {
      _mapController.move(current, 17);
    } catch (_) {}
  }

  Future<void> _startLocationTracking() async {
    await _positionSubscription?.cancel();

    // 제자리에 서 있어도(신호 대기 등) 위치가 계속 들어오도록 distanceFilter 0.
    // 위치가 gpsStaleAfter 동안 안 오면 방향 유도를 멈추기 때문에,
    // "움직이지 않아서 안 온 것"과 "GPS가 끊긴 것"을 구분하려면 주기적으로 받아야 한다.
    final LocationSettings settings = Platform.isAndroid
        ? AndroidSettings(
      accuracy: LocationAccuracy.bestForNavigation,
      distanceFilter: 0,
      intervalDuration: const Duration(seconds: 1),
    )
        : Platform.isIOS
        ? AppleSettings(
      accuracy: LocationAccuracy.bestForNavigation,
      distanceFilter: 0,
      activityType: ActivityType.fitness,
      pauseLocationUpdatesAutomatically: false,
    )
        : const LocationSettings(
      accuracy: LocationAccuracy.bestForNavigation,
      distanceFilter: 0,
    );

    _positionSubscription = Geolocator.getPositionStream(
      locationSettings: settings,
    ).listen(
          (Position position) {
        if (!_isNavigationActive) return;

        _updateCurrentLocationOnMap(position);
        _moveMapToCurrentLocation();

        unawaited(
          _updateGuidanceByPosition(
            position,
            forceSpeak: false,
          ),
        );
      },
      onError: (Object error) {
        // 여기서는 기록만 한다. 위치가 gpsStaleAfter 동안 안 들어오면
        // _checkNavigationInputs()가 방향 유도를 멈추고 알린다.
        debugPrint('[GPS ERROR] $error');
      },
    );
  }

  void _startGuidanceTimer() {
    _guidanceTimer?.cancel();

    _navInputWatchdog?.cancel();
    _navInputWatchdog = Timer.periodic(
      const Duration(seconds: 1),
      (_) => unawaited(_checkNavigationInputs()),
    );

    // 예전 앱과 동일하게 주기적으로 현재 위치 기준으로 안내 갱신.
    _guidanceTimer = Timer.periodic(
      const Duration(seconds: 12),
          (_) async {
        if (!_isNavigationActive) return;

        try {
          final Position position = await _getCurrentGpsPosition();
          await _updateGuidanceByPosition(
            position,
            forceSpeak: true,
          );
        } catch (e) {
          debugPrint('[GUIDANCE TIMER] $e');
        }
      },
    );
  }

  Future<void> _updateGuidanceByPosition(
      Position currentPosition, {
        required bool forceSpeak,
      }) async {
    if (_isGuidanceUpdating ||
        !_isNavigationActive ||
        _destination == null ||
        _routePoints.isEmpty) {
      return;
    }

    _isGuidanceUpdating = true;

    try {
      final Destination destination = _destination!;

      // 오래된 위치(캐시)나 정확도가 나쁜 위치로는 방향을 정하지 않는다.
      // 이전 _targetBearing으로 계속 유도하지 않도록, 유효한 위치가
      // gpsStaleAfter 동안 없으면 _checkNavigationInputs()가 유도를 멈춘다.
      final double accuracy = currentPosition.accuracy;
      final Duration fixAge =
          DateTime.now().difference(currentPosition.timestamp);

      _lastFixAccuracy = accuracy;

      if (!accuracy.isFinite ||
          accuracy > gpsMaxAccuracyMeters ||
          fixAge > gpsStaleAfter) {
        debugPrint(
          '[GPS] unusable fix: accuracy=${accuracy.toStringAsFixed(1)}m '
              'age=${fixAge.inMilliseconds}ms',
        );
        await _checkNavigationInputs();
        return;
      }

      _lastGoodFixAt = DateTime.now();

      final double distanceToDestination = Geolocator.distanceBetween(
        currentPosition.latitude,
        currentPosition.longitude,
        destination.latitude,
        destination.longitude,
      );

      // 현재 위치가 TMAP 경로선에서 얼마나 벗어났는지 계산한다.
      // 경로 '점'까지의 거리가 아니라 인접한 경로 좌표 사이의 선분까지
      // 최단 거리를 계산하므로 실제 보행 경로와의 좌우 오차를 판단한다.
      // GPS가 한 번 튀어서 경로의 엉뚱한 구간으로 넘어가지 않도록
      // 이동 거리와 진행 방향을 함께 확인한다. (_selectRouteProjection)
      final _RouteProjection routeProjection =
      _selectRouteProjection(currentPosition);

      final double remainingRoute = _remainingRouteDistance(routeProjection);

      if (_checkArrival(
        position: currentPosition,
        distanceToDestination: distanceToDestination,
        remainingRoute: remainingRoute,
      )) {
        await _finishNavigation();
        return;
      }

      final double routeTolerance = _routeToleranceFor(currentPosition);

      final bool isInsideRouteTolerance =
          routeProjection.distanceMeters <= routeTolerance;

      // 허용 폭 안: 선 위로 끌어당기지 않고 경로와 나란한 방향을 목표로 함.
      // 허용 폭 밖: 벗어난 거리만큼만 경로 쪽으로 틀어 완만하게 복귀시킨다.
      final double targetBearing = _calculateRouteFollowingBearing(
        currentPosition: currentPosition,
        projection: routeProjection,
        routePoints: _routePoints,
        routeTolerance: routeTolerance,
        destination: destination,
      );

      final RoutePoint nextRoutePoint = _routePoints[math.min(
        routeProjection.segmentIndex + 1,
        _routePoints.length - 1,
      )];

      final double distanceToNextPoint = Geolocator.distanceBetween(
        currentPosition.latitude,
        currentPosition.longitude,
        nextRoutePoint.latitude,
        nextRoutePoint.longitude,
      );

      debugPrint(
        '[ROUTE] deviation=${routeProjection.distanceMeters.toStringAsFixed(2)}m '
            'tolerance=${routeTolerance.toStringAsFixed(1)}m '
            'target=${isInsideRouteTolerance ? 'FOLLOW_ROUTE' : 'RETURN_TO_ROUTE'} '
            'bearing=${targetBearing.toStringAsFixed(0)}',
      );

      if (mounted) {
        setState(() {
          _currentLatLng = LatLng(
            currentPosition.latitude,
            currentPosition.longitude,
          );

          _targetBearing = targetBearing;
          _distanceToDestinationMeters = distanceToDestination;
          _distanceToNextPointMeters = distanceToNextPoint;
          _routeDeviationMeters = routeProjection.distanceMeters;
          _currentRouteToleranceMeters = routeTolerance;
          _remainingRouteMeters = remainingRoute;
        });
      }

      final double? remaining = _remainingRouteMeters;
      if (remaining != null) {
        _announceProgressIfNeeded(remaining);
      }

      _updateOffRouteAnnouncement(isInsideRouteTolerance);

      await _sendBearingToCaneIfNeeded(targetBearing);

      // 목표 방위각이 바뀌었으므로 직진/좌/우 판단도 다시 한다.
      await _updateSteering();

      // 직진 / 왼쪽 / 오른쪽 길안내 TTS는 사용하지 않는다.
      // 화면 표시와 ESP32 방향 명령(F/L/R)은 계속 동작한다.
    } finally {
      _isGuidanceUpdating = false;
    }
  }

  // 경로 선분 [fromMeters, toMeters] 구간에서 현재 위치에 가장 가까운 점.
  // 범위를 주지 않으면 경로 전체를 찾는다.
  _RouteProjection _projectOnRoute(
      Position currentPosition, {
        double fromMeters = double.negativeInfinity,
        double toMeters = double.infinity,
      }) {
    final List<RoutePoint> routePoints = _routePoints;
    final List<double> cumulative = _routeCumulativeMeters;

    if (routePoints.isEmpty) {
      throw StateError('경로 좌표가 없습니다.');
    }

    if (routePoints.length == 1 || cumulative.length != routePoints.length) {
      final double distance = Geolocator.distanceBetween(
        currentPosition.latitude,
        currentPosition.longitude,
        routePoints.first.latitude,
        routePoints.first.longitude,
      );

      return _RouteProjection(
        nearestPoint: routePoints.first,
        distanceMeters: distance,
        segmentIndex: 0,
        alongMeters: 0.0,
      );
    }

    double bestDistanceMeters = double.infinity;
    RoutePoint bestPoint = routePoints.first;
    int bestSegmentIndex = 0;
    double bestAlong = 0.0;

    for (int i = 0; i < routePoints.length - 1; i++) {
      // 검색 범위 밖의 선분은 건너뛴다.
      if (cumulative[i + 1] < fromMeters || cumulative[i] > toMeters) {
        continue;
      }

      final RoutePoint a = routePoints[i];
      final RoutePoint b = routePoints[i + 1];

      // 현재 GPS 위치를 원점(0, 0)으로 두고 위경도를 짧은 거리의
      // 평면 좌표(m)로 바꾼 뒤, 원점에서 선분 AB까지의 최단점을 구한다.
      final _LocalMeters localA = _toLocalMeters(currentPosition, a);
      final _LocalMeters localB = _toLocalMeters(currentPosition, b);

      final double ax = localA.x;
      final double ay = localA.y;
      final double dx = localB.x - ax;
      final double dy = localB.y - ay;
      final double segmentLengthSquared = dx * dx + dy * dy;

      double t = 0.0;

      if (segmentLengthSquared > 0.000001) {
        // P=(0,0)을 선분 AB에 투영.
        t = -(ax * dx + ay * dy) / segmentLengthSquared;
        t = t.clamp(0.0, 1.0).toDouble();
      }

      final double nearestX = ax + t * dx;
      final double nearestY = ay + t * dy;
      final double distanceMeters =
      math.sqrt(nearestX * nearestX + nearestY * nearestY);

      if (distanceMeters < bestDistanceMeters) {
        bestDistanceMeters = distanceMeters;
        bestSegmentIndex = i;
        bestAlong = cumulative[i] + (cumulative[i + 1] - cumulative[i]) * t;

        bestPoint = RoutePoint(
          latitude: a.latitude + (b.latitude - a.latitude) * t,
          longitude:
          a.longitude + (b.longitude - a.longitude) * t,
        );
      }
    }

    return _RouteProjection(
      nearestPoint: bestPoint,
      distanceMeters: bestDistanceMeters,
      segmentIndex: bestSegmentIndex,
      alongMeters: bestAlong,
    );
  }

  // 지금 진행 위치 주변(뒤 30m ~ 앞 60m)에서 현재 위치를 경로에 맞춘다.
  //
  // - 진행 위치는 뒤로도 갈 수 있다. (잘못 넘어갔다가 바로잡을 수 있게)
  // - 진행 위치 변화가 실제 이동 거리(+GPS 정확도 + 여유)보다 크거나,
  //   앞으로 넘어가는데 걷는 방향이 그 구간 방향과 90° 넘게 다르면
  //   GPS 튐으로 보고 바로 받아들이지 않는다.
  //   같은 곳을 progressJumpConfirmCount번 연속 가리키면 그때 받아들인다.
  // - 검색 범위 안에서 경로가 너무 멀면(크게 벗어남 / GPS 공백 뒤)
  //   경로 전체에서 다시 찾고, 같은 확인 과정을 거친다.
  _RouteProjection _selectRouteProjection(Position position) {
    final double current = _routeProgressMeters;
    final double tolerance = _routeToleranceFor(position);

    _RouteProjection candidate = _projectOnRoute(
      position,
      fromMeters: current - progressSearchBackMeters,
      toMeters: current + progressSearchAheadMeters,
    );

    if (candidate.distanceMeters > tolerance * 3) {
      final _RouteProjection global = _projectOnRoute(position);

      if (global.distanceMeters < candidate.distanceMeters - 5.0) {
        candidate = global;
      }
    }

    final Position? last = _lastProgressPosition;
    bool plausible = true;

    if (last != null) {
      final double moved = Geolocator.distanceBetween(
        last.latitude,
        last.longitude,
        position.latitude,
        position.longitude,
      );

      final double allowed = moved +
          (position.accuracy.isFinite ? position.accuracy : gpsMaxAccuracyMeters) +
          progressJumpMarginMeters;

      final double advance = candidate.alongMeters - current;

      if (advance.abs() > allowed) {
        plausible = false;
      }

      // 앞으로 넘어가는데 실제로 걸은 방향이 그 구간과 반대쪽이면 의심
      if (plausible && advance > 5.0 && moved >= 3.0) {
        final RoutePoint a = _routePoints[candidate.segmentIndex];
        final RoutePoint b = _routePoints[math.min(
          candidate.segmentIndex + 1,
          _routePoints.length - 1,
        )];

        final double walkBearing = _calculateTargetBearing(
          startLatitude: last.latitude,
          startLongitude: last.longitude,
          destinationLatitude: position.latitude,
          destinationLongitude: position.longitude,
        );

        final double segmentBearing = _calculateTargetBearing(
          startLatitude: a.latitude,
          startLongitude: a.longitude,
          destinationLatitude: b.latitude,
          destinationLongitude: b.longitude,
        );

        if (_angleDifference(walkBearing, segmentBearing) > 90.0) {
          plausible = false;
        }
      }
    }

    if (!plausible) {
      final double? pending = _progressJumpCandidateMeters;

      if (pending != null &&
          (candidate.alongMeters - pending).abs() <= progressJumpMarginMeters) {
        _progressJumpCount++;
      } else {
        _progressJumpCandidateMeters = candidate.alongMeters;
        _progressJumpCount = 1;
      }

      if (_progressJumpCount < progressJumpConfirmCount) {
        debugPrint(
          '[ROUTE] progress jump ${current.toStringAsFixed(0)}m -> '
              '${candidate.alongMeters.toStringAsFixed(0)}m held '
              '($_progressJumpCount/$progressJumpConfirmCount)',
        );

        // 확인될 때까지는 지금 진행 위치 바로 주변에서만 맞춘다.
        // 기준 위치(_lastProgressPosition)는 바꾸지 않는다.
        // (튄 위치를 기준으로 다음 이동 거리를 재면 튐이 정상처럼 보이므로)
        return _projectOnRoute(
          position,
          fromMeters: current - progressJumpMarginMeters,
          toMeters: current + progressJumpMarginMeters,
        );
      }

      debugPrint('[ROUTE] progress jump confirmed');
    }

    _routeProgressMeters = candidate.alongMeters;
    _lastProgressPosition = position;
    _progressJumpCandidateMeters = null;
    _progressJumpCount = 0;

    return candidate;
  }

  // 도착 판정
  //  - 목적지 직선거리 arrivalDistanceMeters 이하
  //  - 경로상 남은 거리 arrivalRouteRemainingMeters 이하
  //    (도로 반대편처럼 직선으로는 가깝지만 길로는 먼 곳을 거른다)
  //  - GPS 정확도 arrivalMaxAccuracyMeters 이하
  //  - 횡단 중이 아님
  // 이 상태가 arrivalConfirmDuration 동안 이어지면 도착.
  bool _checkArrival({
    required Position position,
    required double distanceToDestination,
    required double remainingRoute,
  }) {
    final bool near = distanceToDestination <= arrivalDistanceMeters &&
        remainingRoute <= arrivalRouteRemainingMeters &&
        position.accuracy <= arrivalMaxAccuracyMeters &&
        !_isCrossing;

    if (!near) {
      _arrivalCandidateSince = null;
      return false;
    }

    final DateTime now = DateTime.now();
    final DateTime? since = _arrivalCandidateSince;

    if (since == null) {
      _arrivalCandidateSince = now;
      return false;
    }

    return now.difference(since) >= arrivalConfirmDuration;
  }

  // GPS 오차(accuracy)가 기본 허용 폭보다 크면 그만큼 넓혀서 판단한다.
  double _routeToleranceFor(Position position) {
    final double accuracy = position.accuracy;

    if (!accuracy.isFinite || accuracy <= routeToleranceMeters) {
      return routeToleranceMeters;
    }

    return math.min(accuracy, maxRouteToleranceMeters);
  }

  // 초록색 경로선을 정확히 밟지 않아도 되도록, 경로선을 현재 위치 쪽으로
  // 최대 허용 폭만큼 평행 이동한 "가상 경로"를 따라가게 한다.
  //  - 허용 폭 안: 가상 경로가 현재 위치를 지나므로 경로 진행 방향만 안내
  //  - 허용 폭 밖: 가상 경로까지 남은 거리만큼만 경로 쪽으로 틀어 합류
  //    (틀어 주는 각도 = atan(허용 폭을 넘은 거리 / routeLookAheadMeters))
  double _calculateRouteFollowingBearing({
    required Position currentPosition,
    required _RouteProjection projection,
    required List<RoutePoint> routePoints,
    required double routeTolerance,
    required Destination destination,
  }) {
    final RoutePoint lookAheadPoint = _pointAlongRoute(
      projection: projection,
      routePoints: routePoints,
      distanceMeters: routeLookAheadMeters,
    );

    final _LocalMeters lookAhead =
    _toLocalMeters(currentPosition, lookAheadPoint);

    double targetX = lookAhead.x;
    double targetY = lookAhead.y;

    final int segmentIndex = projection.segmentIndex;

    if (segmentIndex + 1 < routePoints.length) {
      final _LocalMeters a =
      _toLocalMeters(currentPosition, routePoints[segmentIndex]);
      final _LocalMeters b =
      _toLocalMeters(currentPosition, routePoints[segmentIndex + 1]);
      final _LocalMeters nearest =
      _toLocalMeters(currentPosition, projection.nearestPoint);

      final double dx = b.x - a.x;
      final double dy = b.y - a.y;
      final double segmentLength = math.sqrt(dx * dx + dy * dy);

      if (segmentLength > 0.001) {
        // 경로 진행 방향 기준 왼쪽을 가리키는 단위 벡터
        final double leftX = -dy / segmentLength;
        final double leftY = dx / segmentLength;

        // 경로선에서 현재 위치까지의 좌우 거리 (+: 경로 왼쪽, -: 경로 오른쪽)
        final double lateralOffset =
        -(nearest.x * leftX + nearest.y * leftY);

        final double shift = lateralOffset
            .clamp(-routeTolerance, routeTolerance)
            .toDouble();

        targetX += leftX * shift;
        targetY += leftY * shift;
      }
    }

    // 경로 끝에 거의 도착해 따라갈 경로가 남아 있지 않으면 목적지 방향 사용
    if (targetX * targetX + targetY * targetY < 1.0) {
      return _calculateTargetBearing(
        startLatitude: currentPosition.latitude,
        startLongitude: currentPosition.longitude,
        destinationLatitude: destination.latitude,
        destinationLongitude: destination.longitude,
      );
    }

    // 평면 좌표(x: 동, y: 북)에서 북쪽 기준 시계 방향 방위각
    return (_radianToDegree(math.atan2(targetX, targetY)) + 360.0) % 360.0;
  }

  // 경로선 위의 투영점에서 경로를 따라 distanceMeters만큼 앞의 지점
  RoutePoint _pointAlongRoute({
    required _RouteProjection projection,
    required List<RoutePoint> routePoints,
    required double distanceMeters,
  }) {
    double remaining = distanceMeters;
    RoutePoint from = projection.nearestPoint;

    for (int i = projection.segmentIndex + 1; i < routePoints.length; i++) {
      final RoutePoint to = routePoints[i];

      final double segmentLength = Geolocator.distanceBetween(
        from.latitude,
        from.longitude,
        to.latitude,
        to.longitude,
      );

      if (segmentLength > 0 && segmentLength >= remaining) {
        final double t = remaining / segmentLength;

        return RoutePoint(
          latitude: from.latitude + (to.latitude - from.latitude) * t,
          longitude: from.longitude + (to.longitude - from.longitude) * t,
        );
      }

      remaining -= segmentLength;
      from = to;
    }

    return routePoints.last;
  }

  // 현재 위치를 원점으로 하는 짧은 거리용 평면 좌표(m)
  _LocalMeters _toLocalMeters(Position origin, RoutePoint point) {
    const double earthRadiusMeters = 6371000.0;
    final double cosOriginLat = math.cos(_degreeToRadian(origin.latitude));

    return (
    x: _degreeToRadian(point.longitude - origin.longitude) *
        cosOriginLat *
        earthRadiusMeters,
    y: _degreeToRadian(point.latitude - origin.latitude) *
        earthRadiusMeters,
    );
  }

  // ===========================================================================
  // 길 안내 음성
  //
  // 직진/왼쪽/오른쪽은 지팡이 모터가 몸으로 알려 주므로 음성으로 읽지 않는다.
  // 대신 화면을 볼 수 없는 사용자가 "지금 어떤 상황인지"를 알 수 있게
  //  - 남은 거리 기준점(1km, 500m, ... 50m)
  //  - 경로 이탈 / 복귀
  //  - "상황" 음성 명령, "지금 상황 듣기" 버튼
  // 을 알려 준다.
  // ===========================================================================

  void _announceProgressIfNeeded(double remainingMeters) {
    // 횡단·장애물 안내가 우선. 끝나고 다음 위치 갱신 때 알려 준다.
    if (_isCrosswalkMode || _obstacleConfirmed) return;

    double? crossed;

    for (final double milestone in progressMilestonesMeters) {
      final double? last = _lastProgressMilestone;

      if (remainingMeters <= milestone && (last == null || milestone < last)) {
        crossed = crossed == null ? milestone : math.min(crossed, milestone);
      }
    }

    if (crossed == null) return;

    _lastProgressMilestone = crossed;

    _announce(
      '목적지까지 약 ${_spokenDistance(remainingMeters)} 남았습니다.',
      key: 'progress',
      maxAge: const Duration(seconds: 10),
    );
  }

  void _updateOffRouteAnnouncement(bool isInsideRouteTolerance) {
    final DateTime now = DateTime.now();

    if (isInsideRouteTolerance) {
      _offRouteSince = null;

      if (_offRouteAnnounced) {
        _offRouteAnnounced = false;
        _announce(
          '경로로 돌아왔습니다.',
          key: 'route',
          maxAge: const Duration(seconds: 8),
        );
      }
      return;
    }

    _offRouteSince ??= now;

    // 횡단보도·장애물 회피 중에는 경로에서 잠깐 벗어나는 게 정상이다.
    if (_isCrosswalkMode || _obstacleConfirmed) return;

    if (now.difference(_offRouteSince!) < offRouteAnnounceDelay) return;

    final DateTime? lastAt = _lastOffRouteAnnouncedAt;
    if (lastAt != null && now.difference(lastAt) < offRouteRepeatInterval) {
      return;
    }

    _lastOffRouteAnnouncedAt = now;
    _offRouteAnnounced = true;

    _announce(
      '경로에서 벗어났습니다. 지팡이가 당기는 방향으로 천천히 돌아가 주세요.',
      priority: _SpeechPriority.important,
      key: 'route',
      maxAge: const Duration(seconds: 8),
    );
  }

  Future<void> _speakCurrentGuidance() async {
    final List<String> parts = [];

    final Destination? destination = _destination;
    final double? remaining = _remainingRouteMeters;

    if (_isDemoMode) {
      parts.add(_demoStepSpeech());
    } else if (_isNavigationActive && destination != null) {
      parts.add(
        remaining == null
            ? '${destination.name}까지 안내 중입니다.'
            : '${destination.name}까지 약 ${_spokenDistance(remaining)} 남았습니다.',
      );
    } else {
      parts.add('현재 진행 중인 길 안내가 없습니다.');
    }

    if (_isCrossing) {
      parts.add('지금은 횡단보도를 건너는 중입니다. 지팡이를 따라 끝까지 건너 주세요.');
    } else if (_isCrosswalkMode) {
      if (_trafficLight == 'RED' || _trafficLight == 'GREEN_WAIT') {
        parts.add('앞에 횡단보도가 있고, 신호를 기다리는 중입니다.');
      } else {
        parts.add('앞에 횡단보도가 있습니다.');
      }
    }

    if (_obstacleConfirmed) {
      parts.add(
        _avoidDirection == 'BLOCKED'
            ? '앞이 막혀 있습니다.'
            : '장애물을 피하는 중입니다.',
      );
    }

    parts.add(
      _isBleConnected
          ? '지팡이는 연결되어 있습니다.'
          : '지팡이가 연결되어 있지 않습니다.',
    );

    if (_isBleConnected && _frontSensorLost) {
      parts.add('앞 장애물 센서가 응답하지 않습니다.');
    }

    await _speak(parts.join(' '));
  }

  Future<void> _stopNavigationByUser() async {
    if (_isDemoMode) {
      await _exitDemoMode();
      return;
    }

    await _positionSubscription?.cancel();
    _positionSubscription = null;

    _guidanceTimer?.cancel();
    _guidanceTimer = null;

    _navInputWatchdog?.cancel();
    _navInputWatchdog = null;
    _navInputProblem = null;
    _lastGoodFixAt = null;

    if (mounted) {
      setState(() {
        _isNavigationActive = false;
        _routePoints = [];
        _routeProgressMeters = 0.0;
        _lastProgressPosition = null;
        _progressJumpCandidateMeters = null;
        _progressJumpCount = 0;
        _arrivalCandidateSince = null;
        _destination = null;
        _targetBearing = null;
        _steeringError = null;
        _distanceToDestinationMeters = null;
        _distanceToNextPointMeters = null;
        _routeDeviationMeters = null;
        _currentRouteToleranceMeters = null;
        _remainingRouteMeters = null;
        _recognizedDestinationText = '';
        _speechStatus = '안내가 중지되었습니다.';
      });
    }

    _dropQueuedSpeech((r) => r.key == 'progress' || r.key == 'route');

    await _sendStopToCane();
    await _speak('길 안내를 중지합니다.');

    await Future<void>.delayed(const Duration(milliseconds: 500));

    if (mounted) {
      await _speakAndListen('새 목적지를 말씀해주세요.');
    }
  }

  Future<void> _finishNavigation() async {
    await _positionSubscription?.cancel();
    _positionSubscription = null;

    _guidanceTimer?.cancel();
    _guidanceTimer = null;

    _navInputWatchdog?.cancel();
    _navInputWatchdog = null;
    _navInputProblem = null;
    _lastGoodFixAt = null;

    if (mounted) {
      setState(() {
        _isNavigationActive = false;
        _routePoints = [];
        _routeProgressMeters = 0.0;
        _lastProgressPosition = null;
        _progressJumpCandidateMeters = null;
        _progressJumpCount = 0;
        _arrivalCandidateSince = null;
        _targetBearing = null;
        _steeringError = null;
        _distanceToNextPointMeters = null;
        _distanceToDestinationMeters = 0;
        _routeDeviationMeters = null;
        _currentRouteToleranceMeters = null;
        _remainingRouteMeters = null;
        _speechStatus = '목적지에 도착했습니다.';
      });
    }

    _dropQueuedSpeech((r) => r.key == 'progress' || r.key == 'route');

    await _sendStopToCane();
    await _speak('목적지에 도착했습니다. 안내를 종료합니다.');
  }

  // ===========================================================================
  // 스마트폰 나침반 / 방위각
  // ===========================================================================

  void _startCompass() {
    final Stream<CompassEvent>? stream = FlutterCompass.events;

    if (stream == null) return;

    // Android 나침반(자북 기준)을 TMAP 경로와 같은 진북 기준으로 맞춘다.
    final double declination =
    Platform.isAndroid ? magneticDeclinationDegrees : 0.0;

    _compassSubscription = stream.listen(
          (CompassEvent event) {
        final double? raw = event.heading;

        if (raw == null || !raw.isFinite || !mounted) {
          return;
        }

        // 걸을 때 흔들리는 값을 그대로 쓰지 않고 평활화한 값을 사용한다.
        final double heading = _smoothCompassHeading(
          (raw + declination + 360.0) % 360.0,
        );

        setState(() {
          _phoneHeading = heading;
        });

        unawaited(_updateSteering());
      },
    );
  }

  // 지수 이동 평균으로 나침반 값을 부드럽게 만든다.
  // 이벤트 간격에 맞춰 가중치를 정하므로 기기마다 나침반 주기가 달라도
  // 반응 속도는 compassSmoothingSeconds로 같다.
  double _smoothCompassHeading(double rawHeading) {
    final DateTime now = DateTime.now();
    final double rawRad = _degreeToRadian(rawHeading);
    final double rawSin = math.sin(rawRad);
    final double rawCos = math.cos(rawRad);

    final DateTime? lastEventAt = _lastCompassEventAt;

    if (lastEventAt == null) {
      _headingSin = rawSin;
      _headingCos = rawCos;
    } else {
      final double dtSeconds = math.max(
        0.0,
        now.difference(lastEventAt).inMicroseconds / 1000000.0,
      );
      final double alpha =
          1.0 - math.exp(-dtSeconds / compassSmoothingSeconds);

      _headingSin += (rawSin - _headingSin) * alpha;
      _headingCos += (rawCos - _headingCos) * alpha;
    }

    _lastCompassEventAt = now;

    return (_radianToDegree(math.atan2(_headingSin, _headingCos)) + 360.0) %
        360.0;
  }

  // 목표 방위각과 나침반 방위각의 오차로 직진/좌/우를 정해 ESP32로 보낸다.
  // 나침반 이벤트(Android 기준 초당 약 30회)와 GPS 갱신 양쪽에서 호출된다.
  Future<void> _updateSteering() async {
    final double? targetBearing = _targetBearing;
    final double? heading = _phoneHeading;

    if (!mounted ||
        !_isNavigationActive ||
        targetBearing == null ||
        heading == null) {
      return;
    }

    final double error = _signedAngleDifference(
      targetBearing,
      heading,
    );

    // 장애물을 막 피한 직후에는 잠깐 쉬어서 장애물 쪽으로 다시 끌지 않음
    final DateTime? clearedAt = _obstacleClearedAt;
    final bool justAvoidedObstacle = clearedAt != null &&
        DateTime.now().difference(clearedAt) < obstacleClearPause;

    // 횡단을 막 마친 직후에도 잠깐 쉰다.
    final DateTime? crossingEndedAt = _crossingEndedAt;
    final bool justFinishedCrossing = crossingEndedAt != null &&
        DateTime.now().difference(crossingEndedAt) < crossingClearPause;

    // 횡단보도(Raspberry Pi) / 장애물 회피(ESP32) / 정면 센서 이상 /
    // GPS·나침반 값이 오래됨 중에는 화면 값만 갱신
    final bool canSteer = !_frontSensorLost &&
        _navInputProblem == null &&
        !_isCrosswalkMode &&
        !_obstacleConfirmed &&
        !justAvoidedObstacle &&
        !justFinishedCrossing;

    if (canSteer) {
      _updateSteeringCommand(error);
    }

    setState(() {
      _steeringError = error;
    });

    if (canSteer) {
      await _sendMotorCommandToCaneIfNeeded(
        _steeringCommand,
        angle: error.abs(),
      );
    }
  }

  // 길 안내 중 방향 유도에 쓰는 값이 살아 있는지 확인한다. (1초마다 + 나쁜 GPS 수신 시)
  //  - GPS: 유효한 위치(정확도 gpsMaxAccuracyMeters 이하)가 gpsStaleAfter 동안 없음
  //  - 나침반: 이벤트가 compassStaleAfter 동안 없음
  // 문제가 생기면 L/R 유도를 멈추고(F) 한 번 알린다. 회복되면 다시 알린다.
  Future<void> _checkNavigationInputs() async {
    if (!mounted || !_isNavigationActive) return;

    final DateTime now = DateTime.now();
    final DateTime? fixAt = _lastGoodFixAt;
    final DateTime? compassAt = _lastCompassEventAt;

    // 도착 조건을 만족한 채로 arrivalConfirmDuration이 지났으면 도착.
    // (도착 지점에 서 있으면 새 위치가 와도 같은 판정이므로 여기서도 확인)
    final DateTime? arrivalSince = _arrivalCandidateSince;
    if (arrivalSince != null &&
        fixAt != null &&
        now.difference(fixAt) <= gpsStaleAfter &&
        now.difference(arrivalSince) >= arrivalConfirmDuration &&
        !_isGuidanceUpdating) {
      await _finishNavigation();
      return;
    }

    String? problem;

    if (fixAt == null || now.difference(fixAt) > gpsStaleAfter) {
      problem = 'gps';
    } else if (compassAt == null || now.difference(compassAt) > compassStaleAfter) {
      problem = 'compass';
    }

    if (problem == _navInputProblem) return;

    final String? previous = _navInputProblem;

    setState(() {
      _navInputProblem = problem;
    });

    if (problem != null) {
      debugPrint(
        '[NAV] steering paused: $problem '
            '(accuracy=${_lastFixAccuracy?.toStringAsFixed(1)}m)',
      );

      // 지금 당기고 있으면 멈춘다.
      if (_steeringCommand != 'F' || _lastMotorCommand != 'F') {
        _resetMotorCommandCache();
        await _sendMotorCommandToCaneIfNeeded('F', angle: 0);
      }

      _announce(
        problem == 'gps'
            ? 'GPS 신호가 약해서 방향 안내를 잠시 멈춥니다. 주변을 확인하며 천천히 걸어 주세요.'
            : '휴대폰 방향 센서 값이 들어오지 않아 방향 안내를 잠시 멈춥니다.',
        priority: _SpeechPriority.important,
        key: 'nav_input',
        vibrate: true,
      );
    } else if (previous != null) {
      debugPrint('[NAV] steering resumed');

      _announce(
        '방향 안내를 다시 시작합니다.',
        priority: _SpeechPriority.important,
        key: 'nav_input',
        maxAge: const Duration(seconds: 10),
      );
    }
  }

  // 히스테리시스 + 유지 시간으로 직진/좌/우 판단이 자주 바뀌지 않게 한다.
  void _updateSteeringCommand(double steeringError) {
    final String candidate = _steeringCandidate(steeringError);

    if (candidate == _steeringCommand) {
      _pendingSteeringCommand = null;
      _pendingSteeringSince = null;
      return;
    }

    // 지금 유도 중인 방향과 다른 판단이 나오면 우선 바로 직진(유도 중지)으로.
    _steeringCommand = 'F';

    if (candidate == 'F') {
      _pendingSteeringCommand = null;
      _pendingSteeringSince = null;
      return;
    }

    // 좌/우는 같은 판단이 steeringHoldDuration 동안 이어질 때만 반영한다.
    final DateTime now = DateTime.now();
    final DateTime? pendingSince = _pendingSteeringSince;

    if (candidate != _pendingSteeringCommand || pendingSince == null) {
      _pendingSteeringCommand = candidate;
      _pendingSteeringSince = now;
      return;
    }

    if (now.difference(pendingSince) >= steeringHoldDuration) {
      _steeringCommand = candidate;
      _pendingSteeringCommand = null;
      _pendingSteeringSince = null;
    }
  }

  String _steeringCandidate(double steeringError) {
    // 이미 같은 방향으로 유도 중이면 더 작은 오차(headingReleaseDegrees)까지
    // 그 방향을 유지
    if (_steeringCommand == 'R' && steeringError > headingReleaseDegrees) {
      return 'R';
    }

    if (_steeringCommand == 'L' && steeringError < -headingReleaseDegrees) {
      return 'L';
    }

    // 직진 중에는 오차가 headingToleranceDegrees를 넘어야 좌/우로 판단
    if (steeringError > headingToleranceDegrees) return 'R';
    if (steeringError < -headingToleranceDegrees) return 'L';

    return 'F';
  }

  double _calculateTargetBearing({
    required double startLatitude,
    required double startLongitude,
    required double destinationLatitude,
    required double destinationLongitude,
  }) {
    final double startLatRad = _degreeToRadian(startLatitude);
    final double destinationLatRad =
    _degreeToRadian(destinationLatitude);

    final double deltaLongitudeRad = _degreeToRadian(
      destinationLongitude - startLongitude,
    );

    final double y =
        math.sin(deltaLongitudeRad) * math.cos(destinationLatRad);

    final double x =
        math.cos(startLatRad) * math.sin(destinationLatRad) -
            math.sin(startLatRad) *
                math.cos(destinationLatRad) *
                math.cos(deltaLongitudeRad);

    final double bearingRad = math.atan2(y, x);

    double bearingDegree = _radianToDegree(bearingRad);
    bearingDegree = (bearingDegree + 360.0) % 360.0;

    return bearingDegree;
  }

  double _signedAngleDifference(
      double targetBearing,
      double currentHeading,
      ) {
    return ((targetBearing - currentHeading + 540.0) % 360.0) - 180.0;
  }

  double _angleDifference(double a, double b) {
    return _signedAngleDifference(a, b).abs();
  }

  double _degreeToRadian(double degree) {
    return degree * math.pi / 180.0;
  }

  double _radianToDegree(double radian) {
    return radian * 180.0 / math.pi;
  }

  // 화면에는 실제로 지팡이에 보내고 있는 방향을 표시한다.
  String _directionText() {
    if (!_isNavigationActive) return '안내 대기';
    if (_isCrosswalkMode) return '횡단보도 제어 중';
    if (_obstacleConfirmed) return '장애물 회피 중';
    if (_steeringError == null) return '나침반 준비 중';
    if (_steeringCommand == 'R') return '오른쪽';
    if (_steeringCommand == 'L') return '왼쪽';
    return '직진';
  }

  String _routeDeviationText() {
    final double? deviation = _routeDeviationMeters;
    final double? tolerance = _currentRouteToleranceMeters;

    if (deviation == null || tolerance == null) return '--';

    return '${deviation.toStringAsFixed(1)} m '
        '(허용 ${tolerance.toStringAsFixed(0)} m)';
  }

  String _headingText() {
    final double? h = _phoneHeading;

    if (h == null) return '--';

    String direction;

    if (h >= 337.5 || h < 22.5) {
      direction = '북';
    } else if (h < 67.5) {
      direction = '북동';
    } else if (h < 112.5) {
      direction = '동';
    } else if (h < 157.5) {
      direction = '남동';
    } else if (h < 202.5) {
      direction = '남';
    } else if (h < 247.5) {
      direction = '남서';
    } else if (h < 292.5) {
      direction = '서';
    } else {
      direction = '북서';
    }

    return '${h.toStringAsFixed(0)}° $direction';
  }

  // ===========================================================================
  // BLE
  // ===========================================================================

  Future<bool> _requestBlePermissions() async {
    if (!Platform.isAndroid) return true;

    final Map<Permission, PermissionStatus> result =
    await <Permission>[
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.locationWhenInUse,
    ].request();

    final PermissionStatus? scan =
    result[Permission.bluetoothScan];

    final PermissionStatus? connect =
    result[Permission.bluetoothConnect];

    final PermissionStatus? location =
    result[Permission.locationWhenInUse];

    if (scan?.isPermanentlyDenied == true ||
        connect?.isPermanentlyDenied == true ||
        location?.isPermanentlyDenied == true) {
      return false;
    }

    if (location?.isDenied == true) {
      return false;
    }

    return true;
  }

  // silent: 실패해도 화면 SnackBar를 띄우지 않음
  // announceFailure: 실패를 음성으로 알림 (버튼으로 직접 연결할 때는 항상 알림)
  //                  자동 재연결은 매번 말하지 않고 _scheduleReconnect가 주기적으로 알린다.
  Future<void> _connectSmartCane({
    bool silent = false,
    bool announceFailure = false,
  }) async {
    if (_isConnecting || _isBleConnected) return;

    if (mounted) {
      setState(() {
        _isConnecting = true;
        _bleStatus = 'SMART_CANE 연결 준비 중...';
      });
    }

    try {
      final bool permissionsOk = await _requestBlePermissions();

      if (!permissionsOk) {
        throw Exception('블루투스/위치 권한이 필요합니다.');
      }

      if (!await FlutterBluePlus.isSupported) {
        throw Exception('이 휴대폰은 BLE를 지원하지 않습니다.');
      }

      if (!kIsWeb &&
          Platform.isAndroid &&
          FlutterBluePlus.adapterStateNow != BluetoothAdapterState.on) {
        try {
          await FlutterBluePlus.turnOn();
        } catch (_) {}
      }

      await FlutterBluePlus.adapterState
          .where((state) => state == BluetoothAdapterState.on)
          .first
          .timeout(
        const Duration(seconds: 10),
        onTimeout: () {
          throw TimeoutException(
            '휴대폰 블루투스를 켜주세요.',
          );
        },
      );

      if (mounted) {
        setState(() {
          _bleStatus = '$targetDeviceName 검색 중...';
        });
      }

      final Completer<BluetoothDevice?> completer =
      Completer<BluetoothDevice?>();

      late final StreamSubscription<List<ScanResult>> scanSubscription;

      scanSubscription = FlutterBluePlus.onScanResults.listen(
            (List<ScanResult> results) {
          for (final ScanResult result in results) {
            final String advName =
            result.advertisementData.advName.trim();

            final String platformName =
            result.device.platformName.trim();

            if (advName == targetDeviceName ||
                platformName == targetDeviceName) {
              if (!completer.isCompleted) {
                completer.complete(result.device);
              }
              return;
            }
          }
        },
        onError: (_) {
          if (!completer.isCompleted) {
            completer.complete(null);
          }
        },
      );

      await FlutterBluePlus.startScan(
        timeout: const Duration(seconds: 10),
      );

      final BluetoothDevice? foundDevice =
      await completer.future.timeout(
        const Duration(seconds: 11),
        onTimeout: () => null,
      );

      await FlutterBluePlus.stopScan();
      await scanSubscription.cancel();

      if (foundDevice == null) {
        throw TimeoutException('SMART_CANE을 찾지 못했습니다.');
      }

      if (mounted) {
        setState(() {
          _bleStatus = 'ESP32 연결 중...';
        });
      }

      if (foundDevice.isDisconnected) {
        await foundDevice.connect(
          license: License.nonprofit,
          timeout: const Duration(seconds: 15),
        );
      }

      final List<BluetoothService> services =
      await foundDevice.discoverServices();

      final Guid targetService = Guid(serviceUuid);
      final Guid targetRx = Guid(rxUuid);
      final Guid targetTx = Guid(txUuid);

      BluetoothCharacteristic? writeCharacteristic;
      BluetoothCharacteristic? notifyCharacteristic;

      for (final BluetoothService service in services) {
        if (service.serviceUuid != targetService) continue;

        for (final BluetoothCharacteristic characteristic
        in service.characteristics) {
          if (characteristic.characteristicUuid == targetRx) {
            writeCharacteristic = characteristic;
          }

          if (characteristic.characteristicUuid == targetTx) {
            notifyCharacteristic = characteristic;
          }
        }
      }

      if (writeCharacteristic == null ||
          notifyCharacteristic == null) {
        await foundDevice.disconnect();
        throw Exception(
          'ESP32 Nordic UART Service 특성을 찾지 못했습니다.',
        );
      }

      await _bleConnectionSubscription?.cancel();

      _bleConnectionSubscription =
          foundDevice.connectionState.listen(
                (BluetoothConnectionState state) {
              if (!mounted) return;

              if (state ==
                  BluetoothConnectionState.disconnected) {

                final bool wasConnected = _isBleConnected;

                setState(() {
                  _isBleConnected = false;
                  _bleStatus = '연결 끊김';

                  _device = null;
                  _writeCharacteristic = null;
                  _notifyCharacteristic = null;

                  _distanceCm = null;
                  _leftDistanceCm = null;
                  _rightDistanceCm = null;

                  _obstacleConfirmed = false;
                  _avoidDirection = 'NONE';

                  _isCrosswalkMode = false;
                  _crosswalkDetected = false;
                  _trafficLight = 'NONE';
                  _crosswalkSteering = 'NONE';
                  _isCrossing = false;
                  // 다시 연결되면 ESP32가 아직 끊겨 있는 경우 SENSOR:FRONT_LOST를 다시 보낸다.
                  _frontSensorLost = false;
                });

                _resetMotorCommandCache();

                // 지팡이가 끊기면 장애물 감지와 방향 유도가 모두 멈춘다.
                // 사용자가 모른 채 계속 걷지 않도록 바로 알린다.
                if (wasConnected) {
                  _bleWasLost = true;
                  _lastDisconnectReminderAt = DateTime.now();

                  _dropQueuedSpeech(
                        (r) => r.key == 'obstacle' || r.key == 'avoid',
                  );

                  _announce(
                    '지팡이 연결이 끊겼습니다. 장애물을 감지할 수 없으니 '
                        '잠시 멈춰 주세요. 다시 연결하고 있습니다.',
                    priority: _SpeechPriority.urgent,
                    key: 'ble',
                    vibrate: true,
                  );

                  _scheduleReconnect(const Duration(seconds: 2));
                }
              }
            },
          );

      await _bleNotifySubscription?.cancel();

      _bleNotifySubscription =
          notifyCharacteristic.onValueReceived.listen(
            _onBleData,
          );

      await notifyCharacteristic.setNotifyValue(true);

      if (!mounted) return;

      setState(() {
        _device = foundDevice;
        _writeCharacteristic = writeCharacteristic;
        _notifyCharacteristic = notifyCharacteristic;
        _writeWithoutResponse =
            !writeCharacteristic!.properties.write &&
                writeCharacteristic.properties.writeWithoutResponse;
        _isBleConnected = true;
        _bleStatus = '연결됨';
      });

      _reconnectTimer?.cancel();
      _reconnectTimer = null;

      _announce(
        _bleWasLost ? '지팡이가 다시 연결되었습니다.' : '지팡이가 연결되었습니다.',
        priority: _bleWasLost ? _SpeechPriority.important : _SpeechPriority.info,
        key: 'ble',
      );

      _bleWasLost = false;

      // 끊긴 동안의 상태를 앱이 모르므로, ESP32의 현재 상태를 한 번에 받아 온다.
      // ESP32는 상태를 바뀔 때만 보내기 때문에, 이렇게 묻지 않으면
      // 이미 회피 중이거나 빨간불인 상태를 놓칠 수 있다. (응답: ST:...)
      unawaited(_requestCaneState());
    } catch (e) {
      if (mounted) {
        setState(() {
          _isBleConnected = false;
          _bleStatus = '연결 실패';
        });
      }

      debugPrint('[BLE ERROR] $e');

      if (announceFailure || !silent) {
        _announce(
          '지팡이 연결에 실패했습니다. 지팡이 전원과 휴대폰 블루투스를 확인해 주세요. '
              '계속 다시 연결을 시도합니다.',
          priority: _SpeechPriority.important,
          key: 'ble',
        );

        _scheduleReconnect();
      }

      if (!silent) {
        ScaffoldMessenger.of(context)
          ..hideCurrentSnackBar()
          ..showSnackBar(
            SnackBar(
              content: Text('BLE 연결 실패: $e'),
            ),
          );
      }
    } finally {
      if (mounted) {
        setState(() {
          _isConnecting = false;
        });
      }
    }
  }

  // 연결이 끊겨 있으면 reconnectInterval마다 조용히 다시 연결한다.
  // 길 안내 중이면 disconnectReminderInterval마다 아직 끊겨 있다고 알린다.
  void _scheduleReconnect([Duration delay = reconnectInterval]) {
    _reconnectTimer?.cancel();

    _reconnectTimer = Timer(delay, () async {
      if (!mounted || _isBleConnected) return;

      if (!_isConnecting) {
        await _connectSmartCane(silent: true);
      }

      if (!mounted || _isBleConnected) return;

      final DateTime now = DateTime.now();
      final DateTime? lastReminder = _lastDisconnectReminderAt;

      if (_isNavigationActive &&
          (lastReminder == null ||
              now.difference(lastReminder) >= disconnectReminderInterval)) {
        _lastDisconnectReminderAt = now;

        _announce(
          '지팡이 연결이 아직 끊겨 있습니다. 주의해서 걸어 주세요.',
          priority: _SpeechPriority.important,
          key: 'ble',
        );
      }

      _scheduleReconnect();
    });
  }

  // 성공하면 true. 연결이 없거나 쓰기에 실패하면 false.
  // (withoutResponse 쓰기는 ESP32가 받았는지까지는 알 수 없고, 휴대폰이 보냈는지까지만 안다)
  Future<bool> _sendBleLine(String message) async {
    final BluetoothCharacteristic? characteristic =
        _writeCharacteristic;

    if (!_isBleConnected || characteristic == null) {
      return false;
    }

    try {
      await characteristic.write(
        utf8.encode('$message\n'),
        withoutResponse: _writeWithoutResponse,
      );
      return true;
    } catch (e) {
      debugPrint('[BLE TX ERROR] $message: $e');
      return false;
    }
  }

  // ESP32 현재 상태 요청 (Q). 응답은 ST:로 온다. 응답이 없으면 두 번까지 다시 묻는다.
  Future<void> _requestCaneState() async {
    _caneStateReceived = false;

    for (int attempt = 0; attempt < 3 && _isBleConnected; attempt++) {
      if (attempt > 0) {
        await Future<void>.delayed(const Duration(seconds: 1));
        if (_caneStateReceived || !_isBleConnected) return;
      }

      await _sendBleLine('Q');
      await Future<void>.delayed(const Duration(milliseconds: 800));

      if (_caneStateReceived) return;
    }

    debugPrint('[BLE] no ST: reply (old firmware?)');
  }

  // 정지 명령(S:0). 실패하면 몇 번 다시 보내고, 끝내 실패하면 알린다.
  // (연결 자체가 끊긴 경우에는 ESP32가 연결 해제를 감지해서 스스로 멈춘다)
  Future<bool> _sendStopToCane() async {
    _resetMotorCommandCache();

    if (!_isBleConnected) return false;

    for (int attempt = 1; attempt <= stopCommandRetries; attempt++) {
      if (await _sendBleLine('S:0')) {
        return true;
      }

      if (!_isBleConnected) return false;

      await Future<void>.delayed(const Duration(milliseconds: 200));
    }

    _announce(
      '지팡이에 정지 명령을 보내지 못했습니다. 지팡이가 계속 당기면 전원을 꺼 주세요.',
      priority: _SpeechPriority.urgent,
      key: 'ble_tx',
      vibrate: true,
    );

    return false;
  }

  Future<void> _sendBearingToCaneIfNeeded(
      double bearing,
      ) async {
    if (!_isBleConnected) return;

    final int bearingInt = bearing.round() % 360;
    final DateTime now = DateTime.now();

    final bool changed = _lastSentBearing == null ||
        _angleDifference(
          bearingInt.toDouble(),
          _lastSentBearing!.toDouble(),
        ) >=
            3.0;

    final bool enoughTime = _lastBearingSentAt == null ||
        now.difference(_lastBearingSentAt!).inMilliseconds >= 1500;

    if (!changed && !enoughTime) {
      return;
    }

    if (await _sendBleLine('B:$bearingInt')) {
      _lastSentBearing = bearingInt;
      _lastBearingSentAt = now;
    }
  }

  Future<void> _sendMotorCommandToCaneIfNeeded(
      String command, {
        required double angle,
      }) async {
    if (!_isBleConnected) return;

    // 횡단보도에서는 Raspberry Pi가 제어
    if (_isCrosswalkMode) {
      return;
    }

    // ESP32가 장애물을 처리하는 동안
    // Flutter의 L/R/F 명령 차단
    if (_obstacleConfirmed) {
      return;
    }

    // 정면 센서 이상 / GPS·나침반 문제 중에는 당기는 명령(L/R)을 보내지 않는다.
    // (F는 "유도 멈춤"이므로 보낸다)
    if (command != 'F' && (_frontSensorLost || _navInputProblem != null)) {
      return;
    }

    // 이전 전송이 아직 끝나지 않았으면 겹쳐 보내지 않는다.
    // (나침반 이벤트가 초당 수십 번 오므로)
    if (_motorSendInFlight) {
      return;
    }

    final DateTime now = DateTime.now();

    // 방금 실패했으면 아주 잠깐 쉬었다가 다시 보낸다.
    final DateTime? failedAt = _lastMotorFailedAt;
    if (failedAt != null && now.difference(failedAt) < motorRetryInterval) {
      return;
    }

    final bool commandChanged =
        command != _lastMotorCommand;

    // 같은 명령은 motorRepeatInterval마다 한 번만 다시 보낸다.
    // 나침반이 조금 흔들릴 때마다 L/R을 연달아 보내지 않도록
    // 각도 변화만으로는 다시 보내지 않는다.
    final bool enoughTime =
        _lastMotorSentAt == null ||
            now.difference(_lastMotorSentAt!) >= motorRepeatInterval;

    if (!commandChanged && !enoughTime) {
      return;
    }

    _motorSendInFlight = true;

    final bool ok;

    try {
      ok = await _sendBleLine(
        '$command:${angle.toStringAsFixed(1)}',
      );
    } finally {
      _motorSendInFlight = false;
    }

    // 성공했을 때만 "보낸 명령"으로 기록한다.
    // 실패하면 기록하지 않으므로 motorRetryInterval 뒤 다음 판단 때 다시 보낸다.
    if (ok) {
      _lastMotorCommand = command;
      _lastMotorSentAt = now;
      _lastMotorFailedAt = null;
    } else {
      _lastMotorFailedAt = DateTime.now();
    }

    debugPrint(
      '[NAV -> ESP32] '
          '$command:${angle.toStringAsFixed(1)} ${ok ? 'OK' : 'FAILED'}',
    );
  }

  // ===========================================================================
  // ESP32 -> 앱 수신
  // ===========================================================================

  void _onBleData(List<int> bytes) {
    final String message =
    utf8.decode(bytes, allowMalformed: true).trim();

    if (message.isEmpty) return;

    // 한 notify에 여러 줄이 들어온 경우도 처리.
    final List<String> lines = message
        .split(RegExp(r'[\r\n]+'))
        .map((line) => line.trim())
        .where((line) => line.isNotEmpty)
        .toList();

    for (final String line in lines) {
      _handleEsp32Message(line);
    }
  }

  void _handleEsp32Message(String message) {
    debugPrint('[ESP32 RX] $message');

    // ============================================================
    // 1. TFmini 거리
    //
    // ESP32 -> Flutter
    // DIST:85
    //
    // 앱:
    // 40cm 이하 -> 화면 장애물 표시
    // ============================================================
    if (message.startsWith('DIST:')) {
      final double? distance = double.tryParse(
        message.substring('DIST:'.length).trim(),
      );

      if (distance != null && mounted) {
        setState(() {
          _distanceCm = distance;
        });
      }

      return;
    }

    // ============================================================
    // 1-1. 좌/우 VL53L1X 거리
    //
    // ESP32 -> Flutter
    // SIDE:왼쪽cm,오른쪽cm
    // 예: SIDE:82,135
    // ============================================================
    if (message.startsWith('SIDE:')) {
      final String payload =
      message.substring('SIDE:'.length).trim();
      final List<String> values = payload.split(',');

      if (values.length >= 2) {
        final double? left =
        double.tryParse(values[0].trim());
        final double? right =
        double.tryParse(values[1].trim());

        if (mounted) {
          setState(() {
            _leftDistanceCm =
            (left != null && left > 0) ? left : null;
            _rightDistanceCm =
            (right != null && right > 0) ? right : null;
          });
        }
      }

      return;
    }

    // LEFT_DIST / RIGHT_DIST 형식도 같이 지원
    // 나중에 ESP32 메시지 형식을 따로 보낼 때 그대로 사용 가능.
    if (message.startsWith('LEFT_DIST:')) {
      final double? distance = double.tryParse(
        message.substring('LEFT_DIST:'.length).trim(),
      );

      if (distance != null && distance > 0 && mounted) {
        setState(() {
          _leftDistanceCm = distance;
        });
      }

      return;
    }

    if (message.startsWith('RIGHT_DIST:')) {
      final double? distance = double.tryParse(
        message.substring('RIGHT_DIST:'.length).trim(),
      );

      if (distance != null && distance > 0 && mounted) {
        setState(() {
          _rightDistanceCm = distance;
        });
      }

      return;
    }

    // ============================================================
    // 1-2. 역토크 시작 음성 안내
    //
    // ESP32 -> Flutter
    // OBSTACLE_BRAKE
    //
    // TFmini 40cm 이하가 3회 연속 확인되어 실제 역토크가
    // 시작될 때만 ESP32가 한 번 전송한다.
    // 40 cm 거리 표시만으로는 음성이 나오지 않고, 역토크 시작 시점에만 음성이 나온다.
    // ============================================================
    if (message == 'OBSTACLE_BRAKE') {
      // 가장 급한 안내. "멈추세요"를 먼저 말해야 바로 반응할 수 있다.
      _announce(
        '멈추세요. 앞에 장애물이 있습니다.',
        priority: _SpeechPriority.urgent,
        key: 'obstacle',
        maxAge: const Duration(seconds: 3),
        vibrate: true,
      );

      debugPrint('[OBSTACLE] brake voice guidance');
      return;
    }

    // ============================================================
    // 0. 연결 직후 상태 복원 (Q에 대한 응답)
    //
    // ST:abcdefg (BLE 기본 MTU 20바이트 안에 들어가도록 한 글자씩)
    //  a 장애물 처리 중   0 / 1
    //  b 회피 방향        N / L / R / B(막힘)
    //  c 횡단보도 모드    0 / 1
    //  d 신호             N / R(빨강) / G(초록) / W(초록 대기)
    //  e 횡단 중          0 / 1
    //  f 정면 센서        1(정상) / 0(이상)
    //  g 정지 유지        0 / 1 (다음 L/R/F 명령에 풀림, 로그용)
    // ============================================================
    if (message.startsWith('ST:')) {
      _applyCaneState(message.substring(3));
      return;
    }

    // ============================================================
    // 1-3. 정면 센서(TFmini) 끊김 / 복구
    //
    // ESP32 -> Flutter
    // SENSOR:FRONT_LOST : TFmini 프레임이 안 옴. 끊긴 동안 3초마다 다시 온다.
    // SENSOR:FRONT_OK   : 다시 들어옴
    //
    // 끊긴 동안은 장애물을 감지할 수 없으므로 상태가 바뀔 때 한 번 알린다.
    // (이 때 ESP32는 OBSTACLE_CLEAR를 보내지 않으므로 장애물 상태는 여기서 정리)
    // ============================================================
    if (message == 'SENSOR:FRONT_LOST' || message == 'SENSOR:FRONT_OK') {
      final bool lost = message == 'SENSOR:FRONT_LOST';

      if (lost == _frontSensorLost) {
        return;
      }

      if (mounted) {
        setState(() {
          _frontSensorLost = lost;

          if (lost) {
            _distanceCm = null;
            _obstacleConfirmed = false;
            _avoidDirection = 'NONE';
          }
        });
      } else {
        _frontSensorLost = lost;
      }

      if (lost) {
        _dropQueuedSpeech((r) => r.key == 'obstacle' || r.key == 'avoid');

        _announce(
          '앞 장애물 센서가 응답하지 않습니다. 지팡이로 앞을 확인하며 천천히 걸어 주세요.',
          priority: _SpeechPriority.urgent,
          key: 'front_sensor',
          vibrate: true,
        );
      } else {
        _announce(
          '앞 장애물 센서가 다시 작동합니다.',
          priority: _SpeechPriority.important,
          key: 'front_sensor',
          maxAge: const Duration(seconds: 10),
        );
      }

      debugPrint('[SENSOR] front ${lost ? 'LOST' : 'OK'}');
      return;
    }

    // ============================================================
    // 2. 장애물 확정
    //
    // ESP32에서 40cm 이하 감지 시:
    // OBSTACLE:25
    //
    // 이 메시지를 보낸 직후 ESP32가
    // 좌/우 VL53L1X 거리를 비교해 회피 방향을 결정함.
    // ============================================================
    if (message.startsWith('OBSTACLE:')) {
      final double? distance = double.tryParse(
        message.substring('OBSTACLE:'.length).trim(),
      );

      if (mounted) {
        setState(() {
          _obstacleConfirmed = true;
          _avoidDirection = 'NONE';

          if (_isDemoMode && _demoStep == 3) {
            _demoObstacleSeen = true;
          }

          if (distance != null) {
            _distanceCm = distance;
          }
        });
      }

      // 장애물 처리 중 Flutter의 이전 L/R/F 캐시 초기화
      _resetMotorCommandCache();

      debugPrint(
        '[OBSTACLE] confirmed - ESP32 side-distance avoidance',
      );

      return;
    }

    // ============================================================
    // 2-1. 장애물 회피 방향
    //
    // ESP32 -> Flutter
    // AVOID:L / AVOID:R : 정면이 풀릴 때까지 그 방향으로 계속 유도
    // AVOID:BLOCKED     : 양옆도 막혔거나, 계속 유도해도 정면이 안 풀림
    // ============================================================
    if (message.startsWith('AVOID:')) {
      final String side = message.substring('AVOID:'.length).trim();

      final String direction = side == 'L'
          ? 'LEFT'
          : side == 'R'
          ? 'RIGHT'
          : 'BLOCKED';

      if (direction != _avoidDirection) {
        // key가 같아서, 밀려 있던 이전 방향 안내는 새 방향으로 바뀐다.
        _announce(
          direction == 'LEFT'
              ? '왼쪽으로 피해 주세요.'
              : direction == 'RIGHT'
              ? '오른쪽으로 피해 주세요.'
              : '앞이 막혀 있습니다. 멈춰서 주변을 확인해 주세요.',
          priority: _SpeechPriority.urgent,
          key: 'avoid',
          maxAge: const Duration(seconds: 4),
        );
      }

      if (mounted) {
        setState(() {
          _avoidDirection = direction;
        });
      }

      debugPrint('[OBSTACLE] avoid direction: $direction');
      return;
    }

    // ============================================================
    // 3. 장애물 해제
    //
    // ESP32에서 TFmini 거리 50cm 이상이 되면:
    // OBSTACLE_CLEAR
    // ============================================================
    if (message == 'OBSTACLE_CLEAR') {
      final bool wasAvoiding =
          _avoidDirection == 'LEFT' || _avoidDirection == 'RIGHT';

      if (mounted) {
        setState(() {
          _obstacleConfirmed = false;
          _avoidDirection = 'NONE';
        });
      }

      // 방향 유도는 잠깐 쉬었다가 다시 시작 (obstacleClearPause)
      _obstacleClearedAt = DateTime.now();

      // 아직 말하지 못한 장애물/회피 안내는 이제 맞지 않으므로 버린다.
      _dropQueuedSpeech((r) => r.key == 'obstacle' || r.key == 'avoid');

      if (wasAvoiding) {
        _announce(
          '장애물을 지났습니다.',
          priority: _SpeechPriority.important,
          key: 'avoid',
          maxAge: const Duration(seconds: 5),
        );
      }

      // 다시 일반 TMAP 방향 유도 가능
      _resetMotorCommandCache();

      debugPrint('[OBSTACLE] cleared');

      return;
    }

    // ============================================================
    // 4. 횡단보도 인식
    //
    // Raspberry Pi
    // -> ESP32
    // -> Flutter
    //
    // CROSSWALK:1
    //
    // 이때 Flutter의 일반 TMAP L/R/F 명령 중지
    // ============================================================
    if (message == 'CROSSWALK:1') {
      final bool wasDetected = _crosswalkDetected;

      if (mounted) {
        setState(() {
          _crosswalkDetected = true;
          _isCrosswalkMode = true;
        });
      }

      _resetMotorCommandCache();

      // 횡단 종료 직후(crossingGrace)라면 같은 횡단보도를 다시 봤을 수도 있다.
      // 도로 위에서 "멈춰 주세요"라고 하면 위험하므로 바로 말하지 않고,
      // 유예가 끝났는데도 횡단보도 앞이면 그때 말한다.
      if (!wasDetected) {
        if (_inCrossingGrace) {
          _deferCrosswalkAnnouncement();
        } else {
          _announceCrosswalkAhead();
        }
      }

      debugPrint('[MODE] CROSSWALK MODE');

      return;
    }

    // ============================================================
    // 5. 횡단보도 사라짐
    //
    // CROSSWALK:0
    // ============================================================
    if (message == 'CROSSWALK:0') {
      _endCrossing();

      if (mounted) {
        setState(() {
          _crosswalkDetected = false;
          _isCrosswalkMode = false;
          _trafficLight = 'NONE';
          _crosswalkSteering = 'NONE';
        });
      }

      _resetMotorCommandCache();

      debugPrint('[MODE] NORMAL NAVIGATION');

      return;
    }

    // ============================================================
    // 6. 빨간불
    //
    // LIGHT:RED
    // ============================================================
    // ============================================================
    // 6-0. 횡단 중 빨간불
    //
    // ESP32 -> CROSSING_RED
    // 이미 도로 위에 있으므로 멈추게 하지 않고 끝까지 건너게 안내한다.
    // 예전 ESP32 펌웨어는 횡단 중에도 LIGHT:RED를 보내므로,
    // 실제 횡단 중(CROSSING_START ~ END)이면 LIGHT:RED도 같은 안내로 바꾼다.
    // 횡단 종료 직후 유예는 ESP32가 판단한다. 앱이 유예 시간으로 다시 거르면
    // BLE 지연 때문에 유예가 끝난 뒤의 LIGHT:RED까지 "계속 건너세요"로 바뀔 수 있다.
    // ============================================================
    if (message == 'CROSSING_RED' ||
        (_isCrossing &&
            (message == 'LIGHT:RED' || message == 'SIGNAL_RED'))) {
      if (mounted) {
        setState(() {
          _trafficLight = 'CROSSING_RED';
        });
      }

      // 앞서 밀려 있던 "정지해 주세요" 류 안내가 있으면 버린다.
      _dropQueuedSpeech((r) => r.key == 'light');

      if (!_crossingRedAnnounced) {
        _crossingRedAnnounced = true;

        // 실제 횡단 중(CROSSING_START ~ END)이면 끝까지 건너라고 한다.
        // 횡단 종료 직후 유예 중이면 앱은 아직 도로 위인지 이미 건넜는지 모른다.
        // 시간만으로 "건너는 중"이라고 단정하지 않고 두 경우 모두 맞는 말로 안내한다.
        // (유예가 끝나도 횡단보도 앞이면 ESP32가 LIGHT:RED를 보내고 "멈추세요"가 나온다)
        _announce(
          _isCrossing
              ? '신호가 바뀌었습니다. 멈추지 말고 지팡이를 따라 끝까지 건너 주세요.'
              : '빨간불로 바뀌었습니다. 아직 도로 위라면 멈추지 말고 끝까지 건너고, '
                  '이미 건넜다면 그 자리에서 기다려 주세요.',
          priority: _SpeechPriority.urgent,
          key: 'light',
          vibrate: true,
        );
      }

      debugPrint('[TRAFFIC] RED during crossing -> keep crossing');

      return;
    }

    if (message == 'LIGHT:RED' ||
        message == 'SIGNAL_RED') {
      final String oldLight = _trafficLight;
      final bool oldCrosswalk = _crosswalkDetected;

      if (mounted) {
        setState(() {
          _crosswalkDetected = true;
          _isCrosswalkMode = true;
          _trafficLight = 'RED';
        });
      }

      _resetMotorCommandCache();

      // "정지"가 먼저 들려야 하므로 횡단보도 안내와 합쳐서 한 번에 말한다.
      if (oldLight != 'RED') {
        _announce(
          oldCrosswalk
              ? '빨간불입니다. 멈춰서 기다려 주세요.'
              : '멈추세요. 앞에 횡단보도가 있고 빨간불입니다.',
          priority: _SpeechPriority.urgent,
          key: 'light',
          maxAge: const Duration(seconds: 6),
          vibrate: true,
        );
        _dropQueuedSpeech((r) => r.key == 'crosswalk');
      } else if (!oldCrosswalk) {
        _announceCrosswalkAhead();
      }

      debugPrint('[TRAFFIC] RED');

      return;
    }

    // ============================================================
    // 7. 초록불
    //
    // LIGHT:GREEN
    // ============================================================
    if (message == 'LIGHT:GREEN' ||
        message == 'SIGNAL_GREEN') {
      final String oldLight = _trafficLight;
      final bool oldCrosswalk = _crosswalkDetected;

      if (mounted) {
        setState(() {
          _crosswalkDetected = true;
          _isCrosswalkMode = true;
          _trafficLight = 'GREEN';
        });
      }

      _resetMotorCommandCache();

      if (oldLight != 'GREEN') {
        _announce(
          oldCrosswalk
              ? '초록불입니다. 지팡이를 따라 건너 주세요.'
              : '앞에 횡단보도가 있고 초록불입니다. 지팡이를 따라 건너 주세요.',
          priority: _SpeechPriority.important,
          key: 'light',
          maxAge: const Duration(seconds: 6),
        );
        _dropQueuedSpeech((r) => r.key == 'crosswalk');
      } else if (!oldCrosswalk) {
        _announceCrosswalkAhead();
      }

      debugPrint('[TRAFFIC] GREEN');

      return;
    }

    // ============================================================
    // 7-1. 초록불이 이미 켜져 있음 (Raspberry Pi가 빨간불을 못 봄)
    //
    // LIGHT:GREEN_WAIT
    // 남은 시간을 모르므로 건너지 않고 다음 신호를 기다린다.
    // ============================================================
    if (message == 'LIGHT:GREEN_WAIT') {
      // 건너는 중에 "다음 신호를 기다리라"는 안내는 위험하다. 무시한다.
      // (횡단 종료 직후 유예는 ESP32가 판단해서 아예 보내지 않는다)
      if (_isCrossing) {
        debugPrint('[TRAFFIC] GREEN_WAIT during crossing -> ignored');
        return;
      }

      final String oldLight = _trafficLight;
      final bool oldCrosswalk = _crosswalkDetected;

      if (mounted) {
        setState(() {
          _crosswalkDetected = true;
          _isCrosswalkMode = true;
          _trafficLight = 'GREEN_WAIT';
        });
      }

      _resetMotorCommandCache();

      if (oldLight != 'GREEN_WAIT') {
        _announce(
          '${oldCrosswalk ? '' : '앞에 횡단보도가 있습니다. '}'
              '초록불이 이미 켜져 있어 남은 시간을 알 수 없습니다. '
              '멈춰서 다음 초록불을 기다려 주세요.',
          priority: _SpeechPriority.urgent,
          key: 'light',
          maxAge: const Duration(seconds: 6),
          vibrate: true,
        );
        _dropQueuedSpeech((r) => r.key == 'crosswalk');
      } else if (!oldCrosswalk) {
        _announceCrosswalkAhead();
      }

      debugPrint('[TRAFFIC] GREEN_WAIT');

      return;
    }

    // ============================================================
    // 8. 신호등 미인식
    //
    // LIGHT:NONE
    // ============================================================
    if (message == 'LIGHT:NONE') {
      // 일시적인 미인식에서는 직전 RED/GREEN 상태를 유지한다.
      // 그래야 한 프레임 미인식 후 같은 색을 다시 인식해도
      // 동일 음성 안내가 반복되지 않는다.
      debugPrint('[TRAFFIC] NONE - keep previous state: $_trafficLight');
      return;
    }

    // ============================================================
    // 9. 횡단 시작
    //
    // CROSSING_START
    // ============================================================
    if (message == 'CROSSING_START') {
      _isCrossing = true;
      _crossingRedAnnounced = false;

      if (mounted) {
        setState(() {
          _crosswalkDetected = true;
          _isCrosswalkMode = true;
        });
      }

      _resetMotorCommandCache();

      debugPrint('[MODE] CROSSING START');

      return;
    }

    // ============================================================
    // 10. 횡단 종료
    //
    // CROSSING_END
    //
    // 다시 Flutter 일반 TMAP 유도로 복귀
    // ============================================================
    if (message == 'CROSSING_END') {
      final bool wasCrossing = _isCrossing;

      _endCrossing();

      // 횡단 중 안내가 아직 밀려 있으면 버린다.
      _dropQueuedSpeech((r) => r.key == 'light' || r.key == 'crosswalk');

      if (wasCrossing || _crosswalkDetected) {
        _announce(
          '횡단보도를 다 건넜습니다.',
          priority: _SpeechPriority.important,
          key: 'crosswalk',
          maxAge: const Duration(seconds: 6),
        );
      }

      if (mounted) {
        setState(() {
          _isCrosswalkMode = false;
          _crosswalkDetected = false;
          _trafficLight = 'NONE';
          _crosswalkSteering = 'NONE';
        });
      }

      _resetMotorCommandCache();

      debugPrint(
        '[MODE] CROSSING END -> Flutter navigation restored',
      );

      // 시연 1단계(횡단보도까지 직진)는 횡단을 마치면 회전 단계로 넘어간다.
      if (_isDemoMode && _demoStep == 1) {
        unawaited(_setDemoStep(2));
      }

      return;
    }
    // ============================================================
    // 11. 횡단보도 주행 방향 보정
    //
    // Raspberry Pi
    //   ↓ UART
    // ESP32
    //   ↓ BLE
    // Flutter
    //
    // CROSS_MOTOR:L
    // CROSS_MOTOR:R
    // CROSS_MOTOR:CENTER
    // ============================================================

    if (message == 'CROSS_MOTOR:L') {
      if (mounted) {
        setState(() {
          _crosswalkSteering = 'LEFT';
        });
      }

      debugPrint('[CROSS MOTOR] LEFT');

      return;
    }

    if (message == 'CROSS_MOTOR:R') {
      if (mounted) {
        setState(() {
          _crosswalkSteering = 'RIGHT';
        });
      }

      debugPrint('[CROSS MOTOR] RIGHT');

      return;
    }

    if (message == 'CROSS_MOTOR:CENTER') {
      if (mounted) {
        setState(() {
          _crosswalkSteering = 'CENTER';
        });
      }

      debugPrint('[CROSS MOTOR] CENTER');

      return;
    }
    // ============================================================
    // 12. 모터 동작 확인
    //
    // MOTOR:L
    // MOTOR:R
    // MOTOR:STOP
    // ============================================================
    if (message.startsWith('MOTOR:')) {
      debugPrint('[MOTOR STATUS] $message');
      return;
    }

    debugPrint('[ESP32] Unknown message: $message');
  }

  void _applyCaneState(String state) {
    if (state.length < 7) {
      debugPrint('[BLE] bad ST: $state');
      return;
    }

    _caneStateReceived = true;

    final bool obstacle = state[0] == '1';
    final String avoid = switch (state[1]) {
      'L' => 'LEFT',
      'R' => 'RIGHT',
      'B' => 'BLOCKED',
      _ => 'NONE',
    };
    final bool crosswalk = state[2] == '1';
    final String light = switch (state[3]) {
      'R' => 'RED',
      'G' => 'GREEN',
      'W' => 'GREEN_WAIT',
      _ => 'NONE',
    };
    final bool crossing = state[4] == '1';
    final bool frontLost = state[5] == '0';

    debugPrint('[BLE] cane state restored: $state');

    if (mounted) {
      setState(() {
        _obstacleConfirmed = obstacle;
        _avoidDirection = obstacle ? avoid : 'NONE';
        _isCrosswalkMode = crosswalk || crossing;
        _crosswalkDetected = crosswalk || crossing;
        _trafficLight = light;
        _isCrossing = crossing;
        _frontSensorLost = frontLost;
      });
    }

    _resetMotorCommandCache();

    // 지금 사용자가 알아야 하는 것만 한 번에 말한다. (위험한 것 먼저)
    String? text;

    if (frontLost) {
      text = '앞 장애물 센서가 응답하지 않습니다. 지팡이로 앞을 확인하며 천천히 걸어 주세요.';
    } else if (obstacle) {
      text = avoid == 'LEFT'
          ? '앞에 장애물이 있습니다. 왼쪽으로 피해 주세요.'
          : avoid == 'RIGHT'
          ? '앞에 장애물이 있습니다. 오른쪽으로 피해 주세요.'
          : '앞에 장애물이 있습니다. 멈춰서 주변을 확인해 주세요.';
    } else if (crossing) {
      text = '지금은 횡단보도를 건너는 중입니다. 지팡이를 따라 끝까지 건너 주세요.';
    } else if (crosswalk && (light == 'RED' || light == 'GREEN_WAIT')) {
      text = '앞에 횡단보도가 있고 신호를 기다리는 중입니다. 멈춰서 기다려 주세요.';
    } else if (crosswalk) {
      text = '앞에 횡단보도가 있습니다.';
    }

    if (text != null) {
      _announce(
        text,
        priority: frontLost || obstacle || (crosswalk && light != 'GREEN')
            ? _SpeechPriority.urgent
            : _SpeechPriority.important,
        key: 'restore',
        maxAge: const Duration(seconds: 6),
        vibrate: frontLost || obstacle,
      );
    }
  }

  void _resetMotorCommandCache() {
    _lastMotorCommand = null;
    _lastMotorSentAt = null;
    _lastMotorFailedAt = null;

    // 모드가 바뀌면 좌/우 판단도 직진부터 다시 시작한다.
    _steeringCommand = 'F';
    _pendingSteeringCommand = null;
    _pendingSteeringSince = null;
  }

  void _announceCrosswalkAhead() {
    _announce(
      '앞에 횡단보도가 있습니다. 신호를 확인하는 동안 멈춰 주세요.',
      priority: _SpeechPriority.important,
      key: 'crosswalk',
      maxAge: const Duration(seconds: 6),
    );
  }

  // 횡단 종료 직후 유예 중인가.
  // "건너는 중"이라고 판단하는 데는 쓰지 않는다. (그건 _isCrossing만)
  // 횡단보도 재인식 안내를 잠깐 미루는 데만 쓴다.
  bool get _inCrossingGrace {
    if (_isCrossing) return false;

    final DateTime? endedAt = _crossingEndedAt;
    return endedAt != null && DateTime.now().difference(endedAt) < crossingGrace;
  }

  // 유예가 끝났을 때 아직 횡단보도 앞이고(횡단 시작 안 함) 신호 안내도 없었으면
  // 그때 "앞에 횡단보도가 있습니다"를 말한다.
  void _deferCrosswalkAnnouncement() {
    final DateTime? endedAt = _crossingEndedAt;
    if (endedAt == null) return;

    final Duration wait =
        crossingGrace - DateTime.now().difference(endedAt);

    _deferredCrosswalkTimer?.cancel();
    _deferredCrosswalkTimer = Timer(
      wait.isNegative ? Duration.zero : wait,
      () {
        if (!mounted) return;

        if (_crosswalkDetected && !_isCrossing && _trafficLight == 'NONE') {
          _announceCrosswalkAhead();
        }
      },
    );
  }

  void _endCrossing() {
    if (_isCrossing) {
      _crossingEndedAt = DateTime.now();
    }

    _isCrossing = false;
  }

  // ===========================================================================
  // ㄱ자 실내 시연 모드
  //
  // 실내에서는 GPS가 안 되므로 TMAP 경로 대신 "정면 직진 → 회전 → 직진"의
  // ㄱ자 경로를 방향으로 안내한다. 방향 유도는 일반 길 안내와 같은
  // 나침반 조향(_updateSteering)을 쓰고, 목표 방위각만 단계마다 바꾼다.
  // GPS 감시(_navInputWatchdog)는 켜지 않는다. (실내에서는 GPS가 없으므로)
  //
  // 단계 전환은 시간이 아니라 실제 이벤트로 한다.
  //  1 → 2: Raspberry Pi의 CROSSING_END (횡단 완료)
  //  2 → 3: 새 방향을 demoTurnHoldDuration 동안 유지
  //  3 → 4: 장애물을 만난 뒤 풀리고 demoArriveAfterObstacle 경과
  // 횡단보도/장애물 처리 중에는 기존 코드가 조향을 멈추므로 단계도 기다린다.
  // 보호자 화면의 '다음 단계' 버튼으로 언제든 강제로 넘길 수 있다.
  // 사용자 화면에서 두 번 탭하면 (= 안내 중지) 시연을 끝낸다.
  // ===========================================================================

  Future<void> _startDemo() async {
    final double? heading = _phoneHeading;

    if (heading == null) {
      _announce(
        '나침반을 준비하는 중입니다. 잠시 후 다시 눌러 주세요.',
        priority: _SpeechPriority.important,
        key: 'demo',
      );
      return;
    }

    _listenRetryTimer?.cancel();
    _resumeListeningAfterSpeech = false;

    if (_speech.isListening) {
      try {
        await _speech.cancel();
      } catch (_) {}
    }

    if (!mounted) return;

    setState(() {
      _isDemoMode = true;
      _demoStep = 1;
      _demoFrontBearing = heading;
      _demoTurnHeldSince = null;
      _demoObstacleSeen = false;
      _isNavigationActive = true;
      _navInputProblem = null;
      _targetBearing = heading;
      _steeringError = null;
      _isListening = false;
      _speechStatus = 'ㄱ자 시연 중';
    });

    _resetMotorCommandCache();

    _demoTimer?.cancel();
    _demoTimer = Timer.periodic(
      const Duration(milliseconds: 300),
      (_) => _demoTick(),
    );

    _announce(
      '시연을 시작합니다. 앞으로 직진하세요.',
      priority: _SpeechPriority.important,
      key: 'demo',
    );

    await _applyDemoTarget();
  }

  Future<void> _setDemoStep(int step) async {
    if (!mounted || !_isDemoMode || step <= _demoStep) return;

    if (step >= 4) {
      await _finishDemo();
      return;
    }

    final double front = _demoFrontBearing ?? _phoneHeading ?? 0.0;
    final double turned =
        (front + (_demoTurnRight ? 90.0 : -90.0) + 360.0) % 360.0;

    setState(() {
      _demoStep = step;
      _demoTurnHeldSince = null;
      _demoObstacleSeen = false;
      _targetBearing = step == 1 ? front : turned;
    });

    _resetMotorCommandCache();

    if (step == 2 || step == 3) {
      _announce(
        _demoStepSpeech(),
        priority: _SpeechPriority.important,
        key: 'demo',
      );
    }

    await _applyDemoTarget();
  }

  // 지금 단계에서 해야 할 일 (단계가 바뀔 때 / '지금 상황' 요청 때 읽는다)
  String _demoStepSpeech() {
    switch (_demoStep) {
      case 1:
        return '실내 시연 중입니다. 앞으로 직진하세요.';
      case 2:
        return _demoTurnRight ? '오른쪽으로 도세요.' : '왼쪽으로 도세요.';
      case 3:
        return '앞으로 직진하세요.';
      default:
        return '시연을 마쳤습니다.';
    }
  }

  // 바뀐 목표 방위각을 ESP32에 알리고 직진/좌/우 판단을 바로 다시 한다.
  Future<void> _applyDemoTarget() async {
    final double? target = _targetBearing;

    if (target == null) return;

    await _sendBearingToCaneIfNeeded(target);
    await _updateSteering();
  }

  void _demoTick() {
    if (!mounted || !_isDemoMode) return;

    // 횡단보도는 Raspberry Pi가 제어하는 동안 단계를 넘기지 않는다.
    if (_isCrosswalkMode) {
      _demoTurnHeldSince = null;
      return;
    }

    if (_demoStep == 2) {
      final double? target = _targetBearing;
      final double? heading = _phoneHeading;

      if (target == null ||
          heading == null ||
          _obstacleConfirmed ||
          _angleDifference(target, heading) > demoTurnDoneDegrees) {
        _demoTurnHeldSince = null;
        return;
      }

      final DateTime now = DateTime.now();
      final DateTime heldSince = _demoTurnHeldSince ??= now;

      if (now.difference(heldSince) >= demoTurnHoldDuration) {
        unawaited(_setDemoStep(3));
      }
      return;
    }

    if (_demoStep == 3) {
      if (_obstacleConfirmed) {
        _demoObstacleSeen = true;
        return;
      }

      final DateTime? clearedAt = _obstacleClearedAt;

      if (_demoObstacleSeen &&
          clearedAt != null &&
          DateTime.now().difference(clearedAt) >= demoArriveAfterObstacle) {
        unawaited(_setDemoStep(4));
      }
    }
  }

  Future<void> _finishDemo() async {
    _demoTimer?.cancel();
    _demoTimer = null;

    if (mounted) {
      setState(() {
        _demoStep = 4;
        _isNavigationActive = false;
        _targetBearing = null;
        _steeringError = null;
        _speechStatus = '목적지에 도착했습니다.';
      });
    }

    await _sendStopToCane();
    await _speak('목적지에 도착했습니다. 안내를 종료합니다.');
  }

  Future<void> _exitDemoMode() async {
    final bool wasGuiding = _isNavigationActive;

    _demoTimer?.cancel();
    _demoTimer = null;

    if (mounted) {
      setState(() {
        _isDemoMode = false;
        _demoStep = 0;
        _demoFrontBearing = null;
        _demoTurnHeldSince = null;
        _demoObstacleSeen = false;
        _isNavigationActive = false;
        _targetBearing = null;
        _steeringError = null;
        _speechStatus = '목적지를 말씀해주세요.';
      });
    }

    _dropQueuedSpeech((r) => r.key == 'demo');

    if (wasGuiding) {
      await _sendStopToCane();
    } else {
      _resetMotorCommandCache();
    }

    await _speak('시연을 종료합니다.');
  }

  // ===========================================================================
  // UI
  // ===========================================================================

  Widget _sectionCard({
    required Widget child,
  }) {
    return Card(
      elevation: 0,
      margin: const EdgeInsets.only(bottom: 12),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(18),
        side: BorderSide(
          color: Colors.grey.shade300,
        ),
      ),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: child,
      ),
    );
  }

  Widget _demoCard() {
    final List<String> stepTexts = [
      '출발점에서 횡단보도 쪽을 보고 시작하세요',
      '1/4 · 직진 (횡단보도까지)',
      _demoTurnRight ? '2/4 · 오른쪽으로 회전' : '2/4 · 왼쪽으로 회전',
      '3/4 · 직진 (장애물 구간)',
      '4/4 · 도착',
    ];

    // 실제 TMAP 길 안내 중에는 시연을 시작하지 않는다.
    final bool canStart = !_isSearchingRoute && !_isNavigationActive;
    final bool canSkip = _demoStep >= 1 && _demoStep <= 3;

    return _sectionCard(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          const Text(
            'ㄱ자 실내 시연',
            textAlign: TextAlign.center,
            style: TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.bold,
            ),
          ),
          if (_isDemoMode) ...[
            const SizedBox(height: 12),
            ExcludeSemantics(
              child: SizedBox(
                height: 190,
                child: CustomPaint(
                  painter: _DemoCoursePainter(
                    step: _demoStep,
                    turnRight: _demoTurnRight,
                    atCrosswalk: _isCrosswalkMode,
                    atObstacle: _obstacleConfirmed,
                  ),
                ),
              ),
            ),
          ],
          const SizedBox(height: 10),
          Text(
            stepTexts[_isDemoMode ? _demoStep : 0],
            textAlign: TextAlign.center,
            style: TextStyle(
              fontSize: _isDemoMode ? 21 : 15,
              fontWeight: FontWeight.bold,
            ),
          ),
          if (_demoFrontBearing != null) ...[
            const SizedBox(height: 4),
            Text(
              '정면 기준 ${_demoFrontBearing!.toStringAsFixed(0)}°',
              textAlign: TextAlign.center,
              style: TextStyle(
                fontSize: 13,
                color: Colors.grey.shade700,
              ),
            ),
          ],
          const SizedBox(height: 12),
          if (!_isDemoMode) ...[
            SegmentedButton<bool>(
              segments: const [
                ButtonSegment<bool>(
                  value: false,
                  label: Text('왼쪽 회전'),
                ),
                ButtonSegment<bool>(
                  value: true,
                  label: Text('오른쪽 회전'),
                ),
              ],
              selected: {_demoTurnRight},
              onSelectionChanged: (Set<bool> selected) {
                setState(() {
                  _demoTurnRight = selected.first;
                });
              },
            ),
            const SizedBox(height: 8),
            FilledButton.icon(
              style: _bigButtonStyle(),
              onPressed: canStart ? _startDemo : null,
              icon: const Icon(Icons.play_arrow, size: 30),
              label: const Text('지금 방향으로 시연 시작'),
            ),
          ] else
            Row(
              children: [
                Expanded(
                  child: OutlinedButton.icon(
                    onPressed:
                    canSkip ? () => _setDemoStep(_demoStep + 1) : null,
                    icon: const Icon(Icons.skip_next),
                    label: const Text('다음 단계'),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  child: OutlinedButton.icon(
                    onPressed: _exitDemoMode,
                    icon: const Icon(Icons.close),
                    label: const Text('시연 종료'),
                  ),
                ),
              ],
            ),
        ],
      ),
    );
  }

  String _crosswalkText() {
    return _crosswalkDetected
        ? '횡단보도 인식함'
        : '횡단보도 대기 중';
  }

  String _trafficLightText() {
    if (_trafficLight == 'RED') {
      return '빨간불 인식함';
    }

    if (_trafficLight == 'GREEN') {
      return '초록불 인식함';
    }

    if (_trafficLight == 'GREEN_WAIT') {
      return '초록불 진행 중 · 다음 신호 대기';
    }

    if (_trafficLight == 'CROSSING_RED') {
      return '횡단 중 빨간불 · 계속 건너기';
    }

    return '신호등 대기 중';
  }

  Color _trafficLightColor() {
    if (_trafficLight == 'CROSSING_RED') return Colors.deepOrange;
    if (_trafficLight == 'RED') return Colors.red;
    if (_trafficLight == 'GREEN') return Colors.green;
    if (_trafficLight == 'GREEN_WAIT') return Colors.orange;
    return Colors.grey;
  }

  Widget _distanceSensorBox({
    required String title,
    required IconData icon,
    required double? distanceCm,
    required double warningDistanceCm,
  }) {
    final bool warning = distanceCm != null &&
        distanceCm > 0 &&
        distanceCm <= warningDistanceCm;

    // 스크린리더용: "VL53L1X" 같은 부품 이름은 빼고 방향과 거리만 읽는다.
    final String place = title.split('\n').first;
    final String spokenDistance = distanceCm == null
        ? '측정 안 됨'
        : '${distanceCm.toStringAsFixed(0)}센티미터${warning ? ', 가까움' : ''}';

    return Expanded(
      child: Semantics(
        label: '$place 거리 $spokenDistance',
        excludeSemantics: true,
        child: Container(
        padding: const EdgeInsets.symmetric(
          horizontal: 8,
          vertical: 12,
        ),
        decoration: BoxDecoration(
          color: warning
              ? Colors.red.shade50
              : Colors.grey.shade100,
          borderRadius: BorderRadius.circular(12),
          border: Border.all(
            color: warning
                ? Colors.red.shade300
                : Colors.grey.shade300,
          ),
        ),
        child: Column(
          children: [
            Icon(
              icon,
              size: 26,
              color: warning ? Colors.red : Colors.deepPurple,
            ),
            const SizedBox(height: 5),
            Text(
              title,
              textAlign: TextAlign.center,
              style: const TextStyle(
                fontSize: 13,
                fontWeight: FontWeight.bold,
              ),
            ),
            const SizedBox(height: 8),
            FittedBox(
              fit: BoxFit.scaleDown,
              child: Text(
                distanceCm == null
                    ? '-- cm'
                    : '${distanceCm.toStringAsFixed(0)} cm',
                style: TextStyle(
                  fontSize: 27,
                  fontWeight: FontWeight.bold,
                  color: warning ? Colors.red : Colors.black87,
                ),
              ),
            ),
          ],
        ),
      ),
      ),
    );
  }

  ButtonStyle _bigButtonStyle({bool outlined = false}) {
    const Size size = Size.fromHeight(68);
    const TextStyle text = TextStyle(fontSize: 22, fontWeight: FontWeight.bold);

    return outlined
        ? OutlinedButton.styleFrom(
      minimumSize: size,
      textStyle: text,
      side: const BorderSide(width: 2),
    )
        : FilledButton.styleFrom(
      minimumSize: size,
      textStyle: text,
    );
  }

  Future<void> _onTalkButtonPressed() async {
    unawaited(HapticFeedback.mediumImpact());

    if (_isSearchingRoute) {
      await _speak('경로를 찾는 중입니다. 잠시만 기다려 주세요.');
      return;
    }

    if (_isDemoMode) {
      await _speak('시연 중에는 말하기를 쓸 수 없습니다. 두 번 누르면 시연을 끝냅니다.');
      return;
    }

    if (_isListening || _speech.isListening) return;

    // 밀려 있던 참고 안내(남은 거리 등)는 버리고 바로 듣는다.
    _dropQueuedSpeech((r) => r.priority == _SpeechPriority.info);

    await _speak(
      _isNavigationActive
          ? '말씀해 주세요. 상황, 또는 안내 중지라고 말할 수 있습니다.'
          : '목적지를 말씀해 주세요.',
    );

    if (!mounted) return;

    await _startListening();
  }

  // 지금 가장 중요한 상태 하나 (위험한 것부터)
  ({String text, Color color, IconData icon}) _statusBannerInfo() {
    if (!_isBleConnected) {
      return (
      text: _isConnecting ? '지팡이 연결 중' : '지팡이 연결 안 됨',
      color: Colors.red.shade800,
      icon: Icons.bluetooth_disabled,
      );
    }

    if (_frontSensorLost) {
      return (
      text: '앞 센서 응답 없음 · 천천히',
      color: Colors.red.shade700,
      icon: Icons.sensors_off,
      );
    }

    if (_obstacleConfirmed) {
      return (
      text: _avoidDirection == 'BLOCKED'
          ? '앞이 막힘 · 멈추세요'
          : _avoidDirection == 'LEFT'
          ? '장애물 · 왼쪽으로 피하기'
          : _avoidDirection == 'RIGHT'
          ? '장애물 · 오른쪽으로 피하기'
          : '장애물 · 멈추세요',
      color: Colors.deepOrange.shade800,
      icon: Icons.warning_rounded,
      );
    }

    if (_isCrossing) {
      return (
      text: _trafficLight == 'CROSSING_RED'
          ? '횡단 중 · 끝까지 건너세요'
          : '횡단보도 건너는 중',
      color: Colors.deepPurple.shade700,
      icon: Icons.directions_walk,
      );
    }

    if (_isCrosswalkMode) {
      final bool wait =
          _trafficLight == 'RED' || _trafficLight == 'GREEN_WAIT';

      return (
      text: wait
          ? '횡단보도 · 멈춰서 기다리기'
          : _trafficLight == 'GREEN'
          ? '초록불 · 건너기'
          : '횡단보도 앞 · 신호 확인 중',
      color: wait ? Colors.red.shade800 : Colors.green.shade800,
      icon: Icons.traffic,
      );
    }

    if (_isDemoMode) {
      return (
      text: _demoStep >= 4 ? '시연 · 도착' : '실내 시연 중 · $_demoStep/4',
      color: Colors.blue.shade800,
      icon: Icons.turn_right,
      );
    }

    if (_isNavigationActive) {
      final double? remaining = _remainingRouteMeters;

      return (
      text: remaining == null
          ? '길 안내 중'
          : '길 안내 중 · ${_spokenDistance(remaining)} 남음',
      color: Colors.blue.shade800,
      icon: Icons.navigation,
      );
    }

    return (
    text: '목적지를 말씀해 주세요',
    color: Colors.grey.shade800,
    icon: Icons.mic_none,
    );
  }

  // large: 사용자 화면용 (더 크게, 검은 배경에서 잘 보이도록 흰 테두리)
  Widget _statusBanner({bool large = false}) {
    final info = _statusBannerInfo();

    return Semantics(
      header: true,
      liveRegion: large,
      label: '현재 상태: ${info.text}',
      excludeSemantics: true,
      child: Container(
        width: double.infinity,
        padding: EdgeInsets.symmetric(horizontal: 16, vertical: large ? 22 : 18),
        decoration: BoxDecoration(
          color: info.color,
          borderRadius: BorderRadius.circular(18),
          border: large ? Border.all(color: Colors.white, width: 3) : null,
        ),
        child: Row(
          children: [
            Icon(info.icon, color: Colors.white, size: large ? 52 : 40),
            const SizedBox(width: 14),
            Expanded(
              child: Text(
                info.text,
                style: TextStyle(
                  color: Colors.white,
                  fontSize: large ? 32 : 26,
                  fontWeight: FontWeight.bold,
                  height: 1.2,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  // ===========================================================================
  // 화면 전환
  // ===========================================================================

  @override
  Widget build(BuildContext context) {
    return _isGuardianView ? _buildGuardianScaffold() : _buildUserScaffold();
  }

  void _setGuardianView(bool guardian) {
    unawaited(HapticFeedback.mediumImpact());

    setState(() {
      _isGuardianView = guardian;
    });

    _announce(
      guardian
          ? '보호자 화면입니다. 화면 맨 위 오른쪽 버튼을 누르면 사용자 화면으로 돌아갑니다.'
          : '사용자 화면입니다.',
      priority: _SpeechPriority.important,
      key: 'view',
      maxAge: const Duration(seconds: 5),
    );
  }

  // ===========================================================================
  // 사용자(시각장애인) 화면 - 버튼 없이 화면 전체가 하나의 터치 영역
  //
  //  한 번 탭     : 지금 상황 듣기
  //  길게 누르기  : 말하기 (목적지 / 명령)
  //  두 번 탭     : 길 안내 중지
  //
  // 버튼 위치를 찾을 필요가 없도록 화면 어디를 눌러도 된다.
  // 동작마다 진동 세기를 다르게 해서, 소리 없이도 무엇이 눌렸는지 알 수 있다.
  // 보호자 화면은 맨 위 오른쪽 글자 버튼으로만 간다. (사용자가 실수로 가지 않게)
  //
  // 스크린리더(TalkBack / VoiceOver)를 켜면 화면 터치를 스크린리더가 가져가므로
  // 위 제스처 대신 스크린리더의 기본 동작으로 쓴다.
  //  두 번 탭                 : 지금 상황 듣기
  //  두 번 탭 후 길게 누르기  : 말하기
  //  동작 메뉴(위/아래 쓸기)  : 말하기 / 길 안내 중지 / 사용법
  // ===========================================================================

  void _onUserTap() {
    _tapCount++;
    _tapTimer?.cancel();

    // 두 번째 탭은 더 기다릴 필요가 없다.
    if (_tapCount >= 2) {
      _tapCount = 0;
      _handleTapGesture(2);
      return;
    }

    _tapTimer = Timer(multiTapWindow, () {
      _tapCount = 0;
      _handleTapGesture(1);
    });
  }

  void _handleTapGesture(int count) {
    if (count == 1) {
      unawaited(HapticFeedback.selectionClick());
      _showGestureFeedback('지금 상황');
      unawaited(_speakCurrentGuidance());
      return;
    }

    unawaited(HapticFeedback.heavyImpact());

    // 시연은 도착한 뒤에도 두 번 탭으로 끝낼 수 있게 한다.
    if (_isNavigationActive || _isDemoMode) {
      _showGestureFeedback('안내 중지');
      unawaited(_stopNavigationByUser());
    } else {
      _showGestureFeedback('안내 중 아님');
      _announce(
        '지금은 길 안내 중이 아닙니다. 길게 누르고 목적지를 말씀하세요.',
        priority: _SpeechPriority.important,
        key: 'gesture',
        maxAge: const Duration(seconds: 3),
      );
    }
  }

  void _onUserLongPress() {
    _tapTimer?.cancel();
    _tapCount = 0;

    // _onTalkButtonPressed()가 진동을 준다.
    _showGestureFeedback('말하기');
    unawaited(_onTalkButtonPressed());
  }

  // 저시력 사용자 / 옆에 있는 보호자가 방금 무엇이 눌렸는지 볼 수 있게 잠깐 표시
  void _showGestureFeedback(String text) {
    _gestureFeedbackTimer?.cancel();

    setState(() {
      _gestureFeedback = text;
    });

    _gestureFeedbackTimer = Timer(const Duration(milliseconds: 1500), () {
      if (!mounted) return;
      setState(() {
        _gestureFeedback = null;
      });
    });
  }

  // 검은 배경에서 잘 보이도록 상태 색을 밝게
  Color _brightStatusColor(Color color) {
    final HSLColor hsl = HSLColor.fromColor(color);

    if (hsl.saturation < 0.15) {
      return Colors.white70;
    }

    return hsl
        .withLightness(0.62)
        .withSaturation(math.max(hsl.saturation, 0.85))
        .toColor();
  }

  // 카드 큰 제목 (길 안내 중이면 남은 거리는 아래 줄에 따로 쓴다)
  String _userTitleText(({String text, Color color, IconData icon}) info) {
    if (_isListening) return '듣고 있어요';
    if (info.icon == Icons.navigation) return '길 안내 중';
    return info.text;
  }

  // 카드 아래쪽 할 일 안내 한 줄
  String? _userHintText() {
    if (_isListening) return '목적지나 명령을 말씀하세요';
    if (!_isBleConnected) {
      return _isConnecting ? '잠시만 기다려 주세요' : '보호자 화면에서 지팡이를 연결해 주세요';
    }
    if (_frontSensorLost) return '지팡이 앞쪽 센서를 확인해 주세요';
    if (_obstacleConfirmed) return '진동 방향을 따라 천천히 움직이세요';
    if (_isCrossing) return '멈추지 말고 끝까지 건너세요';
    if (_isCrosswalkMode) return '신호가 바뀌면 알려 드릴게요';
    if (!_isNavigationActive) return '길게 누르고 목적지를 말씀하세요';
    return null;
  }

  Widget _buildUserScaffold() {
    final info = _statusBannerInfo();
    final Color accent = _brightStatusColor(info.color);

    final String destinationText = _recognizedDestinationText.isNotEmpty
        ? _recognizedDestinationText
        : _destination?.name ?? '';

    final double? remaining = _remainingRouteMeters;
    final String title = _userTitleText(info);
    final String? hint = _gestureFeedback ?? _userHintText();

    return Scaffold(
      backgroundColor: Colors.black,
      body: SafeArea(
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            _userTopBar(info, accent),
            Expanded(
              child: Semantics(
                // 카드 전체가 하나의 요소. 상태가 바뀌면 스크린리더가 다시 읽는다.
                container: true,
                liveRegion: true,
                excludeSemantics: true,
                label: [
                  title,
                  if (destinationText.isNotEmpty) '목적지 $destinationText',
                  if (remaining != null && _isNavigationActive)
                    '남은 거리 약 ${_spokenDistance(remaining)}',
                  if (hint != null) hint,
                ].join('. '),
                hint: '두 번 누르면 지금 상황 듣기. 동작 메뉴에서 말하기와 안내 중지를 고를 수 있습니다.',
                onTap: () => _handleTapGesture(1),
                onLongPress: _onUserLongPress,
                customSemanticsActions: {
                  const CustomSemanticsAction(label: '말하기'): _onUserLongPress,
                  if (_isNavigationActive)
                    const CustomSemanticsAction(label: '길 안내 중지'): () =>
                        _handleTapGesture(2),
                  const CustomSemanticsAction(label: '사용법 듣기'): () =>
                      unawaited(_speak(gestureGuideText)),
                },
                child: RawGestureDetector(
                  behavior: HitTestBehavior.opaque,
                  gestures: {
                    TapGestureRecognizer:
                    GestureRecognizerFactoryWithHandlers<TapGestureRecognizer>(
                      TapGestureRecognizer.new,
                          (recognizer) => recognizer.onTapUp = (_) => _onUserTap(),
                    ),
                    LongPressGestureRecognizer:
                    GestureRecognizerFactoryWithHandlers<LongPressGestureRecognizer>(
                          () => LongPressGestureRecognizer(duration: longPressDuration),
                          (recognizer) => recognizer.onLongPress = _onUserLongPress,
                    ),
                  },
                  child: Padding(
                    padding: const EdgeInsets.fromLTRB(16, 8, 16, 16),
                    child: _userCard(
                      accent: accent,
                      title: title,
                      destinationText: destinationText,
                      remaining: _isNavigationActive ? remaining : null,
                      hint: hint,
                      hintIsFeedback: _gestureFeedback != null,
                    ),
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  // 맨 위: 지금 상태 한 줄 + 보호자 화면 (작은 글자 버튼)
  Widget _userTopBar(({String text, Color color, IconData icon}) info, Color accent) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(20, 8, 8, 8),
      child: Row(
        children: [
          ExcludeSemantics(
            child: Icon(info.icon, color: accent, size: 30),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: ExcludeSemantics(
              child: Text(
                info.text,
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
                style: TextStyle(
                  color: accent,
                  fontSize: 22,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
          ),
          Semantics(
            button: true,
            label: '보호자 화면으로 바꾸기',
            excludeSemantics: true,
            child: TextButton(
              onPressed: () => _setGuardianView(true),
              style: TextButton.styleFrom(
                foregroundColor: Colors.white70,
                textStyle: const TextStyle(fontSize: 18),
              ),
              child: const Text('보호자 화면'),
            ),
          ),
        ],
      ),
    );
  }

  Widget _userCard({
    required Color accent,
    required String title,
    required String destinationText,
    required double? remaining,
    required String? hint,
    required bool hintIsFeedback,
  }) {
    const TextStyle bodyStyle = TextStyle(
      color: Colors.white,
      fontSize: 28,
      fontWeight: FontWeight.w600,
      height: 1.3,
    );

    return Container(
      decoration: BoxDecoration(
        color: const Color(0xFF141414),
        borderRadius: BorderRadius.circular(32),
        border: Border.all(color: accent, width: 5),
      ),
      padding: const EdgeInsets.fromLTRB(24, 28, 24, 20),
      child: Column(
        children: [
          Expanded(
            child: SingleChildScrollView(
              physics: const ClampingScrollPhysics(),
              child: Column(
                children: [
                  Icon(
                    _isListening ? Icons.hearing : Icons.record_voice_over,
                    size: 96,
                    color: accent,
                  ),
                  const SizedBox(height: 16),
                  Text(
                    title,
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      color: accent,
                      fontSize: 46,
                      fontWeight: FontWeight.w900,
                      height: 1.2,
                    ),
                  ),
                  if (destinationText.isNotEmpty) ...[
                    const SizedBox(height: 20),
                    Text(
                      '목적지 $destinationText',
                      textAlign: TextAlign.center,
                      style: bodyStyle,
                    ),
                  ],
                  if (remaining != null) ...[
                    const SizedBox(height: 12),
                    Text(
                      '남은 거리 약 ${_spokenDistance(remaining)}',
                      textAlign: TextAlign.center,
                      style: bodyStyle,
                    ),
                  ],
                  if (hint != null) ...[
                    const SizedBox(height: 12),
                    Text(
                      hint,
                      textAlign: TextAlign.center,
                      style: hintIsFeedback
                          ? bodyStyle.copyWith(
                        color: const Color(0xFFFFD600),
                        fontWeight: FontWeight.w900,
                      )
                          : bodyStyle,
                    ),
                  ],
                ],
              ),
            ),
          ),
          const Divider(color: Colors.white24, thickness: 1, height: 28),
          _gestureGuide(),
        ],
      ),
    );
  }

  // 카드 아래 사용법 (저시력 사용자 / 보호자용)
  Widget _gestureGuide() {
    const TextStyle style = TextStyle(
      color: Colors.white60,
      fontSize: 20,
      height: 1.5,
    );

    return Column(
      children: [
        const Text('한 번 탭: 상태 듣기', style: style),
        Text(
          _isNavigationActive ? '길게 누르기: 말하기' : '길게 누르기: 목적지 말하기',
          style: style,
        ),
        Text(
          '두 번 탭: 안내 중지',
          style: _isNavigationActive
              ? style
              : style.copyWith(color: Colors.white30),
        ),
      ],
    );
  }

  // ===========================================================================
  // 보호자 화면
  //
  // 기존 화면(지도, 경로, 센서 값, 라즈베리파이 인식 결과)에
  // "안전 상태" 요약을 맨 위에 더했다. 동작 확인 / 시연 / 보호자 확인용.
  // ===========================================================================

  Widget _guardianInfoRow(String label, String value, {Color? color}) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          Expanded(
            child: Text(
              label,
              style: TextStyle(color: Colors.grey.shade700),
            ),
          ),
          Text(
            value,
            style: TextStyle(
              fontWeight: FontWeight.bold,
              color: color ?? Colors.black87,
            ),
          ),
        ],
      ),
    );
  }

  Widget _guardianSafetyCard() {
    final double? accuracy = _lastFixAccuracy;
    final double? remaining = _remainingRouteMeters;

    return _sectionCard(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          const Text(
            '안전 상태',
            textAlign: TextAlign.center,
            style: TextStyle(fontWeight: FontWeight.bold),
          ),
          const SizedBox(height: 8),
          _guardianInfoRow(
            '지팡이 연결',
            _bleStatus,
            color: _isBleConnected ? Colors.green.shade700 : Colors.red,
          ),
          _guardianInfoRow(
            '정면 센서',
            _frontSensorLost
                ? '응답 없음'
                : _distanceCm == null
                ? '값 없음'
                : '정상',
            color: _frontSensorLost ? Colors.red : null,
          ),
          _guardianInfoRow(
            'GPS 정확도',
            accuracy == null
                ? '--'
                : '±${accuracy.toStringAsFixed(0)} m'
                '${_navInputProblem == 'gps' ? ' · 유도 멈춤' : ''}',
            color: _navInputProblem == 'gps' ? Colors.red : null,
          ),
          _guardianInfoRow(
            '나침반',
            _navInputProblem == 'compass'
                ? '값 없음 · 유도 멈춤'
                : _phoneHeading == null
                ? '--'
                : '정상',
            color: _navInputProblem == 'compass' ? Colors.red : null,
          ),
          _guardianInfoRow(
            '남은 거리 (경로)',
            remaining == null ? '--' : '${remaining.toStringAsFixed(0)} m',
          ),
          _guardianInfoRow(
            '마지막 모터 명령',
            _lastMotorCommand ?? '--',
          ),
        ],
      ),
    );
  }

  Widget _buildGuardianScaffold() {
    final bool obstacleDetected = _distanceCm != null &&
        _distanceCm! > 0 &&
        _distanceCm! <= obstacleWarningCm;

    return Scaffold(
      appBar: AppBar(
        title: const Text('Smart Cane · 보호자'),
        centerTitle: true,
        actions: [
          IconButton(
            tooltip: '사용자 화면으로 바꾸기',
            iconSize: 30,
            onPressed: () => _setGuardianView(false),
            icon: const Icon(Icons.accessibility_new),
          ),
        ],
      ),
      body: SafeArea(
        child: ListView(
          padding: const EdgeInsets.all(14),
          children: [
            // -----------------------------------------------------------------
            // 현재 상태 (가장 중요한 것 하나만 크게)
            // 저시력 사용자와 보호자가 한눈에 볼 수 있게
            // -----------------------------------------------------------------
            _statusBanner(),
            const SizedBox(height: 12),

            _guardianSafetyCard(),

            // -----------------------------------------------------------------
            // ㄱ자 실내 시연
            // -----------------------------------------------------------------
            _demoCard(),

            // -----------------------------------------------------------------
            // 목적지 / 음성 입력
            // -----------------------------------------------------------------
            _sectionCard(
              child: Column(
                children: [
                  const Text(
                    '목적지',
                    style: TextStyle(
                      fontSize: 18,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 10),
                  Icon(
                    _isListening ? Icons.mic : Icons.mic_none,
                    size: 48,
                    color:
                    _isListening ? Colors.red : Colors.deepPurple,
                  ),
                  const SizedBox(height: 8),
                  Text(
                    _speechStatus,
                    textAlign: TextAlign.center,
                    style: const TextStyle(
                      fontSize: 17,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                  const SizedBox(height: 10),
                  Container(
                    width: double.infinity,
                    padding: const EdgeInsets.symmetric(
                      horizontal: 14,
                      vertical: 13,
                    ),
                    decoration: BoxDecoration(
                      color: Colors.grey.shade100,
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: Text(
                      _recognizedDestinationText.isEmpty
                          ? '목적지를 말씀해주세요'
                          : _recognizedDestinationText,
                      textAlign: TextAlign.center,
                      style: TextStyle(
                        fontSize: _recognizedDestinationText.isEmpty
                            ? 17
                            : 24,
                        fontWeight: FontWeight.bold,
                        color: _recognizedDestinationText.isEmpty
                            ? Colors.grey.shade600
                            : Colors.black87,
                      ),
                    ),
                  ),
                  const SizedBox(height: 12),

                  // 화면을 보지 않고도 누를 수 있도록 크게, 항상 누를 수 있게 둔다.
                  // (안내 중이어도 누르면 그 안내가 끝난 뒤 바로 듣는다)
                  Semantics(
                    button: true,
                    label: _isNavigationActive
                        ? '말하기. 상황 듣기나 안내 중지를 말씀하세요.'
                        : '목적지 말하기',
                    excludeSemantics: true,
                    child: FilledButton.icon(
                      style: _bigButtonStyle(),
                      onPressed: _onTalkButtonPressed,
                      icon: Icon(
                        _isListening ? Icons.hearing : Icons.mic,
                        size: 32,
                      ),
                      label: Text(
                        _isListening
                            ? '듣고 있어요'
                            : _isNavigationActive
                            ? '말하기 (명령)'
                            : '목적지 말하기',
                      ),
                    ),
                  ),
                  const SizedBox(height: 10),
                  Semantics(
                    button: true,
                    label: '지금 상황 듣기',
                    excludeSemantics: true,
                    child: FilledButton.tonalIcon(
                      style: _bigButtonStyle(),
                      onPressed: () {
                        unawaited(HapticFeedback.mediumImpact());
                        unawaited(_speakCurrentGuidance());
                      },
                      icon: const Icon(Icons.record_voice_over, size: 30),
                      label: const Text('지금 상황 듣기'),
                    ),
                  ),
                  if (_isNavigationActive) ...[
                    const SizedBox(height: 10),
                    Semantics(
                      button: true,
                      label: '길 안내 중지',
                      excludeSemantics: true,
                      child: OutlinedButton.icon(
                        style: _bigButtonStyle(outlined: true),
                        onPressed: () {
                          unawaited(HapticFeedback.heavyImpact());
                          unawaited(_stopNavigationByUser());
                        },
                        icon: const Icon(Icons.stop_circle, size: 30),
                        label: const Text('안내 중지'),
                      ),
                    ),
                  ],
                ],
              ),
            ),

            // -----------------------------------------------------------------
            // 지도 / 경로 (시연 중에는 위의 ㄱ자 경로 그림으로 대신함)
            // 스크린리더(TalkBack/VoiceOver)로는 의미가 없고 초점만 잡아먹으므로 제외
            // -----------------------------------------------------------------
            if (!_isDemoMode)
            ExcludeSemantics(
              child: _sectionCard(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  const Text(
                    '지도 / 보행 경로',
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 12),
                  SizedBox(
                    height: 300,
                    child: ClipRRect(
                      borderRadius: BorderRadius.circular(14),
                      child: FlutterMap(
                        mapController: _mapController,
                        options: MapOptions(
                          initialCenter: _currentLatLng ??
                              const LatLng(
                                37.5665,
                                126.9780,
                              ),
                          initialZoom: 16,
                        ),
                        children: [
                          TileLayer(
                            urlTemplate:
                            'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                            userAgentPackageName:
                            'com.example.smart_cane_app',
                          ),
                          if (_routeLatLngs.isNotEmpty)
                            PolylineLayer(
                              polylines: [
                                Polyline(
                                  points: _routeLatLngs,
                                  strokeWidth: 5,
                                ),
                              ],
                            ),
                          if (_currentLatLng != null)
                            MarkerLayer(
                              markers: [
                                Marker(
                                  point: _currentLatLng!,
                                  width: 52,
                                  height: 52,
                                  child: Transform.rotate(
                                    angle: ((_phoneHeading ?? 0) *
                                        math.pi /
                                        180.0),
                                    child: const Icon(
                                      Icons.navigation,
                                      size: 42,
                                      color: Colors.blue,
                                    ),
                                  ),
                                ),
                              ],
                            ),
                          if (_destination != null)
                            MarkerLayer(
                              markers: [
                                Marker(
                                  point: LatLng(
                                    _destination!.latitude,
                                    _destination!.longitude,
                                  ),
                                  width: 52,
                                  height: 52,
                                  child: const Icon(
                                    Icons.location_on,
                                    size: 44,
                                    color: Colors.red,
                                  ),
                                ),
                              ],
                            ),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
            ),
            ),

            // -----------------------------------------------------------------
            // 내비게이션 정보
            // -----------------------------------------------------------------
            _sectionCard(
              child: Column(
                children: [
                  const Text(
                    '길 안내',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 10),
                  Text(
                    _directionText(),
                    style: const TextStyle(
                      fontSize: 28,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 12),
                  Row(
                    children: [
                      const Expanded(
                        child: Text('스마트폰 나침반'),
                      ),
                      Text(
                        _headingText(),
                        style: const TextStyle(
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 6),
                  Row(
                    children: [
                      const Expanded(
                        child: Text('목표 방위각'),
                      ),
                      Text(
                        _targetBearing == null
                            ? '--'
                            : '${_targetBearing!.toStringAsFixed(0)}°',
                        style: const TextStyle(
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 6),
                  Row(
                    children: [
                      const Expanded(
                        child: Text('경로선과 거리'),
                      ),
                      Text(
                        _routeDeviationText(),
                        style: const TextStyle(
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 6),
                  Row(
                    children: [
                      const Expanded(
                        child: Text('다음 경로점'),
                      ),
                      Text(
                        _distanceToNextPointMeters == null
                            ? '--'
                            : '${_distanceToNextPointMeters!.toStringAsFixed(0)} m',
                        style: const TextStyle(
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 6),
                  Row(
                    children: [
                      const Expanded(
                        child: Text('목적지까지'),
                      ),
                      Text(
                        _distanceToDestinationMeters == null
                            ? '--'
                            : '${_distanceToDestinationMeters!.toStringAsFixed(0)} m',
                        style: const TextStyle(
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),

            // -----------------------------------------------------------------
            // BLE
            // -----------------------------------------------------------------
            _sectionCard(
              child: Column(
                children: [
                  const Text(
                    'BLE 연결 상태',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 8),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(
                        Icons.circle,
                        size: 13,
                        color: _isBleConnected
                            ? Colors.green
                            : Colors.grey,
                      ),
                      const SizedBox(width: 8),
                      Text(
                        _bleStatus,
                        style: const TextStyle(
                          fontSize: 17,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 10),
                  Semantics(
                    button: true,
                    label: _isBleConnected
                        ? '지팡이 연결됨'
                        : _isConnecting
                        ? '지팡이 연결 중'
                        : '지팡이 연결하기',
                    excludeSemantics: true,
                    child: FilledButton.icon(
                      style: _bigButtonStyle(),
                      onPressed: _isConnecting || _isBleConnected
                          ? null
                          : () {
                        unawaited(HapticFeedback.mediumImpact());
                        unawaited(_connectSmartCane());
                      },
                      icon: const Icon(Icons.bluetooth, size: 30),
                      label: Text(
                        _isBleConnected
                            ? '지팡이 연결됨'
                            : _isConnecting
                            ? '지팡이 연결 중...'
                            : '지팡이 연결하기',
                      ),
                    ),
                  ),
                ],
              ),
            ),

            // -----------------------------------------------------------------
            // 거리 센서: LEFT VL53L1X / FRONT TFmini / RIGHT VL53L1X
            // -----------------------------------------------------------------
            _sectionCard(
              child: Column(
                children: [
                  const Text(
                    '장애물 거리 센서',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 12),
                  Row(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      _distanceSensorBox(
                        title: '왼쪽\nVL53L1X',
                        icon: Icons.arrow_back,
                        distanceCm: _leftDistanceCm,
                        warningDistanceCm: 30.0,
                      ),
                      const SizedBox(width: 8),
                      _distanceSensorBox(
                        title: '정면\nTFmini',
                        icon: Icons.arrow_upward,
                        distanceCm: _distanceCm,
                        warningDistanceCm: obstacleWarningCm,
                      ),
                      const SizedBox(width: 8),
                      _distanceSensorBox(
                        title: '오른쪽\nVL53L1X',
                        icon: Icons.arrow_forward,
                        distanceCm: _rightDistanceCm,
                        warningDistanceCm: 30.0,
                      ),
                    ],
                  ),
                  const SizedBox(height: 10),
                  Text(
                    '정면 40 cm 이하: 화면 장애물 경고  •  '
                        '3회 연속 확인: 역토크 3회 후 좌우 거리 비교·회피',
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      fontSize: 12,
                      color: Colors.grey.shade700,
                    ),
                  ),
                  if (obstacleDetected) ...[
                    const SizedBox(height: 10),
                    const Text(
                      '정면 장애물 감지',
                      style: TextStyle(
                        color: Colors.red,
                        fontSize: 21,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                  ],
                  if (_obstacleConfirmed) ...[
                    const SizedBox(height: 12),
                    Container(
                      width: double.infinity,
                      padding: const EdgeInsets.all(12),
                      decoration: BoxDecoration(
                        color: Colors.orange.shade50,
                        borderRadius: BorderRadius.circular(12),
                        border: Border.all(
                          color: Colors.orange.shade300,
                        ),
                      ),
                      child: const Column(
                        children: [
                          Text(
                            'ESP32 장애물 회피 동작',
                            style: TextStyle(
                              color: Colors.deepOrange,
                              fontSize: 18,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                          SizedBox(height: 6),
                          Text(
                            '정면 40 cm 이하 3회 연속 감지',
                            style: TextStyle(
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                          SizedBox(height: 4),
                          Text(
                            '좌·우 VL53L1X 거리를 비교해 더 넓은 방향으로 유도',
                            textAlign: TextAlign.center,
                            style: TextStyle(
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                        ],
                      ),
                    ),
                    const SizedBox(height: 8),
                    Text(
                      _avoidDirection == 'LEFT'
                          ? '회피 방향: 왼쪽 (정면이 트일 때까지 유도)'
                          : _avoidDirection == 'RIGHT'
                          ? '회피 방향: 오른쪽 (정면이 트일 때까지 유도)'
                          : _avoidDirection == 'BLOCKED'
                          ? '길이 막힘: 멈춤'
                          : '회피 방향 판단 중',
                      style: const TextStyle(
                        color: Colors.deepOrange,
                        fontSize: 18,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                  ],
                ],
              ),
            ),

            // -----------------------------------------------------------------
            // 횡단보도 / 신호등
            // -----------------------------------------------------------------
            _sectionCard(
              child: Column(
                children: [
                  const Text(
                    '라즈베리파이 인식 결과',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 14),
                  Row(
                    children: [
                      Icon(
                        Icons.directions_walk,
                        color: _crosswalkDetected
                            ? Colors.deepPurple
                            : Colors.grey,
                      ),
                      const SizedBox(width: 12),
                      Expanded(
                        child: Text(
                          _crosswalkText(),
                          style: TextStyle(
                            fontSize: 20,
                            fontWeight: FontWeight.bold,
                            color: _crosswalkDetected
                                ? Colors.deepPurple
                                : Colors.grey,
                          ),
                        ),
                      ),
                    ],
                  ),
                  const Divider(height: 28),
                  Row(
                    children: [
                      Icon(
                        Icons.traffic,
                        color: _trafficLightColor(),
                      ),
                      const SizedBox(width: 12),
                      Expanded(
                        child: Text(
                          _trafficLightText(),
                          style: TextStyle(
                            fontSize: 20,
                            fontWeight: FontWeight.bold,
                            color: _trafficLightColor(),
                          ),
                        ),
                      ),
                    ],
                  ),
                  if (_isCrosswalkMode) ...[
                    const Divider(height: 28),

                    Row(
                      children: [
                        const Icon(
                          Icons.compare_arrows,
                          color: Colors.deepPurple,
                        ),

                        const SizedBox(width: 12),

                        Expanded(
                          child: Text(
                            _crosswalkSteering == 'LEFT'
                                ? '횡단보도 보정: 왼쪽'
                                : _crosswalkSteering == 'RIGHT'
                                ? '횡단보도 보정: 오른쪽'
                                : _crosswalkSteering == 'CENTER'
                                ? '횡단보도 중앙 유지'
                                : '횡단보도 방향 판단 중',
                            style: const TextStyle(
                              fontSize: 20,
                              fontWeight: FontWeight.bold,
                              color: Colors.deepPurple,
                            ),
                          ),
                        ),
                      ],
                    ),
                  ],
                ],
              ),
            ),

            Text(
              '정면 TFmini와 좌·우 VL53L1X 거리를 BLE로 표시합니다.',
              textAlign: TextAlign.center,
              style: TextStyle(
                fontSize: 12,
                color: Colors.grey.shade700,
              ),
            ),
          ],
        ),
      ),
    );
  }
}

// =============================================================================
// 데이터 모델
// =============================================================================

class Destination {
  const Destination({
    required this.name,
    required this.latitude,
    required this.longitude,
  });

  final String name;
  final double latitude;
  final double longitude;
}

class RoutePoint {
  const RoutePoint({
    required this.latitude,
    required this.longitude,
  });

  final double latitude;
  final double longitude;
}

class _RouteProjection {
  const _RouteProjection({
    required this.nearestPoint,
    required this.distanceMeters,
    required this.segmentIndex,
    required this.alongMeters,
  });

  final RoutePoint nearestPoint;
  final double distanceMeters;
  final int segmentIndex;

  // 경로 시작점부터 nearestPoint까지 경로를 따라 잰 거리
  final double alongMeters;
}

// 현재 위치를 원점으로 하는 평면 좌표(m). x: 동쪽, y: 북쪽
typedef _LocalMeters = ({double x, double y});

// 음성 안내 우선순위 (_announce 참고). 뒤에 있을수록 급하다.
enum _SpeechPriority { info, important, urgent }

class _SpeechRequest {
  _SpeechRequest({
    required this.text,
    required this.priority,
    this.key,
    this.maxAge,
  }) : createdAt = DateTime.now();

  final String text;
  final _SpeechPriority priority;
  final String? key;
  final Duration? maxAge;
  final DateTime createdAt;

  final Completer<void> done = Completer<void>();

  bool get isStale {
    final Duration? age = maxAge;
    return age != null && DateTime.now().difference(createdAt) > age;
  }

  void finish() {
    if (!done.isCompleted) done.complete();
  }
}

// =============================================================================
// ㄱ자 시연 경로 그림
// 출발 → 횡단보도 → 회전 → 장애물 → 도착. 지나온 구간은 보라색으로 칠한다.
// =============================================================================

class _DemoCoursePainter extends CustomPainter {
  const _DemoCoursePainter({
    required this.step,
    required this.turnRight,
    required this.atCrosswalk,
    required this.atObstacle,
  });

  final int step;
  final bool turnRight;
  final bool atCrosswalk;
  final bool atObstacle;

  @override
  void paint(Canvas canvas, Size size) {
    Offset at(double x, double y) => Offset(size.width * x, size.height * y);

    // 오른쪽으로 꺾으면 아래로, 왼쪽으로 꺾으면 위로 내려가는 모양
    final double firstY = turnRight ? 0.22 : 0.82;
    final double lastY = turnRight ? 0.86 : 0.18;
    final Offset start = at(0.12, firstY);
    final Offset corner = at(0.68, firstY);
    final Offset end = at(0.68, lastY);
    final Offset crosswalk = Offset.lerp(start, corner, 0.55)!;
    final Offset obstacle = Offset.lerp(corner, end, 0.6)!;

    Paint road(bool done) => Paint()
      ..color = done ? Colors.deepPurple : Colors.grey.shade400
      ..strokeWidth = 8
      ..strokeCap = StrokeCap.round
      ..style = PaintingStyle.stroke;

    canvas.drawLine(start, corner, road(step >= 2));
    canvas.drawLine(corner, end, road(step >= 4));

    // 횡단보도: 진행 방향과 수직인 흰/검 줄무늬
    final Paint stripe = Paint()
      ..color = Colors.black87
      ..strokeWidth = 4;
    for (int i = -2; i <= 2; i++) {
      final double x = crosswalk.dx + i * 9.0;
      canvas.drawLine(
        Offset(x, crosswalk.dy - 18),
        Offset(x, crosswalk.dy + 18),
        stripe,
      );
    }

    // 장애물
    canvas.drawRRect(
      RRect.fromRectAndRadius(
        Rect.fromCenter(center: obstacle, width: 30, height: 18),
        const Radius.circular(4),
      ),
      Paint()..color = Colors.orange,
    );

    final double labelGap = turnRight ? -30 : 30;
    _label(canvas, '출발', start + Offset(0, labelGap));
    _label(canvas, '횡단보도', crosswalk + Offset(0, labelGap));
    _label(canvas, '장애물', obstacle + const Offset(48, 0));
    _label(canvas, '도착', end + Offset(0, -labelGap * 0.8));

    // 현재 단계 위치
    final Offset me = switch (step) {
      2 => corner,
      3 => Offset.lerp(corner, end, atObstacle ? 0.42 : 0.25)!,
      4 => end,
      _ => atCrosswalk
          ? Offset.lerp(start, corner, 0.45)!
          : Offset.lerp(start, corner, 0.08)!,
    };

    canvas.drawCircle(me, 12, Paint()..color = Colors.white);
    canvas.drawCircle(me, 9, Paint()..color = Colors.blue);
  }

  void _label(Canvas canvas, String text, Offset center) {
    final TextPainter painter = TextPainter(
      text: TextSpan(
        text: text,
        style: const TextStyle(
          fontSize: 12,
          fontWeight: FontWeight.bold,
          color: Colors.black87,
        ),
      ),
      textDirection: TextDirection.ltr,
    )..layout();

    painter.paint(
      canvas,
      center - Offset(painter.width / 2, painter.height / 2),
    );
  }

  @override
  bool shouldRepaint(_DemoCoursePainter oldDelegate) {
    return oldDelegate.step != step ||
        oldDelegate.turnRight != turnRight ||
        oldDelegate.atCrosswalk != atCrosswalk ||
        oldDelegate.atObstacle != atObstacle;
  }
}
