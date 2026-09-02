import 'dart:async';
import 'dart:convert';
import 'dart:io' show Platform;
import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
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

  // TFmini: 100cm 이내일 때만 "장애물 감지"를 화면에 표시.
  // 장애물 TTS는 하지 않는다.
  static const double obstacleWarningCm = 100.0;

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

  // ===========================================================================
  // 지도 / 경로 / 내비게이션
  // ===========================================================================
  LatLng? _currentLatLng;
  Destination? _destination;
  List<RoutePoint> _routePoints = [];
  int _routeProgressIndex = 0;

  bool _isSearchingRoute = false;
  bool _isNavigationActive = false;
  bool _isGuidanceUpdating = false;

  double? _phoneHeading;
  double? _targetBearing;
  double? _steeringError;
  double? _distanceToDestinationMeters;
  double? _distanceToNextPointMeters;

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

  int? _lastSentBearing;
  DateTime? _lastBearingSentAt;

  String? _lastMotorCommand;
  double? _lastMotorAngle;
  DateTime? _lastMotorSentAt;

  bool _isCrosswalkMode = false;

  // ESP32 장애물 처리 상태
  bool _obstacleConfirmed = false;

  // ===========================================================================
  // TFmini / Raspberry Pi 인식 결과
  // ===========================================================================
  double? _distanceCm;
  bool _crosswalkDetected = false;
  String _trafficLight = 'NONE'; // RED / GREEN / NONE

  // 횡단보도 주행 중 Raspberry Pi → ESP32 모터 보정 상태
  // LEFT / RIGHT / CENTER / NONE
  String _crosswalkSteering = 'NONE';

  @override
  void initState() {
    super.initState();

    _startCompass();

    Future<void>.delayed(const Duration(milliseconds: 600), () {
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

      // 앱 실행 후 자동으로 목적지를 묻고 바로 듣는다.
      await _speakAndListen(
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

  Future<void> _speak(String text) async {
    if (text.trim().isEmpty) return;

    if (_speech.isListening) {
      try {
        await _speech.stop();
      } catch (_) {}
    }

    if (mounted) {
      setState(() {
        _isListening = false;
        _isSpeaking = true;
      });
    } else {
      _isSpeaking = true;
    }

    try {
      await _tts.stop();
      await _tts.speak(text);
    } finally {
      _isSpeaking = false;
      if (mounted) {
        setState(() {});
      }
    }
  }

  Future<void> _speakAndListen(String message) async {
    await _speak(message);
    await Future<void>.delayed(const Duration(milliseconds: 500));

    if (!mounted) return;
    await _startListening();
  }

  Future<void> _startListening() async {
    if (!_speechAvailable ||
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

    if (_containsAny(command, ['다시 안내', '현재 안내', '반복'])) {
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
        _routeProgressIndex = 0;
        _isNavigationActive = true;
        _targetBearing = null;
        _steeringError = null;
        _distanceToDestinationMeters = null;
        _distanceToNextPointMeters = null;
        _speechStatus = '${destination.name} 경로 검색 완료';
      });

      _moveMapToCurrentLocation();

      await _startLocationTracking();
      _startGuidanceTimer();

      // 목적지 입력이 끝난 뒤 BLE 자동 연결 시도.
      // 실패해도 지도/길찾기는 계속 동작한다.
      if (!_isBleConnected && !_isConnecting) {
        unawaited(_connectSmartCane(silent: true));
      }

      await _updateGuidanceByPosition(
        currentPosition,
        forceSpeak: true,
      );

      success = true;
    } catch (e) {
      failureMessage = e.toString();

      if (mounted) {
        setState(() {
          _isNavigationActive = false;
          _destination = null;
          _routePoints = [];
          _routeProgressIndex = 0;
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

    LocationPermission permission = await Geolocator.checkPermission();

    if (permission == LocationPermission.denied) {
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

    const LocationSettings settings = LocationSettings(
      accuracy: LocationAccuracy.bestForNavigation,
      distanceFilter: 2,
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
        debugPrint('[GPS ERROR] $error');
      },
    );
  }

  void _startGuidanceTimer() {
    _guidanceTimer?.cancel();

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

      final double distanceToDestination = Geolocator.distanceBetween(
        currentPosition.latitude,
        currentPosition.longitude,
        destination.latitude,
        destination.longitude,
      );

      if (distanceToDestination <= 10.0) {
        await _finishNavigation();
        return;
      }

      final RoutePoint nextPoint = _findNextRoutePoint(
        currentPosition: currentPosition,
        routePoints: _routePoints,
      );

      final double distanceToNextPoint = Geolocator.distanceBetween(
        currentPosition.latitude,
        currentPosition.longitude,
        nextPoint.latitude,
        nextPoint.longitude,
      );

      final double targetBearing = _calculateTargetBearing(
        startLatitude: currentPosition.latitude,
        startLongitude: currentPosition.longitude,
        destinationLatitude: nextPoint.latitude,
        destinationLongitude: nextPoint.longitude,
      );

      final double? heading = _phoneHeading;
      final double? steeringError = heading == null
          ? null
          : _signedAngleDifference(
              targetBearing,
              heading,
            );

      if (mounted) {
        setState(() {
          _currentLatLng = LatLng(
            currentPosition.latitude,
            currentPosition.longitude,
          );

          _targetBearing = targetBearing;
          _steeringError = steeringError;
          _distanceToDestinationMeters = distanceToDestination;
          _distanceToNextPointMeters = distanceToNextPoint;
        });
      }

      await _sendBearingToCaneIfNeeded(targetBearing);

      if (steeringError != null &&
        !_isCrosswalkMode &&
        !_obstacleConfirmed) {
        final String command = _motorCommandForAngle(steeringError);

        await _sendMotorCommandToCaneIfNeeded(
          command,
          angle: steeringError.abs(),
        );
      }

      // 직진 / 왼쪽 / 오른쪽 길안내 TTS는 사용하지 않는다.
      // 화면 표시와 ESP32 방향 명령(F/L/R)은 계속 동작한다.
    } finally {
      _isGuidanceUpdating = false;
    }
  }

  RoutePoint _findNextRoutePoint({
    required Position currentPosition,
    required List<RoutePoint> routePoints,
  }) {
    if (routePoints.isEmpty) {
      throw StateError('경로 좌표가 없습니다.');
    }

    if (_routeProgressIndex >= routePoints.length) {
      _routeProgressIndex = routePoints.length - 1;
    }

    final int searchEnd = math.min(
      routePoints.length - 1,
      _routeProgressIndex + 80,
    );

    int nearestIndex = _routeProgressIndex;
    double nearestDistance = double.infinity;

    for (int i = _routeProgressIndex; i <= searchEnd; i++) {
      final RoutePoint point = routePoints[i];

      final double distance = Geolocator.distanceBetween(
        currentPosition.latitude,
        currentPosition.longitude,
        point.latitude,
        point.longitude,
      );

      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearestIndex = i;
      }
    }

    if (nearestIndex > _routeProgressIndex) {
      _routeProgressIndex = nearestIndex;
    }

    for (int i = _routeProgressIndex; i < routePoints.length; i++) {
      final RoutePoint point = routePoints[i];

      final double distance = Geolocator.distanceBetween(
        currentPosition.latitude,
        currentPosition.longitude,
        point.latitude,
        point.longitude,
      );

      if (distance >= 8.0) {
        _routeProgressIndex = i;
        return point;
      }
    }

    return routePoints.last;
  }

  Future<void> _speakCurrentGuidance() async {
    // 길안내 방향(직진/왼쪽/오른쪽)은 음성으로 읽지 않는다.
    // 화면의 길 안내 카드에서 현재 방향과 거리를 확인한다.
    if (!_isNavigationActive) {
      await _speak('현재 진행 중인 안내가 없습니다.');
    }
  }

  Future<void> _stopNavigationByUser() async {
    await _positionSubscription?.cancel();
    _positionSubscription = null;

    _guidanceTimer?.cancel();
    _guidanceTimer = null;

    if (mounted) {
      setState(() {
        _isNavigationActive = false;
        _routePoints = [];
        _routeProgressIndex = 0;
        _destination = null;
        _targetBearing = null;
        _steeringError = null;
        _distanceToDestinationMeters = null;
        _distanceToNextPointMeters = null;
        _recognizedDestinationText = '';
        _speechStatus = '안내가 중지되었습니다.';
      });
    }

    await _sendBleLine('S:0');
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

    if (mounted) {
      setState(() {
        _isNavigationActive = false;
        _routePoints = [];
        _routeProgressIndex = 0;
        _targetBearing = null;
        _steeringError = null;
        _distanceToNextPointMeters = null;
        _distanceToDestinationMeters = 0;
        _speechStatus = '목적지에 도착했습니다.';
      });
    }

    await _sendBleLine('S:0');
    await _speak('목적지에 도착했습니다. 안내를 종료합니다.');
  }

  // ===========================================================================
  // 스마트폰 나침반 / 방위각
  // ===========================================================================

  void _startCompass() {
    final Stream<CompassEvent>? stream = FlutterCompass.events;

    if (stream == null) return;

    _compassSubscription = stream.listen(
      (CompassEvent event) {
        final double? raw = event.heading;

        if (raw == null || !raw.isFinite || !mounted) {
          return;
        }

        final double heading = (raw + 360.0) % 360.0;

        setState(() {
          _phoneHeading = heading;
        });

        if (_isNavigationActive &&
            _targetBearing != null &&
            !_isCrosswalkMode &&
            !_obstacleConfirmed) {
          final double error = _signedAngleDifference(
            _targetBearing!,
            heading,
          );

          final String command = _motorCommandForAngle(error);

          unawaited(
            _sendMotorCommandToCaneIfNeeded(
              command,
              angle: error.abs(),
            ),
          );

          if (mounted) {
            setState(() {
              _steeringError = error;
            });
          }
        }
      },
    );
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

  String _motorCommandForAngle(double steeringError) {
    if (steeringError.abs() <= 12.0) {
      return 'F';
    }

    return steeringError > 0 ? 'R' : 'L';
  }

  String _directionText() {
    final double? error = _steeringError;

    if (!_isNavigationActive) return '안내 대기';
    if (error == null) return '나침반 준비 중';
    if (error.abs() <= 12.0) return '직진';
    return error > 0 ? '오른쪽' : '왼쪽';
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

  Future<void> _connectSmartCane({
    bool silent = false,
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
        withNames: const [targetDeviceName],
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

            setState(() {
              _isBleConnected = false;
              _bleStatus = '연결 끊김';

              _device = null;
              _writeCharacteristic = null;
              _notifyCharacteristic = null;

              _distanceCm = null;

              _obstacleConfirmed = false;

              _isCrosswalkMode = false;
              _crosswalkDetected = false;
              _trafficLight = 'NONE';
              _crosswalkSteering = 'NONE';
            });

            _resetMotorCommandCache();
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
    } catch (e) {
      if (mounted) {
        setState(() {
          _isBleConnected = false;
          _bleStatus = '연결 실패';
        });
      }

      debugPrint('[BLE ERROR] $e');

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

  Future<void> _sendBleLine(String message) async {
    final BluetoothCharacteristic? characteristic =
        _writeCharacteristic;

    if (!_isBleConnected || characteristic == null) {
      return;
    }

    try {
      await characteristic.write(
        utf8.encode('$message\n'),
        withoutResponse: _writeWithoutResponse,
      );
    } catch (e) {
      debugPrint('[BLE TX ERROR] $e');
    }
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

    _lastSentBearing = bearingInt;
    _lastBearingSentAt = now;

    await _sendBleLine('B:$bearingInt');
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

      final DateTime now = DateTime.now();

      final bool commandChanged =
          command != _lastMotorCommand;

      final bool angleChanged =
          _lastMotorAngle == null ||
          (angle - _lastMotorAngle!).abs() >= 5.0;

      final bool enoughTime =
          _lastMotorSentAt == null ||
          now.difference(_lastMotorSentAt!).inMilliseconds >= 700;

      if (!commandChanged &&
          !angleChanged &&
          !enoughTime) {
        return;
      }

      _lastMotorCommand = command;
      _lastMotorAngle = angle;
      _lastMotorSentAt = now;

      await _sendBleLine(
        '$command:${angle.toStringAsFixed(1)}',
      );

      debugPrint(
        '[NAV -> ESP32] '
        '$command:${angle.toStringAsFixed(1)}',
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
    // 100cm 이하 -> 화면 장애물 표시
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
    // 2. 장애물 확정
    //
    // ESP32에서 30cm 이하 감지 시:
    // OBSTACLE:25
    //
    // 이 메시지를 보낸 직후 ESP32가
    // 양쪽 모터에 역토크 3회를 자체적으로 실행함.
    // ============================================================
    if (message.startsWith('OBSTACLE:')) {
      final double? distance = double.tryParse(
        message.substring('OBSTACLE:'.length).trim(),
      );

      if (mounted) {
        setState(() {
          _obstacleConfirmed = true;

          if (distance != null) {
            _distanceCm = distance;
          }
        });
      }

      // 장애물 처리 중 Flutter의 이전 L/R/F 캐시 초기화
      _resetMotorCommandCache();

      debugPrint(
        '[OBSTACLE] confirmed - ESP32 reverse torque x3',
      );

      return;
    }

    // ============================================================
    // 3. 장애물 해제
    //
    // ESP32에서 TFmini 거리 40cm 이상이 되면:
    // OBSTACLE_CLEAR
    // ============================================================
    if (message == 'OBSTACLE_CLEAR') {
      if (mounted) {
        setState(() {
          _obstacleConfirmed = false;
        });
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

      if (!wasDetected) {
        unawaited(
          _speakCrosswalkDetectedOnce(),
        );
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

      if (!oldCrosswalk) {
        unawaited(
          _speak('횡단보도를 인식했습니다.'),
        );
      }

      if (oldLight != 'RED') {
        unawaited(
          _speak(
            '빨간불을 인식했습니다. 정지하세요.',
          ),
        );
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

      if (!oldCrosswalk) {
        unawaited(
          _speak('횡단보도를 인식했습니다.'),
        );
      }

      if (oldLight != 'GREEN') {
        unawaited(
          _speak(
            '초록불을 인식했습니다. 건너도 됩니다.',
          ),
        );
      }

      debugPrint('[TRAFFIC] GREEN');

      return;
    }

    // ============================================================
    // 8. 신호등 미인식
    //
    // LIGHT:NONE
    // ============================================================
    if (message == 'LIGHT:NONE') {
      if (mounted) {
        setState(() {
          _trafficLight = 'NONE';
        });
      }

      return;
    }

    // ============================================================
    // 9. 횡단 시작
    //
    // CROSSING_START
    // ============================================================
    if (message == 'CROSSING_START') {
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

  void _resetMotorCommandCache() {
    _lastMotorCommand = null;
    _lastMotorAngle = null;
    _lastMotorSentAt = null;
  }

  Future<void> _speakCrosswalkDetectedOnce() async {
    await _speak(
      '횡단보도를 인식했습니다.',
    );
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

    return '신호등 대기 중';
  }

  Color _trafficLightColor() {
    if (_trafficLight == 'RED') return Colors.red;
    if (_trafficLight == 'GREEN') return Colors.green;
    return Colors.grey;
  }

  @override
  Widget build(BuildContext context) {
    final bool obstacleDetected = _distanceCm != null &&
        _distanceCm! > 0 &&
        _distanceCm! <= obstacleWarningCm;

    return Scaffold(
      appBar: AppBar(
        title: const Text('Smart Cane'),
        centerTitle: true,
      ),
      body: SafeArea(
        child: ListView(
          padding: const EdgeInsets.all(14),
          children: [
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
                  const SizedBox(height: 10),
                  FilledButton.icon(
                    onPressed: _isListening ||
                            _isSearchingRoute ||
                            _isSpeaking
                        ? null
                        : _startListening,
                    icon: const Icon(Icons.mic),
                    label: Text(
                      _isNavigationActive
                          ? '목적지/명령 말하기'
                          : '목적지 다시 말하기',
                    ),
                  ),
                  if (_isNavigationActive) ...[
                    const SizedBox(height: 8),
                    OutlinedButton.icon(
                      onPressed: _stopNavigationByUser,
                      icon: const Icon(Icons.stop),
                      label: const Text('안내 중지'),
                    ),
                  ],
                ],
              ),
            ),

            // -----------------------------------------------------------------
            // 지도 / 경로
            // -----------------------------------------------------------------
            _sectionCard(
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
                  FilledButton.icon(
                    onPressed:
                        _isConnecting ? null : _connectSmartCane,
                    icon: const Icon(Icons.bluetooth),
                    label: Text(
                      _isBleConnected
                          ? 'SMART_CANE 연결됨'
                          : 'SMART_CANE 연결',
                    ),
                  ),
                ],
              ),
            ),

            // -----------------------------------------------------------------
            // TFmini
            // -----------------------------------------------------------------
            _sectionCard(
              child: Column(
                children: [
                  const Text(
                    'TFmini 측정 거리',
                    style: TextStyle(
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  const SizedBox(height: 8),
                  Text(
                    _distanceCm == null
                        ? '-- cm'
                        : '${_distanceCm!.toStringAsFixed(0)} cm',
                    style: const TextStyle(
                      fontSize: 36,
                      fontWeight: FontWeight.bold,
                    ),
                  ),
                  if (obstacleDetected) ...[
                    const SizedBox(height: 8),
                    const Text(
                      '장애물 감지',
                      style: TextStyle(
                        color: Colors.red,
                        fontSize: 21,
                        fontWeight: FontWeight.bold,
                      ),
                    ),
                    const Text(
                      '100 cm 이내',
                      style: TextStyle(
                        color: Colors.red,
                      ),
                    ),
                  ],

                  if (_obstacleConfirmed) ...[
                    const SizedBox(height: 12),

                    Container(
                      width: double.infinity,
                      padding: const EdgeInsets.all(12),
                      decoration: BoxDecoration(
                        color: Colors.red.shade50,
                        borderRadius: BorderRadius.circular(12),
                        border: Border.all(
                          color: Colors.red.shade200,
                        ),
                      ),
                      child: const Column(
                        children: [
                          Text(
                            'ESP32 장애물 경고',
                            style: TextStyle(
                              color: Colors.red,
                              fontSize: 18,
                              fontWeight: FontWeight.bold,
                            ),
                          ),

                          SizedBox(height: 6),

                          Text(
                            '30 cm 이하 장애물 감지',
                            style: TextStyle(
                              fontWeight: FontWeight.bold,
                            ),
                          ),

                          SizedBox(height: 4),

                          Text(
                            '역토크 3회 동작',
                            style: TextStyle(
                              color: Colors.red,
                              fontWeight: FontWeight.bold,
                            ),
                          ),
                        ],
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
              'TFmini는 거리만 표시하며 장애물 음성은 출력하지 않습니다.',
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
