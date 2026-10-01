import cv2
import numpy as np
import os
import time
from ultralytics import YOLO

# ====================================================================
# [추가] PiCamera2 모듈 임포트
# ====================================================================
try:
    from picamera2 import Picamera2
    HAS_PICAM2 = True
except ImportError:
    HAS_PICAM2 = False
    print("[오류] 'picamera2' 모듈이 없습니다. 라즈베리파이 OS 환경을 확인해주세요.")

# ====================================================================
# [튜닝 변수] 노란색 횡단보도 검출을 위한 HSV 범위
# ====================================================================
YELLOW_HSV_LOWER = np.array([15, 60, 80])
YELLOW_HSV_UPPER = np.array([45, 255, 255])

# ====================================================================
# [튜닝 변수] 횡단 중 방향 보정
# ====================================================================
# 횡단보도 박스 중심이 화면 중심에서 "화면 폭 x 이 비율"보다 멀면
# 줄무늬 각도보다 "횡단보도 중앙으로 가기"를 먼저 한다. (예전: 25% 고정)
CENTER_ENTER_RATIO = 0.12
# 중앙으로 가는 중에는 이 비율 안으로 들어와야 각도(직진 유지) 판단으로 돌아간다.
CENTER_EXIT_RATIO = 0.06

# 줄무늬 각도 판단 문턱값(도). 흔들림이 심하면 ENTER를 4~6으로 올린다.
ANGLE_ENTER_DEG = 3.0
ANGLE_EXIT_DEG = 2.0

# 줄무늬 각도 보정에만 좌/우 반전을 적용합니다.
# 중앙 보정, 폭 감소에 따른 이탈 복귀, 검출 소실 복귀는 반전하지 않습니다.
# 각도 보정의 실제 조향 방향을 확인하고 필요하면 False로 바꿉니다.
FLIP_LEFT_RIGHT = True

# ====================================================================
# [ESP32 전송] 메시지 이름은 docs/PROTOCOL.md 기준 (ESP32/앱과 공통)
# ====================================================================
# 라즈베리파이 GPIO14(TX)/GPIO15(RX) UART 포트. 보드에 따라 이름이 다르다.
#  - Pi 5: GPIO14/15 = /dev/ttyAMA0 (/boot/firmware/config.txt 에 dtparam=uart0=on 필요)
#          /dev/serial0 은 기본적으로 HDMI 사이 3핀 디버그 단자(ttyAMA10)라 쓰면 안 된다.
#  - Pi 4: GPIO14/15 = /dev/serial0 (ttyAMA0 은 기본적으로 블루투스 칩)
PI5_SERIAL_PORTS = ['/dev/ttyAMA0', '/dev/ttyUSB0']
PI4_SERIAL_PORTS = ['/dev/serial0', '/dev/ttyS0', '/dev/ttyUSB0']
SERIAL_BAUD = 115200

# ====================================================================
# [속도 튜닝] YOLO 추론 입력 크기
# ====================================================================
# 카메라는 640x480 그대로 두고(줄무늬 각도/신호등 색 분석은 원본 해상도 사용),
# YOLO에 넣는 크기만 줄인다. 640 -> 416 이면 연산량이 약 42%로 줄어든다.
# 32의 배수만 사용 (320 / 416 / 480 / 640). 640이면 기존과 동일.
# 멀리 있는 신호등을 놓치면 480으로 올린다.
YOLO_IMGSZ = 416

# 건너는 동안 현재 보정 방향(CROSS_MOTOR:L/R/CENTER)을 이 간격(초)으로
# 계속 다시 보낸다. ESP32는 1초 동안 소식이 없으면 스스로 멈춘다.
CROSS_RESEND_SEC = 0.3

# 횡단보도 접근/신호 대기 상태(CROSSWALK:1, LIGHT:RED, LIGHT:GREEN_WAIT)도
# 이 간격으로 다시 보낸다. 앱 BLE 연결이 늦거나 한 번 놓쳐도 다시 받을 수 있다.
STATE_RESEND_SEC = 1.0

# 횡단보도 박스가 화면 아래쪽에 이 프레임 수만큼 보여야 CROSSWALK:1 전송.
# (추론 속도 기준 10프레임은 수 초가 걸려 "화면엔 보이는데 앱엔 안 뜸"처럼 보였다)
CROSSWALK_CONFIRM_FRAMES = 5

CROSSING_EVENTS = ["STRAIGHT", "LEFT_CORRECTION", "RIGHT_CORRECTION", "CROSSING_UNKNOWN"]

# ====================================================================
# [추가 튜닝 변수] 선 미검출 / 횡단 중 좌우 이탈 복귀
# 모든 프레임 수는 카메라 FPS가 아니라 실제 추론 루프의 연속 프레임 수입니다.
# ====================================================================
NO_LINES_STRAIGHT_FRAMES = 3  # 연속 3프레임 선 미검출 -> 각도 조향 Straight
ESCAPE_WIDTH_RATIO = 0.35    # 화면 가로 대비 박스 폭 35% 이하
ESCAPE_CONFIRM_FRAMES = 3   # 같은 쪽 이탈 후보가 연속 3프레임이어야 복귀
ESCAPE_SIDE_OFFSET_RATIO = 0.12  # 중심 근처의 작은 박스는 좌우 이탈로 보지 않음
ESCAPE_ARM_WIDTH_RATIO = 0.60    # 초록불 이후 넓게 보이는 정상 구간 기준
ESCAPE_ARM_FRAMES = 3           # 폭 60% 이상 연속 3프레임 -> 감지 활성화
ESCAPE_RELEASE_WIDTH_RATIO = 0.50  # 폭 회복 기준(35%와 다르게 설정)
ESCAPE_RELEASE_FRAMES = 3          # 복귀 해제 조건 연속 확인
# [검출 소실 복귀] 박스가 35%까지 줄기 전에 사라지는 경우에 사용.
LOSS_TREND_FRAMES = 3          # 소실 직전의 연속 검출 3개로 추세 확인
LOSS_TREND_MAX_SPAN_SEC = 3.0  # 지나치게 오래된 검출 묶음은 제외
LOSS_EVIDENCE_MAX_AGE_SEC = 2.0 # 소실 3프레임 확인 시 마지막 검출의 유효 기간
LOSS_MIN_WIDTH_DROP = 0.08     # 화면 폭 대비 최소 8%p 감소
LOSS_MIN_OFFSET_MOVE = 0.08    # 같은 쪽으로 중심이 최소 화면 폭 8% 이동
LOSS_TREND_JITTER = 0.03       # 추세 중 최대 3%p의 작은 흔들림 허용
LOSS_CONFIRM_FRAMES = 3       # 연속 3프레임 미검출
LOSS_RECOVERY_SEC = 0.6        # 새 영상 근거 없는 복귀는 짧게 제한
LOSS_RECOVERY_MAX_FRAMES = 3   # 시간 또는 프레임 제한 중 먼저 도달하면 중립
POSITION_UNKNOWN_FRAMES = 20  # 위치 판단 불가 + 중립 처리 (종료 전 단계)
# 건너는 중 횡단보도가 이 시간(초) 동안 한 번도 안 보이면 다 건넌 것으로 보고 종료.
# 프레임 수가 아니라 시간 기준이라 FPS가 바뀌어도 같은 시간 뒤에 종료된다.
# 짧게 하면 잠깐 놓쳤을 때 중간에 끝나고, 길게 하면 건넌 뒤 일반 안내 복귀가 늦어진다.
CROSSING_END_MISSING_SEC = 3.0


def new_crosswalk_state():
    return {'ema_angle': 0.0, 'direction': 'Straight', 'initialized': False,
            'centering': False, 'no_lines_frames': 0}


def on_no_lines(state):
    """짧은 누락에는 각도 조향 유지, 누락이 지속되면 각도 추적을 중립으로 초기화."""
    state['no_lines_frames'] = min(state['no_lines_frames'] + 1, NO_LINES_STRAIGHT_FRAMES)
    if state['no_lines_frames'] >= NO_LINES_STRAIGHT_FRAMES:
        state['direction'] = 'Straight'
        state['ema_angle'] = 0.0
        state['initialized'] = False
    return f"{state['direction']} (No Lines)", state['ema_angle'], 0.0


class CrossingEscapeGuard:
    """회전 보정된 영상 좌표로 판단합니다. 위치/이탈 복귀 방향은 반전하지 않습니다.

    armed는 실제 발의 진입 확인이 아니라, 초록불 이후 넓은 횡단보도를
    관찰했다는 활성화 조건입니다. 횡단 세션 동안 유지하고 종료 시 초기화합니다.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.armed = False
        self.arm_frames = 0
        self.candidate = None
        self.candidate_frames = 0
        self.recovery_direction = None
        self.release_frames = 0
        self.history = []
        self.loss_frames = 0
        self.loss_direction = None
        self.loss_started_at = None
        self.loss_command_frames = 0
        self.loss_handled = False
        self.loss_hint = None

    def trend_direction(self):
        """폭 감소와 한쪽 이동이 같이 있었을 때만 복귀 방향을 추정."""
        if len(self.history) < LOSS_TREND_FRAMES:
            return None
        samples = self.history[-LOSS_TREND_FRAMES:]
        if samples[-1][0] - samples[0][0] > LOSS_TREND_MAX_SPAN_SEC:
            return None
        widths = [v[1] for v in samples]
        offsets = [v[2] for v in samples]
        if widths[0] - widths[-1] < LOSS_MIN_WIDTH_DROP:
            return None
        if any(b - a > LOSS_TREND_JITTER for a, b in zip(widths, widths[1:])):
            return None
        side = -1 if offsets[-1] < -ESCAPE_SIDE_OFFSET_RATIO else (
            1 if offsets[-1] > ESCAPE_SIDE_OFFSET_RATIO else 0)
        if side == 0:
            return None
        toward_edge = [side * v for v in offsets]
        if toward_edge[-1] - toward_edge[0] < LOSS_MIN_OFFSET_MOVE:
            return None
        if any(b - a < -LOSS_TREND_JITTER for a, b in zip(toward_edge, toward_edge[1:])):
            return None
        return 'Turn Left' if side < 0 else 'Turn Right'

    def update_loss(self, now):
        self.loss_frames += 1
        self.arm_frames = 0
        self.candidate = None
        self.candidate_frames = 0
        self.release_frames = 0
        if self.loss_frames == 1:
            # 첫 누락에서만 직전 근거를 저장합니다. 같은 미검출 구간에서 재발동하지 않습니다.
            self.loss_hint = self.trend_direction()
            # 이미 35%/3프레임으로 확인한 복귀도 짧은 지속의 근거로 사용할 수 있습니다.
            if self.loss_hint is None and self.recovery_direction and self.history:
                _, width, offset = self.history[-1]
                same_side = ((self.recovery_direction == 'Turn Left' and
                              offset < -ESCAPE_SIDE_OFFSET_RATIO) or
                             (self.recovery_direction == 'Turn Right' and
                              offset > ESCAPE_SIDE_OFFSET_RATIO))
                if width <= ESCAPE_WIDTH_RATIO and same_side:
                    self.loss_hint = self.recovery_direction
            self.recovery_direction = None

        if not self.armed:
            self.history = []
            return None
        if self.loss_started_at is not None:
            if (now - self.loss_started_at >= LOSS_RECOVERY_SEC or
                    self.loss_command_frames >= LOSS_RECOVERY_MAX_FRAMES):
                self.loss_direction = None
                return 'Straight'
            self.loss_command_frames += 1
            return self.loss_direction
        if self.loss_frames < LOSS_CONFIRM_FRAMES:
            return None
        if self.loss_handled:
            return 'Straight'
        self.loss_handled = True
        recent = bool(self.history) and now - self.history[-1][0] <= LOSS_EVIDENCE_MAX_AGE_SEC
        if not recent or self.loss_hint is None:
            return 'Straight'
        self.loss_direction = self.loss_hint
        self.loss_started_at = now
        self.loss_command_frames = 1
        return self.loss_direction

    def update(self, crossing, width_ratio, offset_ratio, now=None):
        now = time.monotonic() if now is None else now
        if not crossing:
            self.reset()
            return None
        if width_ratio is None:
            return self.update_loss(now)

        # 다시 검출되면 추정 복귀 대신 현재 박스로 판단합니다.
        # 검출이 끊긴 앞뒤의 박스를 연속 추세로 연결하지 않습니다.
        if self.loss_frames:
            self.history = []
        self.loss_frames = 0
        self.loss_direction = None
        self.loss_started_at = None
        self.loss_command_frames = 0
        self.loss_handled = False
        self.loss_hint = None
        self.history.append((now, width_ratio, offset_ratio))
        self.history = self.history[-LOSS_TREND_FRAMES:]

        if not self.armed:
            if width_ratio >= ESCAPE_ARM_WIDTH_RATIO:
                self.arm_frames += 1
            else:
                self.arm_frames = 0
            if self.arm_frames >= ESCAPE_ARM_FRAMES:
                self.armed = True
            return None

        # 화면 왼쪽에 횡단보도가 남음 -> 영상 기준 왼쪽으로 복귀.
        side = None
        if offset_ratio < -ESCAPE_SIDE_OFFSET_RATIO:
            side = 'Turn Left'
        elif offset_ratio > ESCAPE_SIDE_OFFSET_RATIO:
            side = 'Turn Right'

        if self.recovery_direction is not None:
            # 횡단보도가 반대쪽으로 이동했다면 오래된 복귀 조향을 즉시 버립니다.
            # 반대 복귀 명령도 아래에서 새롭게 3프레임 확인해야 합니다.
            if side is not None and side != self.recovery_direction:
                self.recovery_direction = None
                self.release_frames = 0
                self.candidate = None
                self.candidate_frames = 0
            else:
                recovered = (width_ratio >= ESCAPE_RELEASE_WIDTH_RATIO or
                             abs(offset_ratio) <= CENTER_EXIT_RATIO)
                self.release_frames = self.release_frames + 1 if recovered else 0
                if self.release_frames >= ESCAPE_RELEASE_FRAMES:
                    self.recovery_direction = None
                    self.release_frames = 0
                    self.candidate = None
                    self.candidate_frames = 0
                    return None
                # 중앙에 돌아온 뒤 해제 확인 중에는 과도한 복귀 조향을 피합니다.
                if abs(offset_ratio) <= CENTER_EXIT_RATIO:
                    return 'Straight'
                return self.recovery_direction

        candidate = side if width_ratio <= ESCAPE_WIDTH_RATIO else None
        if candidate is None:
            self.candidate = None
            self.candidate_frames = 0
            return None
        if candidate == self.candidate:
            self.candidate_frames += 1
        else:
            self.candidate = candidate
            self.candidate_frames = 1
        if self.candidate_frames >= ESCAPE_CONFIRM_FRAMES:
            self.recovery_direction = candidate
            self.release_frames = 0
            return candidate
        # 3프레임 확인 전에 중심 보정으로 복귀가 시작되지 않도록 확인 중에는 중립.
        return 'Straight'


def motor_direction(image_direction):
    """줄무늬 각도 보정 방향만 설정에 따라 한 번 반전합니다."""
    if FLIP_LEFT_RIGHT:
        if image_direction == 'Turn Left':
            return 'Turn Right'
        if image_direction == 'Turn Right':
            return 'Turn Left'
    return image_direction


def esp_messages_for_event(event, stable_color, has_seen_red):
    """라즈베리파이 내부 상태 -> ESP32/앱 공통 메시지 목록."""
    if event == "NORMAL":
        return ["CROSSWALK:0"]
    if event == "CROSSWALK_AHEAD":
        return ["CROSSWALK:1"]
    if event == "WAIT_SIGNAL":
        # 빨간불을 못 보고 초록불부터 봤다 = 남은 시간을 모름 -> 다음 신호 대기
        if stable_color == "GREEN" and not has_seen_red:
            return ["LIGHT:GREEN_WAIT"]
        return ["LIGHT:RED"]
    if event == "CROSSING_START":
        return ["LIGHT:GREEN", "CROSSING_START"]
    if event in ["STRAIGHT", "CROSSING_UNKNOWN"]:
        return ["CROSS_MOTOR:CENTER"]
    if event == "LEFT_CORRECTION":
        return ["CROSS_MOTOR:L"]
    if event == "RIGHT_CORRECTION":
        return ["CROSS_MOTOR:R"]
    if event == "CROSSING_END":
        return ["CROSSING_END"]
    return []


def esp_resend_interval(event):
    """같은 메시지를 다시 보낼 간격(초). None이면 바뀔 때만 보낸다."""
    if event in CROSSING_EVENTS:
        return CROSS_RESEND_SEC
    if event in ["CROSSWALK_AHEAD", "WAIT_SIGNAL"]:
        return STATE_RESEND_SEC
    # CROSSWALK:0 은 반복하지 않는다. (앱이 받을 때마다 길안내 조향 상태를 초기화함)
    # CROSSING_START/END 는 1틱용이므로 반복하지 않는다.
    return None


class EspLink:
    """바뀐 메시지는 바로, 상태 유지 메시지는 주기적으로 다시 보낸다."""

    def __init__(self, ser):
        self.ser = ser
        self.last_messages = None
        self.last_sent_time = 0.0

    def update(self, messages, resend_sec, now):
        changed = messages != self.last_messages
        resend = (resend_sec is not None and
                  (now - self.last_sent_time) >= resend_sec)

        if not messages or (not changed and not resend):
            return []

        if changed:
            if self.ser is None:
                print(f"[ESP32 미연결 - 전송 안 됨] {' / '.join(messages)}")
            else:
                print(f"[ESP32 전송] {' / '.join(messages)}")

        if self.ser is not None:
            try:
                for message in messages:
                    self.ser.write(f"{message}\n".encode("utf-8"))
                self.ser.flush()
            except Exception as e:
                print(f" └-> [ESP32 데이터 전송 실패]: {e}")

        self.last_messages = messages
        self.last_sent_time = now
        return messages


def pi_model():
    try:
        with open('/proc/device-tree/model') as f:
            return f.read().strip('\x00 \n')
    except OSError:
        return ''


def open_esp_serial():
    """보드에 맞는 GPIO UART 포트를 순서대로 찾아 연다. 실패하면 None."""
    model_name = pi_model()
    is_pi5 = 'Raspberry Pi 5' in model_name
    ports = PI5_SERIAL_PORTS if is_pi5 else PI4_SERIAL_PORTS
    print(f"보드: {model_name or '알 수 없음'} -> 시리얼 후보 {ports}")
    for port in ports:
        if not os.path.exists(port):
            continue
        try:
            ser = serial.Serial(port, SERIAL_BAUD, timeout=1, write_timeout=0.2)
            real = os.path.realpath(port)
            print(f"ESP32 시리얼 통신 연결 성공! ({port} -> {real})")
            return ser
        except Exception as e:
            print(f"ESP32 시리얼 {port} 열기 실패: {e}")
    if is_pi5:
        print("ESP32 시리얼 연결 실패 (통신 없이 진행합니다). "
              "/boot/firmware/config.txt 에 dtparam=uart0=on 추가 후 재부팅하세요.")
    else:
        print("ESP32 시리얼 연결 실패 (통신 없이 진행합니다). "
              "raspi-config에서 Serial Port 활성화(enable_uart=1) 여부를 확인하세요.")
    return None

# ====================================================================
# [예외 처리] pyserial 모듈 확인
# ====================================================================
try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("[경고] 'pyserial' 모듈이 설치되어 있지 않아 시리얼 통신 기능이 비활성화됩니다.")

# ====================================================================
# [함수 1] 각도 분석 함수 (Gray Edge + Yellow Edge 병합)
# ====================================================================
def analyze_crosswalk_angle(roi, offset_x, offset_y, frame, state, alpha=0.2):
    cw_direction = state['direction']
    ema_angle = state['ema_angle']
    angle_confidence = 0.0

    if roi.size == 0:
        return on_no_lines(state)

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)
    edges_gray = cv2.Canny(blur, 50, 150)

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv_roi, YELLOW_HSV_LOWER, YELLOW_HSV_UPPER)

    kernel = np.ones((3, 3), np.uint8)
    yellow_mask_clean = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    edges_yellow = cv2.Canny(yellow_mask_clean, 50, 150)

    combined_edges = cv2.bitwise_or(edges_gray, edges_yellow)

    lines = cv2.HoughLinesP(combined_edges, 1, np.pi/180, 20, minLineLength=15, maxLineGap=20)

    angles, lengths = [], []
    if lines is not None:
        for line in lines:
            lx1, ly1, lx2, ly2 = line.flatten()
            cv2.line(frame, (offset_x + lx1, offset_y + ly1),
                            (offset_x + lx2, offset_y + ly2), (0, 255, 255), 1)

            line_len = np.hypot(lx2 - lx1, ly2 - ly1)
            if line_len >= 30:
                angle = np.degrees(np.arctan2(ly2 - ly1, lx2 - lx1))
                if angle > 90: angle -= 180
                elif angle < -90: angle += 180

                if -45 <= angle <= 45:
                    angles.append(angle)
                    lengths.append(line_len)

    if angles:
        angles_np = np.array(angles, dtype=np.float32)
        lengths_np = np.array(lengths, dtype=np.float32)

        BIN_SIZE = 5.0
        bins = np.round(angles_np / BIN_SIZE) * BIN_SIZE
        unique_bins, counts = np.unique(bins, return_counts=True)
        main_cluster_index = np.argmax(counts)

        main_cluster_angle = unique_bins[main_cluster_index]
        cluster_mask = np.abs(bins - main_cluster_angle) <= (BIN_SIZE / 2)
        cluster_angles = angles_np[cluster_mask]
        cluster_lengths = lengths_np[cluster_mask]

        if len(cluster_angles) > 0:
            cluster_weights = cluster_lengths / np.sum(cluster_lengths)
            raw_angle = np.sum(cluster_angles * cluster_weights)

            state['no_lines_frames'] = 0

            # EMA 적용
            if not state['initialized']:
                state['ema_angle'] = raw_angle
                state['initialized'] = True
            else:
                state['ema_angle'] = (alpha * raw_angle) + ((1 - alpha) * state['ema_angle'])

            ema_angle = state['ema_angle']

            # 비대칭 문턱값 적용 (Hysteresis)
            current_dir = state['direction']
            if current_dir == "Straight":
                if ema_angle < -ANGLE_ENTER_DEG: state['direction'] = "Turn Right"
                elif ema_angle > ANGLE_ENTER_DEG: state['direction'] = "Turn Left"
            elif current_dir == "Turn Right":
                if ema_angle >= -ANGLE_EXIT_DEG:
                    if ema_angle > ANGLE_ENTER_DEG: state['direction'] = "Turn Left"
                    else: state['direction'] = "Straight"
            elif current_dir == "Turn Left":
                if ema_angle <= ANGLE_EXIT_DEG:
                    if ema_angle < -ANGLE_ENTER_DEG: state['direction'] = "Turn Right"
                    else: state['direction'] = "Straight"

            cw_direction = state['direction']

            # 신뢰도 계산
            cluster_count = len(cluster_angles)
            total_lines = len(angles)
            cluster_ratio = cluster_count / total_lines

            angle_std = np.sqrt(np.average((cluster_angles - raw_angle) ** 2, weights=cluster_lengths))
            angle_consistency = max(0.0, 1.0 - (angle_std / 10.0))
            angle_confidence = (cluster_ratio * 0.6 + angle_consistency * 0.4) * 100.0

            return cw_direction, ema_angle, angle_confidence

    return on_no_lines(state)

# ====================================================================
# [함수 2] 신호등 색상 스코어 계산 함수
# ====================================================================
def calculate_color_score(hsv_roi, mask, total_pixels):
    pixel_count = cv2.countNonZero(mask)
    if pixel_count == 0 or total_pixels == 0:
        return 0.0, 0.0, 0.0, 0.0

    ratio = pixel_count / total_pixels
    if ratio <= 0.03:
        return 0.0, 0.0, 0.0, 0.0

    mean_val = cv2.mean(hsv_roi, mask=mask)
    mean_s = mean_val[1]
    mean_v = mean_val[2]

    s_score = max(0.0, (mean_s - 60.0) / 195.0)
    v_score = mean_v / 255.0

    score = ((0.40 * ratio) + (0.30 * s_score) + (0.30 * v_score)) * 1.5
    return score, ratio, s_score, v_score

# ====================================================================
# [메인 실행 코드] 초기화
# ====================================================================
print("모델 로딩 중...")
model = YOLO('best2_ncnn_model', task='detect')
print("로딩 완료!")

esp_serial = None
if HAS_SERIAL:
    # Pi 5는 /dev/ttyAMA0, Pi 4는 /dev/serial0. USB 연결이면 /dev/ttyUSB0.
    esp_serial = open_esp_serial()

esp_link = EspLink(esp_serial)

# PiCamera2 초기화
picam2 = None
if HAS_PICAM2:
    try:
        picam2 = Picamera2()
        # OpenCV 처리에 알맞게 BGR 형식으로 해상도(640x480) 설정
        config = picam2.create_video_configuration(main={"size": (640, 480)})
        picam2.configure(config)
        picam2.start()
        print("라즈베리파이 카메라 모듈 3 초기화 완료!")
    except Exception as e:
        print(f"카메라 모듈 초기화 실패: {e}")
        exit()
else:
    exit()

print("--------------------------------------------------------------------------------")
print(f"통합 AI 시작 (라즈베리파이 5 + Picamera2, YOLO 입력 {YOLO_IMGSZ})")
print("화면 클릭 후 'q'를 누르면 종료됩니다.")
print("--------------------------------------------------------------------------------")

# 상태 제어 글로벌 변수
event = "NORMAL"

# 신호등 히스테리시스 변수
stable_color = "Unknown"
candidate_color = "Unknown"
candidate_frames = 0
tl_missing_frames = 0
MIN_SCORE = 0.18

# 횡단보도 상태 변수
cw_close_frames = 0
red_frames = 0
green_frames = 0
missing_cw_frames = 0
cw_last_seen_time = 0.0
has_seen_red = False
state_crosswalk = new_crosswalk_state()
escape_guard = CrossingEscapeGuard()
cw_width_ratio = 0.0
steering_source = 'NONE'
center_offset_ratio = 0.0

try:
    while True:
        t_start = time.time()

        # PiCamera2에서 프레임 읽어오기
        try:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        except RuntimeError as e:
            print(f"프레임 캡처 에러: {e}")
            break

        frame_h, frame_w = frame.shape[:2]
        frame_center_x = frame_w // 2
        bottom_40_percent_y = int(frame_h * 0.4)

        results = model(frame, imgsz=YOLO_IMGSZ, stream=False, verbose=False)
        boxes = results[0].boxes

        # ====================================================================
        # [단계 1] 가장 큰 객체(횡단보도, 신호등, 바닥신호등) 추출
        # ====================================================================
        largest_cw_box = None
        max_cw_area = 0
        largest_tl_box = None
        max_tl_area = 0
        largest_gl_box = None
        max_gl_area = 0

        cw_direction = "None"
        is_cw_close = False
        angle_cw, conf_cw = 0.0, 0.0
        all_detections = []

        if len(boxes) > 0:
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_name = model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                box_area = (x2 - x1) * (y2 - y1)

                all_detections.append((x1, y1, x2, y2, cls_name, conf))

                if cls_name == 'cross-walk':
                    if box_area > max_cw_area:
                        max_cw_area = box_area
                        largest_cw_box = (x1, y1, x2, y2)

                elif cls_name == 'traffic_light':
                    if box_area > max_tl_area:
                        max_tl_area = box_area
                        largest_tl_box = (x1, y1, x2, y2)

                elif cls_name == 'ground_light':
                    if box_area > max_gl_area:
                        max_gl_area = box_area
                        largest_gl_box = (x1, y1, x2, y2)

        # ====================================================================
        # [단계 2] 가장 큰 횡단보도 방향 분석
        # ====================================================================
        crossing_now = event in CROSSING_EVENTS
        cw_width_ratio = 0.0
        center_offset_ratio = 0.0
        steering_source = 'NONE'

        if largest_cw_box is not None:
            cx1, cy1, cx2, cy2 = largest_cw_box
            # 비정상 좌표를 ROI/비율 계산에 사용하지 않도록 화면 안으로 제한.
            cx1, cx2 = max(0, cx1), min(frame_w, cx2)
            cy1, cy2 = max(0, cy1), min(frame_h, cy2)
            largest_cw_box = (cx1, cy1, cx2, cy2)
            box_center_x = (cx1 + cx2) / 2.0
            center_offset_ratio = (box_center_x - frame_center_x) / frame_w
            cw_width_ratio = max(0, cx2 - cx1) / frame_w

            # 중심 보정을 하더라도 선 미검출 카운터는 매 프레임 갱신합니다.
            # 각도 state['direction']에는 위치/복귀 조향을 저장하지 않습니다.
            roi = frame[cy1:cy2, cx1:cx2]
            if roi.size != 0:
                roi_h = roi.shape[0]
                start_y = int(roi_h * 0.15)
                end_y = int(roi_h * 0.90)
                roi_target = roi[start_y:end_y, :]
                cv2.rectangle(frame, (cx1, cy1 + start_y),
                              (cx2, cy1 + end_y), (255, 0, 255), 1)
                dir_cw, angle_cw, conf_cw = analyze_crosswalk_angle(
                    roi_target, cx1, cy1 + start_y, frame, state_crosswalk, alpha=0.2)
            else:
                dir_cw, angle_cw, conf_cw = on_no_lines(state_crosswalk)

            if state_crosswalk['centering']:
                centering = abs(center_offset_ratio) > CENTER_EXIT_RATIO
            else:
                centering = abs(center_offset_ratio) > CENTER_ENTER_RATIO
            state_crosswalk['centering'] = centering

            recovery = escape_guard.update(crossing_now, cw_width_ratio,
                                           center_offset_ratio)
            if recovery is not None:
                image_direction = recovery
                steering_source = ('RECOVER' if escape_guard.recovery_direction
                                   else 'CHECK')
            elif centering:
                image_direction = 'Turn Left' if center_offset_ratio < 0 else 'Turn Right'
                steering_source = 'CENTER'
            else:
                # 선 미검출 3프레임이면 dir_cw에 Straight가 들어 있습니다.
                image_direction = ('Turn Left' if 'Turn Left' in dir_cw else
                                   'Turn Right' if 'Turn Right' in dir_cw else 'Straight')
                # 선이 잠시 누락되어 유지된 각도 조향에도 같은 반전을 적용합니다.
                image_direction = motor_direction(image_direction)
                steering_source = ('NO_LINE' if state_crosswalk['no_lines_frames'] else 'ANGLE')
            # 중앙 보정 및 이탈 복귀는 영상에서 판단한 방향을 그대로 사용합니다.
            cw_direction = image_direction

            # 기존 접근 감지 조건은 유지합니다. 실제 진입 확인 조건은 아닙니다.
            is_cw_close = cy2 >= bottom_40_percent_y
        else:
            recovery = escape_guard.update(crossing_now, None, None)
            state_crosswalk['centering'] = False
            _, angle_cw, conf_cw = on_no_lines(state_crosswalk)
            # 검출 소실 시의 추정 복귀도 좌우 반전하지 않습니다.
            cw_direction = recovery if recovery is not None else 'None'
            steering_source = ('LOST_REC' if recovery in ['Turn Left', 'Turn Right'] else 'NO_BOX')

        # ====================================================================
        # [단계 3] 신호등 색상 스코어 기반 추출
        # ====================================================================
        detected_color = "Unknown"
        selected_light = None
        r_score, g_score = 0.0, 0.0
        r_detail, g_detail = "", ""

        if event in ["CROSSWALK_AHEAD", "WAIT_SIGNAL"]:
            if largest_tl_box is not None: selected_light = largest_tl_box
            elif largest_gl_box is not None: selected_light = largest_gl_box

            if selected_light is not None:
                l_x1, l_y1, l_x2, l_y2 = selected_light
                roi = frame[l_y1:l_y2, l_x1:l_x2]
                h, w = roi.shape[:2]

                if h >= 5 and w >= 5:
                    total_pixels = h * w
                    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                    mask_red = cv2.bitwise_or(
                        cv2.inRange(hsv_roi, np.array([0,50,50]), np.array([5,255,255])),
                        cv2.inRange(hsv_roi, np.array([160,50,50]), np.array([180,255,255]))
                    )
                    mask_green = cv2.inRange(hsv_roi, np.array([35,45,45]), np.array([100,255,255]))

                    r_score, r_ratio, r_s, r_v = calculate_color_score(hsv_roi, mask_red, total_pixels)
                    g_score, g_ratio, g_s, g_v = calculate_color_score(hsv_roi, mask_green, total_pixels)

                    if r_score >= MIN_SCORE and r_score > g_score: detected_color = "RED"
                    elif g_score >= MIN_SCORE and g_score > r_score: detected_color = "GREEN"

                    r_detail = f"R(rt:{r_ratio:.2f} s:{r_s:.2f} v:{r_v:.2f})"
                    g_detail = f"G(rt:{g_ratio:.2f} s:{g_s:.2f} v:{g_v:.2f})"

        # --- 신호등 색상 Hysteresis & Missing 로직 ---
        if selected_light is None:
            if event in ["CROSSWALK_AHEAD", "WAIT_SIGNAL"]:
                tl_missing_frames += 1
                if tl_missing_frames >= 20:
                    stable_color = "Unknown"
                    candidate_color = "Unknown"
                    candidate_frames = 0
        else:
            tl_missing_frames = 0
            if detected_color != "Unknown":
                if stable_color == "Unknown":
                    stable_color = detected_color
                else:
                    if detected_color == stable_color:
                        candidate_frames = max(0, candidate_frames - 2)
                    else:
                        if candidate_color != detected_color:
                            candidate_color = detected_color
                            candidate_frames = 1
                        else:
                            candidate_frames += 1

                        if candidate_frames >= 10:
                            stable_color = candidate_color
                            candidate_frames = 0

        # ====================================================================
        # [단계 4] 상태 머신 (State Machine) 로직 업데이트
        # ====================================================================
        if event == "NORMAL":
            if is_cw_close:
                cw_close_frames += 1
                if cw_close_frames >= CROSSWALK_CONFIRM_FRAMES:
                    event = "CROSSWALK_AHEAD"
                    red_frames, green_frames, missing_cw_frames = 0, 0, 0
                    has_seen_red = False
                    state_crosswalk = new_crosswalk_state()
                    escape_guard.reset()
                    # 이전 횡단보도에서 본 신호색이 남아 있지 않게 초기화
                    stable_color = "Unknown"
                    candidate_color = "Unknown"
                    candidate_frames = 0
            else:
                cw_close_frames = max(0, cw_close_frames - 1)

        elif event in ["CROSSWALK_AHEAD", "WAIT_SIGNAL"]:
            if largest_cw_box is None:
                missing_cw_frames += 1
                if missing_cw_frames >= 20:
                    event = "NORMAL"
                    cw_close_frames = 0
            else:
                missing_cw_frames = 0

                if stable_color == "RED":
                    red_frames += 1
                    green_frames = 0
                    if red_frames >= 10:
                        event = "WAIT_SIGNAL"
                        has_seen_red = True

                elif stable_color == "GREEN":
                    green_frames += 1
                    red_frames = 0
                    if green_frames >= 10:
                        if has_seen_red:
                            event = "CROSSING_START"
                            missing_cw_frames = 0
                            cw_last_seen_time = time.time()
                            state_crosswalk = new_crosswalk_state()
                            escape_guard.reset()
                        else:
                            event = "WAIT_SIGNAL"

        elif event in CROSSING_EVENTS:
            if largest_cw_box is None:
                missing_cw_frames += 1
                # 소실 직전 근거가 확인된 경우에만 짧은 복귀를 우선합니다.
                if cw_direction == 'Turn Left':
                    event = 'LEFT_CORRECTION'
                elif cw_direction == 'Turn Right':
                    event = 'RIGHT_CORRECTION'
                elif missing_cw_frames >= NO_LINES_STRAIGHT_FRAMES or cw_direction == 'Straight':
                    event = 'STRAIGHT'
                # 짧은 미검출은 이탈일 수 있으므로 바로 끝내지 않고 복귀/중립 안내를 먼저 합니다.
                if missing_cw_frames >= POSITION_UNKNOWN_FRAMES:
                    event = 'CROSSING_UNKNOWN'
                # 일정 시간 동안 횡단보도가 전혀 안 보이면 횡단 종료 -> 일반 길안내 복귀.
                # 그 전에 다시 보이면 missing이 초기화되어 안내를 계속한다.
                if time.time() - cw_last_seen_time >= CROSSING_END_MISSING_SEC:
                    event = 'CROSSING_END'
                    print(f"[횡단 종료] 횡단보도 {CROSSING_END_MISSING_SEC:.0f}초 미검출")
            else:
                missing_cw_frames = 0
                cw_last_seen_time = time.time()
                if cw_direction == "Turn Left": event = "LEFT_CORRECTION"
                elif cw_direction == "Turn Right": event = "RIGHT_CORRECTION"
                else: event = "STRAIGHT"

        # ====================================================================
        # [데이터 전송] ESP32 통신
        # ====================================================================
        # 내부 상태 이름을 ESP32/앱 공통 메시지로 바꿔서 보낸다.
        # 횡단보도 접근/신호 대기/건너는 중에는 같은 메시지도 주기적으로 반복 전송한다.
        esp_link.update(
            esp_messages_for_event(event, stable_color, has_seen_red),
            resend_sec=esp_resend_interval(event),
            now=time.time(),
        )

        # CROSSING_START/END 는 1틱용 플래그이므로 바로 전환
        if event == "CROSSING_START": event = "STRAIGHT"
        elif event == "CROSSING_END":
            event = "NORMAL"
            cw_close_frames = 0
            missing_cw_frames = 0
            has_seen_red = False
            state_crosswalk = new_crosswalk_state()
            escape_guard.reset()

        # ====================================================================
        # [단계 5] 화면 출력 (HUD & Bounding Boxes)
        # ====================================================================
        for det in all_detections:
            dx1, dy1, dx2, dy2, dcls, dconf = det
            cv2.rectangle(frame, (dx1, dy1), (dx2, dy2), (150, 150, 150), 1)
            cv2.putText(frame, f"{dcls} {dconf:.2f}", (dx1, dy1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        if largest_cw_box is not None:
            cx1, cy1, cx2, cy2 = largest_cw_box
            cv2.rectangle(frame, (cx1, cy1), (cx2, cy2), (255, 0, 0), 2)
            cv2.putText(frame, f"Dir: {cw_direction}", (cx1, cy1 - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.putText(frame, f"Ang: {angle_cw:.1f} (Conf:{conf_cw:.1f}%)", (cx1, cy1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        if selected_light is not None:
            lx1, ly1, lx2, ly2 = selected_light
            color_bgr = (0,0,255) if stable_color=="RED" else (0,255,0) if stable_color=="GREEN" else (0,255,255)
            cv2.rectangle(frame, (lx1, ly1), (lx2, ly2), color_bgr, 2)
            cv2.putText(frame, f"Signal: {stable_color}", (lx1, ly1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)

            cv2.putText(frame, f"R:{r_score:.2f} G:{g_score:.2f}", (lx1, ly2 + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, r_detail, (lx1, ly2 + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 255), 1)
            cv2.putText(frame, g_detail, (lx1, ly2 + 47), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)

        cv2.rectangle(frame, (5, 5), (245, 102), (0, 0, 0), -1)
        t_end = time.time()
        fps = 1.0 / (t_end - t_start) if (t_end - t_start) > 0 else 0.0

        hud_font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"FPS:{fps:.1f}  {event}", (9, 21), hud_font, 0.40, (0, 255, 255), 1)
        # 횡단보도 접근 확인 진행도(CW n/5)도 함께 표시. 다 차면 CROSSWALK:1 전송.
        cv2.putText(frame, f"Color:{stable_color}  CW:{cw_close_frames}/{CROSSWALK_CONFIRM_FRAMES}",
                    (9, 39), hud_font, 0.40, (255, 255, 255), 1)
        cv2.putText(frame, f"Cand:{candidate_color} {candidate_frames}/10", (9, 57), hud_font, 0.38, (200, 200, 200), 1)
        cv2.putText(frame, f"TL miss:{tl_missing_frames}/20  Red:{has_seen_red}", (9, 75), hud_font, 0.36, (0, 165, 255), 1)
        # HUD 외곽(245x102)은 그대로 유지. ESC 옆 작은 점: 빨강 OFF / 초록 ON.
        cv2.putText(frame, f"W:{cw_width_ratio:.0%} N:{state_crosswalk['no_lines_frames']} {steering_source}",
                    (9, 93), hud_font, 0.34, (200, 200, 200), 1)
        cv2.putText(frame, "ESC", (202, 93), hud_font, 0.30, (200, 200, 200), 1)
        dot_color = (0, 255, 0) if escape_guard.armed else (0, 0, 255)
        cv2.circle(frame, (235, 89), 4, dot_color, -1)

        cv2.imshow('Pi5 + Camera Mod 3 AI', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n사용자에 의해 프로그램을 종료합니다.")

finally:
    # 윈도우 창 닫기
    cv2.destroyAllWindows()

    # PiCamera2 리소스 반환
    if 'picam2' in locals() and picam2 is not None:
        try:
            picam2.stop()
            picam2.close()
            print("카메라 모듈 리소스가 정상적으로 해제되었습니다.")
        except Exception:
            pass

    # 시리얼 포트 닫기
    if 'esp_serial' in locals() and esp_serial is not None:
        try:
            esp_serial.close()
            print("ESP32 시리얼 포트가 닫혔습니다.")
        except Exception:
            pass
