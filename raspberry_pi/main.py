import cv2
import numpy as np
import time
from ultralytics import YOLO

# ====================================================================
# [추가] PiCamera2 모듈 임포트q
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

# 카메라를 180도 돌려 달아서 좌/우를 뒤집어 보낸다.
# 시연 전 꼭 확인: 횡단보도가 카메라 기준 왼쪽에 있을 때
# 지팡이가 왼쪽으로 당겨야 정상. 반대로 당기면 False로 바꾼다.
FLIP_LEFT_RIGHT = True

# ====================================================================
# [ESP32 전송] 메시지 이름은 docs/PROTOCOL.md 기준 (ESP32/앱과 공통)
# ====================================================================
# 건너는 동안 현재 보정 방향(CROSS_MOTOR:L/R/CENTER)을 이 간격(초)으로
# 계속 다시 보낸다. ESP32는 1초 동안 소식이 없으면 스스로 멈춘다.
CROSS_RESEND_SEC = 0.3

CROSSING_EVENTS = ["STRAIGHT", "LEFT_CORRECTION", "RIGHT_CORRECTION"]


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
    if event == "STRAIGHT":
        return ["CROSS_MOTOR:CENTER"]
    if event == "LEFT_CORRECTION":
        return ["CROSS_MOTOR:L"]
    if event == "RIGHT_CORRECTION":
        return ["CROSS_MOTOR:R"]
    if event == "CROSSING_END":
        return ["CROSSING_END"]
    return []


class EspLink:
    """바뀐 메시지는 바로, 횡단 중 보정 메시지는 주기적으로 다시 보낸다."""

    def __init__(self, ser):
        self.ser = ser
        self.last_messages = None
        self.last_sent_time = 0.0

    def update(self, messages, repeat, now):
        changed = messages != self.last_messages
        resend = repeat and (now - self.last_sent_time) >= CROSS_RESEND_SEC

        if not messages or (not changed and not resend):
            return []

        if changed:
            print(f"[ESP32 전송] {' / '.join(messages)}")

        if self.ser is not None:
            try:
                for message in messages:
                    self.ser.write(f"{message}\n".encode("utf-8"))
            except Exception as e:
                print(f" └-> [ESP32 데이터 전송 실패]: {e}")

        self.last_messages = messages
        self.last_sent_time = now
        return messages

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
        return f"{cw_direction} (No Lines)", ema_angle, angle_confidence

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

    return f"{cw_direction} (No Lines)", ema_angle, angle_confidence

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
    # 라즈베리파이 환경에 맞춰 시리얼 포트 변경 (USB: /dev/ttyUSB0, GPIO 핀: /dev/ttyS0 또는 /dev/ttyAMA0)
    serial_port = '/dev/ttyAMA0'
    try:
        esp_serial = serial.Serial(serial_port, 115200, timeout=1)
        print("ESP32 시리얼 통신 연결 성공!")
    except Exception as e:
        print(f"ESP32 시리얼 연결 실패 (통신 없이 진행합니다): {e}")

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
print("통합 AI 시작 (라즈베리파이 4 + Picamera2 버전)")
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
has_seen_red = False
state_crosswalk = {'ema_angle': 0.0, 'direction': 'Straight', 'initialized': False, 'centering': False}
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

        results = model(frame, stream=False, verbose=False)
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
        if largest_cw_box is not None:
            cx1, cy1, cx2, cy2 = largest_cw_box
            box_center_x = (cx1 + cx2) // 2

            # 횡단보도가 화면 한쪽으로 치우쳐 있으면(= 사용자가 횡단보도
            # 한쪽 끝에 있음) 먼저 횡단보도 중앙 쪽으로 보낸다.
            center_offset_ratio = (box_center_x - frame_center_x) / frame_w
            if state_crosswalk['centering']:
                centering = abs(center_offset_ratio) > CENTER_EXIT_RATIO
            else:
                centering = abs(center_offset_ratio) > CENTER_ENTER_RATIO
            state_crosswalk['centering'] = centering

            if centering and center_offset_ratio < 0:
                cw_direction = "Turn Left"
                state_crosswalk['direction'] = "Turn Left"
            elif centering:
                cw_direction = "Turn Right"
                state_crosswalk['direction'] = "Turn Right"
            else:
                roi = frame[cy1:cy2, cx1:cx2]
                if roi.size != 0:
                    roi_h = roi.shape[0]
                    start_y = int(roi_h * 0.15)  # 횡단보도 박스 상단 15% 제외
                    end_y = int(roi_h * 0.90)
                    roi_target = roi[start_y:end_y, :]

                    cv2.rectangle(frame, (cx1, cy1 + start_y), (cx2, cy1 + end_y), (255, 0, 255), 1)

                    dir_cw, angle_cw, conf_cw = analyze_crosswalk_angle(
                        roi=roi_target,
                        offset_x=cx1,
                        offset_y=cy1 + start_y,
                        frame=frame,
                        state=state_crosswalk,
                        alpha=0.2
                    )

                    if "Turn Left" in dir_cw: cw_direction = "Turn Left"
                    elif "Turn Right" in dir_cw: cw_direction = "Turn Right"
                    else: cw_direction = "Straight"

            # 화면에 표시하고 ESP32 보정에 사용할 좌우 방향을 반대로 적용합니다.
            # 내부 각도 추적 상태는 원래 기준으로 유지합니다.
            if FLIP_LEFT_RIGHT:
                if cw_direction == "Turn Left":
                    cw_direction = "Turn Right"
                elif cw_direction == "Turn Right":
                    cw_direction = "Turn Left"

            # 방향과 관계없이 박스 아래쪽이 화면 높이의 40%에 닿으면 진입 후보입니다.
            if cy2 >= bottom_40_percent_y:
                is_cw_close = True

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
                if cw_close_frames >= 10:
                    event = "CROSSWALK_AHEAD"
                    red_frames, green_frames, missing_cw_frames = 0, 0, 0
                    has_seen_red = False
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
                        else:
                            event = "WAIT_SIGNAL"

        elif event in ["STRAIGHT", "LEFT_CORRECTION", "RIGHT_CORRECTION"]:
            if largest_cw_box is None:
                missing_cw_frames += 1
                if missing_cw_frames >= 20:
                    event = "CROSSING_END"
            else:
                missing_cw_frames = 0
                if cw_direction == "Turn Left": event = "LEFT_CORRECTION"
                elif cw_direction == "Turn Right": event = "RIGHT_CORRECTION"
                else: event = "STRAIGHT"

        # ====================================================================
        # [데이터 전송] ESP32 통신
        # ====================================================================
        # 내부 상태 이름을 ESP32/앱 공통 메시지로 바꿔서 보낸다.
        # 건너는 중에는 보정 방향을 CROSS_RESEND_SEC마다 반복 전송한다.
        esp_link.update(
            esp_messages_for_event(event, stable_color, has_seen_red),
            repeat=event in CROSSING_EVENTS,
            now=time.time(),
        )

        # CROSSING_START/END 는 1틱용 플래그이므로 바로 전환
        if event == "CROSSING_START": event = "STRAIGHT"
        elif event == "CROSSING_END": event = "NORMAL"

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
        cv2.putText(frame, f"Color:{stable_color}", (9, 39), hud_font, 0.40, (255, 255, 255), 1)
        cv2.putText(frame, f"Cand:{candidate_color} {candidate_frames}/10", (9, 57), hud_font, 0.38, (200, 200, 200), 1)
        cv2.putText(frame, f"TL miss:{tl_missing_frames}/20  Red:{has_seen_red}", (9, 75), hud_font, 0.36, (0, 165, 255), 1)
        cv2.putText(frame, f"CW start:40%  off:{center_offset_ratio:+.2f}", (9, 93), hud_font, 0.36, (200, 200, 200), 1)

        cv2.imshow('Pi4 + Camera Mod 3 AI', frame)

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
