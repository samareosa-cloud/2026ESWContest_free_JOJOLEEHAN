import cv2
import numpy as np
import time
from ultralytics import YOLO


# PiCamera2
try:
    from picamera2 import Picamera2
    HAS_PICAM2 = True
except ImportError:
    HAS_PICAM2 = False
    print("[오류] 'picamera2' 모듈이 없습니다. 라즈베리파이 OS 환경을 확인해주세요.")


# 한국 어린이 보호구역 노란색 횡단보도 검출을 위한 HSV 설정
YELLOW_HSV_LOWER = np.array([15, 60, 80])
YELLOW_HSV_UPPER = np.array([45, 255, 255])


try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("[경고] 'serial' 모듈이 설치되어 있지 않아 시리얼 통신 기능이 비활성화됩니다.")


# 각도 분석 함수
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
                if ema_angle < -3.0: state['direction'] = "Turn Right"
                elif ema_angle > 3.0: state['direction'] = "Turn Left"
            elif current_dir == "Turn Right":
                if ema_angle >= -2.0:
                    if ema_angle > 3.0: state['direction'] = "Turn Left"
                    else: state['direction'] = "Straight"
            elif current_dir == "Turn Left":
                if ema_angle <= 3.0:
                    if ema_angle < -2.0: state['direction'] = "Turn Right"
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


#신호등 색상 검출 함수
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


#메인 코드
model = YOLO('best_ncnn_model', task='detect') 

esp_serial = None
if HAS_SERIAL:
    serial_port = '/dev/serial0' 
    try:
        esp_serial = serial.Serial(serial_port, 115200, timeout=1)
        print("ESP32 시리얼 통신 연결")
    except Exception as e:
        print(f"ESP32 시리얼 연결 실패: {e}")


# PiCamera2 설정
picam2 = None
if HAS_PICAM2:
    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"size": (640, 480), "format": "bgr888"})
        picam2.configure(config)
        picam2.start()
        print("라즈베리파이 카메라 모듈 3 초기화 완료")
    except Exception as e:
        print(f"카메라 모듈 초기화 실패: {e}")
        exit()
else:
    exit()

print("--------------------------------------------------------------------------------")
print("시작")
print("--------------------------------------------------------------------------------")

# 상태 제어 글로벌 변수
event = "CROSSWALK:0"
last_sent_event = ""

# 신호등 히스테리시스 변수
stable_color = "Unknown"
candidate_color = "Unknown"
candidate_frames = 0
tl_missing_frames = 0
MIN_SCORE = 0.18

# 횡단보도 상태 변수
cw_straight_frames = 0
red_frames = 0
green_frames = 0
missing_cw_frames = 0
has_seen_red = False 
state_crosswalk = {'ema_angle': 0.0, 'direction': 'Straight', 'initialized': False}

try:
    while True:
        t_start = time.time()
        
        try:
            frame = picam2.capture_array()
        except RuntimeError as e:
            print(f"프레임 디버깅 캡처 오류: {e}")
            break
            
        frame_h, frame_w = frame.shape[:2]
        frame_center_x = frame_w // 2
        center_margin_cw = int(frame_w * 0.25) 
        bottom_60_percent_y = int(frame_h * 0.6) 
        
        results = model(frame, stream=False, verbose=False)
        boxes = results[0].boxes

        
        #1단계 타깃 박스 추출
        largest_cw_box = None
        max_cw_area = 0
        largest_tl_box = None
        max_tl_area = 0
        largest_gl_box = None
        max_gl_area = 0
        
        cw_direction = "None"
        is_cw_straight_and_close = False
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


        # 2단계 횡단보도 방향 분석
        if largest_cw_box is not None:
            cx1, cy1, cx2, cy2 = largest_cw_box
            box_center_x = (cx1 + cx2) // 2
            
            if box_center_x < (frame_center_x - center_margin_cw): 
                cw_direction = "Turn Left"
                state_crosswalk['direction'] = "Turn Left"
            elif box_center_x > (frame_center_x + center_margin_cw): 
                cw_direction = "Turn Right"
                state_crosswalk['direction'] = "Turn Right"
            else:
                roi = frame[cy1:cy2, cx1:cx2]
                if roi.size != 0:
                    roi_h = roi.shape[0]
                    start_y = int(roi_h * 0.26)
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
            
            if cw_direction == "Straight" and cy2 >= bottom_60_percent_y:
                is_cw_straight_and_close = True

        #신호등 색상 검출
        detected_color = "Unknown" 
        selected_light = None
        r_score, g_score = 0.0, 0.0
        r_detail, g_detail = "", ""
        
        if event in ["CROSSWALK:1", "WAIT_SIGNAL"]:
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

        # 신호등 색상 스택 관련 코드
        if selected_light is None:
            if event in ["CROSSWALK:1", "WAIT_SIGNAL"]:
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

        #4단계 상태머신 업데이트
        if event == "CROSSWALK:0":
            if is_cw_straight_and_close:
                cw_straight_frames += 1
                if cw_straight_frames >= 10:
                    event = "CROSSWALK:1"
                    red_frames, green_frames, missing_cw_frames = 0, 0, 0
                    has_seen_red = False 
            else:
                cw_straight_frames = max(0, cw_straight_frames - 1)

        elif event in ["CROSSWALK:1", "WAIT_SIGNAL"]:
            if largest_cw_box is None:
                missing_cw_frames += 1
                if missing_cw_frames >= 20:
                    event = "CROSSWALK:0"
                    cw_straight_frames = 0 
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

        elif event in ["CROSS_MOTOR:CENTER", "CROSS_MOTOR:L", "CROSS_MOTOR:R"]:
            if largest_cw_box is None:
                missing_cw_frames += 1
                if missing_cw_frames >= 20: 
                    event = "CROSSING_END"
            else:
                missing_cw_frames = 0
                if cw_direction == "Turn Left": event = "CROSS_MOTOR:L"
                elif cw_direction == "Turn Right": event = "CROSS_MOTOR:R"
                else: event = "CROSS_MOTOR:CENTER"

        # ESP32 데이터 전송
        # WAIT_SIGNAL은 Raspberry Pi 내부 상태로만 사용하고,
        # ESP32에는 실제 신호등 색을 전송한다.

        if event == "WAIT_SIGNAL":

            if stable_color == "RED":
                send_event = "LIGHT:RED"
                event_key = "WAIT_SIGNAL:RED"

            elif stable_color == "GREEN":
                send_event = "LIGHT:GREEN"
                event_key = "WAIT_SIGNAL:GREEN"

            else:
                send_event = "LIGHT:NONE"
                event_key = "WAIT_SIGNAL:NONE"

            if event_key != last_sent_event:
                print(f"현재 상태: {event}")

                if esp_serial is not None:
                    try:
                        data_to_send = f"{send_event}\n"
                        esp_serial.write(data_to_send.encode("utf-8"))

                        print(
                            f" ESP32 데이터 전송 완료: {send_event}"
                        )

                    except Exception as e:
                        print(
                            f" ESP32 데이터 전송 실패: {e}"
                        )
                else:
                    print("ESP32 미연결. 시리얼 전송 생략")

                last_sent_event = event_key


        elif event == "CROSSING_START":

            if event != last_sent_event:
                print(f"현재 상태: {event}")

                if esp_serial is not None:
                    try:
                        # 초록불 먼저 전송
                        esp_serial.write(b"LIGHT:GREEN\n")
                        print(
                            " ESP32 데이터 전송 완료: LIGHT:GREEN"
                        )

                        time.sleep(0.05)

                        # 그 다음 횡단 시작
                        esp_serial.write(b"CROSSING_START\n")
                        print(
                            " ESP32 데이터 전송 완료: CROSSING_START"
                        )

                    except Exception as e:
                        print(
                            f" ESP32 데이터 전송 실패: {e}"
                        )
                else:
                    print("ESP32 미연결. 시리얼 전송 생략")

                last_sent_event = event


        else:

            if event != last_sent_event:
                print(f"현재 상태: {event}")

                if esp_serial is not None:
                    try:
                        data_to_send = f"{event}\n"
                        esp_serial.write(data_to_send.encode("utf-8"))

                        print(
                            f" ESP32 데이터 전송 완료: {event}"
                        )

                    except Exception as e:
                        print(
                            f" ESP32 데이터 전송 실패: {e}"
                        )
                else:
                    print("ESP32 미연결. 시리얼 전송 생략")

                last_sent_event = event


                
        # CROSSING_START/END 는 1틱용 플래그이므로 바로 전환
        if event == "CROSSING_START": event = "CROSS_MOTOR:CENTER" 
        elif event == "CROSSING_END": event = "CROSSWALK:0"   

        # 5단계 디버깅용 화면 출력
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

        cv2.rectangle(frame, (5, 5), (280, 100), (0, 0, 0), -1) 
        t_end = time.time()
        fps = 1.0 / (t_end - t_start) if (t_end - t_start) > 0 else 0.0
        
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"EVENT: {event}", (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2) 
        cv2.putText(frame, f"Color: {stable_color}", (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Cand: {candidate_color} ({candidate_frames}/10)", (10, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        cv2.imshow('Pi4 + Camera Mod 3 AI', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n프로그램 종료.")

#종료
finally:
    cv2.destroyAllWindows()
    
    if 'picam2' in locals() and picam2 is not None:
        try:
            picam2.stop()
            picam2.close()
        except Exception:
            pass
            
    if 'esp_serial' in locals() and esp_serial is not None:
        try:
            esp_serial.close()
        except Exception:
            pass
