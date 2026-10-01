"""라즈베리파이 -> ESP32 UART 연결 점검 스크립트.

사용법:
  python3 uart_test.py               # 설정 점검 + ESP32로 테스트 메시지 전송
  python3 uart_test.py --loopback    # GPIO14(TX)와 GPIO15(RX)를 점퍼선으로 직접
                                     # 연결한 상태에서 라즈베리파이 UART 자체 점검

ESP32 쪽은 Arduino 시리얼 모니터(115200)를 열어 두고
"[PI RX] CROSSWALK:1" / "[PI RX] CROSSWALK:0" 이 찍히는지 본다.
"""

import glob
import grp
import os
import pwd
import sys
import time

BAUD = 115200


def board_model():
    try:
        with open('/proc/device-tree/model') as f:
            return f.read().strip('\x00 \n')
    except OSError:
        return ''


def read_text(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def check_setup():
    model = board_model()
    is_pi5 = 'Raspberry Pi 5' in model
    print(f"보드: {model or '알 수 없음'}")

    print("\n[1] 시리얼 장치")
    for dev in sorted(set(glob.glob('/dev/ttyAMA*') + glob.glob('/dev/ttyS*')
                          + glob.glob('/dev/serial*') + glob.glob('/dev/ttyUSB*'))):
        print(f"  {dev} -> {os.path.realpath(dev)}")

    port = '/dev/ttyAMA0' if is_pi5 else '/dev/serial0'
    if not os.path.exists(port):
        print(f"  !! {port} 없음 -> GPIO14/15 UART가 꺼져 있음")
    else:
        print(f"  GPIO14/15 UART 포트: {port} -> {os.path.realpath(port)}")

    print("\n[2] config.txt")
    config = read_text('/boot/firmware/config.txt') or read_text('/boot/config.txt') or ''
    lines = [l.strip() for l in config.splitlines() if l.strip() and not l.strip().startswith('#')]
    uart_lines = [l for l in lines if 'uart' in l or 'disable-bt' in l]
    print("  " + ("\n  ".join(uart_lines) if uart_lines else "(uart 관련 설정 없음)"))
    if is_pi5 and not any(l.startswith('dtparam=uart0') for l in lines):
        print("  !! Pi 5: dtparam=uart0=on 을 추가하고 재부팅해야 /dev/ttyAMA0 이 생긴다")
    if not is_pi5 and 'enable_uart=1' not in lines:
        print("  !! Pi 4: enable_uart=1 이 필요하다 (raspi-config > Interface > Serial Port)")

    print("\n[3] 시리얼 콘솔 (켜져 있으면 리눅스 로그인 화면이 같은 핀을 차지함)")
    cmdline = read_text('/boot/firmware/cmdline.txt') or read_text('/boot/cmdline.txt') or ''
    consoles = [w for w in cmdline.split() if w.startswith('console=')]
    print("  " + " ".join(consoles))
    if any(c.startswith(('console=serial0', 'console=ttyAMA0', 'console=ttyS0')) for c in consoles):
        print("  !! 시리얼 콘솔이 켜져 있음 -> raspi-config > Interface > Serial Port 에서")
        print("     '로그인 셸 사용' = No, '시리얼 하드웨어 사용' = Yes 후 재부팅")

    print("\n[4] 권한")
    user = pwd.getpwuid(os.getuid()).pw_name
    groups = [g.gr_name for g in grp.getgrall() if user in g.gr_mem]
    groups.append(grp.getgrgid(os.getgid()).gr_name)
    if os.getuid() == 0 or 'dialout' in groups:
        print(f"  {user}: OK")
    else:
        print(f"  !! {user} 가 dialout 그룹이 아님 -> sudo usermod -aG dialout {user} 후 재로그인")

    return port


def open_port(port):
    import serial
    try:
        return serial.Serial(port, BAUD, timeout=0.5, write_timeout=0.5)
    except Exception as e:
        print(f"\n!! {port} 열기 실패: {e}")
        return None


def send_test(port):
    ser = open_port(port)
    if ser is None:
        return
    print(f"\n[5] {port} 로 1초마다 CROSSWALK:1 / CROSSWALK:0 전송 (Ctrl+C 종료)")
    print("    ESP32 시리얼 모니터에 [PI RX] 가 안 찍히면 배선을 확인:")
    print("    Pi 8번 핀(GPIO14 TX) -> ESP32 GPIO18,  Pi 10번 핀(GPIO15 RX) <- ESP32 GPIO19,")
    print("    Pi GND(6번 핀) <-> ESP32 GND  (TX끼리 연결하면 안 됨)")
    on = True
    try:
        while True:
            msg = "CROSSWALK:1" if on else "CROSSWALK:0"
            ser.write(f"{msg}\n".encode())
            ser.flush()
            print(f"  보냄: {msg}")
            on = not on
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


def loopback_test(port):
    ser = open_port(port)
    if ser is None:
        return
    ser.reset_input_buffer()
    ser.write(b"LOOPBACK_TEST\n")
    ser.flush()
    got = ser.readline()
    ser.close()
    if got.strip() == b"LOOPBACK_TEST":
        print("\nOK: 라즈베리파이 UART 정상 -> 문제는 배선/ESP32 쪽")
    else:
        print(f"\n!! 되돌아온 데이터 없음 ({got!r}) -> 포트 설정 또는 핀 연결 문제")


if __name__ == '__main__':
    port = check_setup()
    if '--loopback' in sys.argv:
        loopback_test(port)
    else:
        send_test(port)
