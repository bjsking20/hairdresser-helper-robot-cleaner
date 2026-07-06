# -*- coding: utf-8 -*-
import time
import threading
import queue
from rplidar import RPLidar, RPLidarException
from serial import SerialException
import serial
import math
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QPushButton
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QPainter, QPen, QColor, QFont
from PySide6.QtGui import QPainterPath
import signal
import sys
import socket
import numpy as np
from sklearn.cluster import DBSCAN
from collections import deque

STM_PORT_NAME = '/dev/serial0'
LIDAR_PORT_NAME = '/dev/ttyUSB0'

# === LIDAR low-level helpers (health 생략 + 소프트/하드 리싱크) ===
BAUD = 115200
SER_TIMEOUT = 3.0

# --- 서버 접속 정보 ---
HOST = '0.0.0.0'  # 모든 NIC에서 수신
PORT = 9999              # 포트

# ===== Debug flags =====
DEBUG_CHAIR = False   # [CHAIRDBG] 출력
DEBUG_TRACK = False   # [TRACK]    출력

USE_MOTOR_CTRL = False        # 전원 인가 시 자동 회전하는 A1M8 + 기본 USB 어댑터면 False 권장
WARM_UP_SEC     = 3.0         # 워밍업 (2~3초 권장)
MAX_BUF_MEAS    = 2000        # 내부 노드 버퍼 상한(2000부터 시작, 필요 시 2500~3000)
RETRY_WAIT_SEQ  = (2.0, 3.0, 5.0)  # 연속 실패 시 대기 증가

# Lidar 데이터 수집 퍼블리시 목표 주기(10Hz)
COLLECT_HZ = 10.0
COLLECT_PERIOD = 1.0 / COLLECT_HZ  # = 0.1s

# Lidar 데이터 처리 퍼블리시 목표 주기(15Hz)
PROCESS_HZ = 15.0
PROCESS_PERIOD = 1.0 / PROCESS_HZ  # ≈ 0.0667 s

# GUI 퍼블리시 목표 주기(20Hz)
GUI_HZ = 20.0
GUI_PERIOD = 1.0 / GUI_HZ  

# DBSCAN (단위 mm)
DBSCAN_EPS_MM = 150.0   # 점 사이 거리 임계
DBSCAN_MIN_SAMPLES = 2  # 한 군집 최소 점 수      

# --- [신규] 벽 판정 기준 ---
WALL_MIN_ENDPOINT_DIST_MM = 500.0 # 벽으로 간주할 최소 너비 (mm)
WALL_RMSE_THRESHOLD = 30.0        # 벽으로 간주할 피팅 오차 (mm)  

# 0도를 '정면'이라고 할 때, 화면의 윗쪽(+Y)이 정면이 되도록 +90° 회전
ANGLE_OFFSET_DEG = 0.0
MAX_DIST_MM = 3000
BIN_DEG = 2
MIRROR_X = True

# --- Chair detection thresholds (mm / deg) ---
CHAIR_BEAUTICIAN_EXCLUSION_MM = 200.0 # [신규] 의자 후보가 미용사 중심 cm 이내면 탈락
CHAIR_R_MIN_MM = 190.0
CHAIR_R_MAX_MM = 245.0
CHAIR_RMSE_MAX_MM = 2 # 원 적합 평균오차 허용치
CHAIR_MIN_PTS = 4        # 최소 포인트 수
CHAIR_ARC_SPAN_MIN_DEG = 60.0  # 원호로 볼 최소 각도 범위
CHAIR_ARC_SPAN_MAX_DEG = 90.0  # 최대 각도
CHAIR_ENDPOINT_DIST_MIN_MM = 220.0 # [신규] 끝점 사이 최소 거리 (mm)

# --- Robot kinematics (derived from user spec) ---
# 1.7 s forward with "1,300,1,300,0" moves 0.435 m → speed:
FORWARD_SPEED_MPS  = 0.435 / 1.7              # ≈ 0.255882... m/s
FORWARD_SPEED_MMPS = FORWARD_SPEED_MPS * 1000 # ≈ 255.882... mm/s

# --- 카메라 파라미터: Raspberry Pi Camera Module 3 Wide (IMX708) ---
CAMERA_IMG_W = 320
CAMERA_IMG_H = 320

# FOV: 모듈3 Wide는 대각 120°, 수평 ≈102°, 수직 ≈66° (각도 환산에는 수평 FOV 사용)
CAMERA_HFOV_DEG = 102.0      # ← 각도 환산 핵심 파라미터
CAMERA_VFOV_DEG = 66.0       # (참고용)
CAMERA_DFOV_DEG = 120.0      # (참고용)

# 미용사 각도 허용폭(부채꼴 표시 및 군집 선택 기준) +-
CAMERA_TOL_DEG = 30.0         

# --- 로봇 모터 UART 명령 전역 상수 ---
CMD_CW = "1,100,0,100,0"      # 시계방향
CMD_CCW = "0,100,1,100,0"     # 반시계방향
CMD_FWD = "1,100,1,100,0"     # 전진
CMD_STOP = "1,0,1,0,0"       # 정지
CMD_REV = "0,100,0,100,0"      # 후진
CMD_CLEANER = "1,0,1,0,1"    # 청소 (CLEAN과 동일)


def handle_client(stop_event, conn, addr, clean_controller, track_controller, q_tx, motor_stop_event):
    """클라이언트 1명을 독립 스레드로 처리"""
    conn.settimeout(1.0)
    print(f"[TCP] Client connected: {addr}", flush=True)
    buf = b""

    # 기존 camera_collecting의 상수/헬퍼 그대로 사용
    stop_cmd_str = "1,0,1,0,0"
    cleaner_cmd_str = "1,0,1,0,0" # STOP 버튼

    try:
        while not stop_event.is_set():
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue

            # 연결이 끊긴 경우: 남은 버퍼 1줄 처리 후 종료
            if not data:
                if buf:
                    try:
                        msg = buf.decode('utf-8', 'ignore').strip().lower()
                        if msg == 'clean':
                            print("[TCP] RX 'clean' (last): Starting CLEAN sequence...", flush=True)
                            motor_stop_event.clear()
                            clean_controller.start_sequence()
                        elif msg == 'track':
                            print("[TCP] RX 'track' (last): Starting TRACK sequence...", flush=True)
                            motor_stop_event.clear()
                            track_controller.start_sequence()
                        elif msg == 'stop':
                            print("[TCP] RX 'stop' (last): STOP > 1s wait > CLEANER...", flush=True)
                            motor_stop_event.set()
                            q_put_latest(q_tx, stop_cmd_str)
                            time.sleep(1.0)
                            q_put_latest(q_tx, cleaner_cmd_str)
                        else:
                            parts = msg.split(',')
                            if len(parts) >= 6 and parts[0] == "Beautician":
                                conf = float(parts[1]); cx = float(parts[2]); cy = float(parts[3])
                                bw = float(parts[4]);  bh = float(parts[5])
                                # └ 기존 _pixel_to_angle_deg를 그대로 쓰기 위해 전역/바깥에 있는 함수를 호출합니다.
                                ang_deg = _pixel_to_angle_deg(cx)
                                print(f"[TCP] RX beautician(last): cx={cx} -> angle={ang_deg:.1f}°  (ts={time.time():.3f})", flush=True)
                                camera_latest.set(ang_deg, cx, cy, bw, bh)
                    except Exception as e:
                        print(f"[TCP] tail-parse error: {e}", flush=True)
                    finally:
                        buf = b""
                print(f"[TCP] Client disconnected: {addr}", flush=True)
                break

            # 원시 명령(CLEAN/TRACK/STOP) 즉시 처리 (앱인벤터의 개행 없는 케이스 포함)
            data_stripped = data.strip()
            if data_stripped == b'CLEAN':
                print("[TCP] RX 'CLEAN' (raw): Starting CLEAN sequence...", flush=True)
                motor_stop_event.clear()
                clean_controller.start_sequence()
                continue
            if data_stripped == b'TRACK':
                print("[TCP] RX 'TRACK' (raw): Starting TRACK sequence...", flush=True)
                motor_stop_event.clear()
                track_controller.start_sequence()
                continue
            if data_stripped == b'STOP':
                print("[TCP] RX 'STOP' (raw): STOP > 1s wait > CLEANER...", flush=True)
                motor_stop_event.set()
                q_put_latest(q_tx, stop_cmd_str)
                time.sleep(1.0)
                q_put_latest(q_tx, cleaner_cmd_str)
                continue

            # 라인 단위 처리(Beautician, ... \n)
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                msg = line.decode('utf-8', 'ignore').strip()
                if not msg:
                    continue

                low = msg.lower()
                if low == 'clean':
                    print(f"[TCP] RX 'clean\\n' : Starting CLEAN sequence...", flush=True)
                    motor_stop_event.clear()
                    clean_controller.start_sequence()
                    continue
                if low == 'track':
                    print(f"[TCP] RX 'track\\n' : Starting TRACK sequence...", flush=True)
                    motor_stop_event.clear()
                    track_controller.start_sequence()
                    continue
                if low == 'stop':
                    print(f"[TCP] RX 'stop\\n' : STOP > 1s wait > CLEANER...", flush=True)
                    motor_stop_event.set()
                    q_put_latest(q_tx, stop_cmd_str)
                    time.sleep(1.0)
                    q_put_latest(q_tx, cleaner_cmd_str)
                    continue

                parts = msg.split(',')
                if len(parts) >= 6 and parts[0] == "Beautician":
                    try:
                        conf = float(parts[1]); cx = float(parts[2]); cy = float(parts[3])
                        bw   = float(parts[4]);  bh = float(parts[5])
                        ang_deg = _pixel_to_angle_deg(cx)
                        print(f"[TCP] RX beautician: cx={cx} -> angle={ang_deg:.1f}°  (ts={time.time():.3f})", flush=True)
                        camera_latest.set(ang_deg, cx, cy, bw, bh)
                    except ValueError:
                        pass
                else:
                    # 기타 메시지는 필요 시 로깅
                    pass

    except Exception as e:
        print(f"[TCP] recv error from {addr}: {e}", flush=True)
    finally:
        try: conn.close()
        except: pass


# -*- coding: utf-8 -*-

# --- Camera 데이터 수집 및 라이다 좌표계 각도로 변환 ---
def camera_collecting(stop_event, host, port, clean_controller, track_controller, q_tx, motor_stop_event):
    """    
    수집: TCP로 'Beautician,<conf>,<center_x>,<center_y>,<w>,<h>\n' 형식의 문자열 데이터를 수신
          [추가] 'clean', 'track', 'stop' 문자열 수신 시 GUI 버튼과 동일한 동작 트리거
    처리: center_x 픽셀 좌표를 라이다 좌표계 각도로 변환
    공유: 변환된 각도와 원본 좌표/크기 정보를 camera_latest.set()을 호출하여 전역 상태 변수(camera_latest)에 저장
    """

    def _pixel_to_angle_deg(center_x):
        # x_norm: -1(left)..+1(right)
        x_norm = (center_x - (CAMERA_IMG_W / 2.0)) / (CAMERA_IMG_W / 2.0)
        # 라이다 좌표(좌측 +)에 맞추기 위해 부호 반전
        ang = -x_norm * (CAMERA_HFOV_DEG / 2.0)  # [수정] Y축 대칭 복원 (left=+, right=-)
        # signed angle 유지(-HFOV/2 .. +HFOV/2)
        return ang

    # === 공통 명령 문자열 ===
    stop_cmd_str = CMD_STOP     # "1,0,1,0,0"
    cleaner_cmd_str = CMD_CLEANER # "1,0,1,0,1"

    # === 클라이언트별 처리 함수 ===
    def handle_client(conn, addr):
        conn.settimeout(1.0)
        print(f"[TCP] Client connected: {addr}", flush=True)
        buf = b""

        try:
            while not stop_event.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue

                # --- [A] 연결 종료 시 마지막 1줄 처리 ---
                if not data:
                    if buf:
                        try:
                            msg = buf.decode('utf-8', 'ignore').strip().lower()
                            if msg:
                                if msg == 'clean':
                                    print("[TCP] RX 'clean' (last): Starting CLEAN sequence...", flush=True)
                                    motor_stop_event.clear()
                                    clean_controller.start_sequence()
                                elif msg == 'track':
                                    print("[TCP] RX 'track' (last): Starting TRACK sequence...", flush=True)
                                    motor_stop_event.clear()
                                    track_controller.start_sequence()
                                elif msg == 'stop':
                                    # ################# [수정#블록#1#시작] #################
                                    is_clean_active = (clean_controller._th and clean_controller._th.is_alive())
                                    is_track_active = (track_controller._th and track_controller._th.is_alive())
                                    
                                    if is_clean_active or is_track_active:
                                        motor_stop_event.set()
                                        
                                        # [수정] 모드에 따라 즉시 전송할 명령 결정
                                        if is_clean_active:
                                            # [신규] clean_mode 내부 상태 확인
                                            if clean_controller.is_cleaner_on():
                                                print("[TCP] RX 'stop' (last): CLEAN (ON). Sending CLEANER (1,0,1,0,1).", flush=True)
                                                immediate_cmd = cleaner_cmd_str
                                            else:
                                                print("[TCP] RX 'stop' (last): CLEAN (OFF). Sending STOP (1,0,1,0,0).", flush=True)
                                                immediate_cmd = stop_cmd_str
                                        else: # is_track_active
                                            print("[TCP] RX 'stop' (last): TRACK active. Sending STOP (1,0,1,0,0).", flush=True)
                                            immediate_cmd = stop_cmd_str
                                        
                                        q_put_latest(q_tx, immediate_cmd) # 1. 즉시 전송
                                        time.sleep(1.0) # 2. 1초 대기
                                        
                                        # 3. '복귀 시퀀스' 시작
                                        print("[TCP] RX 'stop' (last): Starting 'Return Sequence'.", flush=True)
                                        track_controller.start_return_sequence()
                                    
                                    else:
                                        print("[TCP] RX 'stop' (last): Ignored (not in CLEAN or TRACK mode).", flush=True)
                                    # ################# [수정#블록#1#종료] #################
                                else:
                                    parts = msg.split(',')
                                    if len(parts) >= 6 and parts[0] == "Beautician":
                                        conf = float(parts[1])
                                        cx = float(parts[2])
                                        cy = float(parts[3])
                                        bw = float(parts[4])
                                        bh = float(parts[5])
                                        ang_deg = _pixel_to_angle_deg(cx)
                                        print(
                                            f"[TCP] RX beautician(last): cx={cx} -> angle={ang_deg:.1f}° (ts={time.time():.3f})",
                                            flush=True)
                                        camera_latest.set(ang_deg, cx, cy, bw, bh)
                        except Exception as e:
                            print(f"[TCP] tail-parse error: {e}", flush=True)
                        finally:
                            buf = b""
                    print(f"[TCP] Client disconnected: {addr}", flush=True)
                    break

                # --- [B] 원시 명령 처리 (앱인벤터용 CLEAN/TRACK/STOP) ---
                data_stripped = data.strip()
                if data_stripped == b'CLEAN':
                    print("[TCP] RX 'CLEAN' (raw): Starting CLEAN sequence...", flush=True)
                    motor_stop_event.clear()
                    clean_controller.start_sequence()
                    continue
                if data_stripped == b'TRACK':
                    print("[TCP] RX 'TRACK' (raw): Starting TRACK sequence...", flush=True)
                    motor_stop_event.clear()
                    track_controller.start_sequence()
                    continue
                if data_stripped == b'STOP':
                    # ################# [수정#블록#2#시작] #################
                    is_clean_active = (clean_controller._th and clean_controller._th.is_alive())
                    is_track_active = (track_controller._th and track_controller._th.is_alive())
                    
                    if is_clean_active or is_track_active:
                        motor_stop_event.set()
                        
                        # [수정] 모드에 따라 즉시 전송할 명령 결정
                        if is_clean_active:
                            # [신규] clean_mode 내부 상태 확인
                            if clean_controller.is_cleaner_on():
                                print("[TCP] RX 'STOP' (raw): CLEAN (ON). Sending CLEANER (1,0,1,0,1).", flush=True)
                                immediate_cmd = cleaner_cmd_str
                            else:
                                print("[TCP] RX 'STOP' (raw): CLEAN (OFF). Sending STOP (1,0,1,0,0).", flush=True)
                                immediate_cmd = stop_cmd_str
                        else: # is_track_active
                            print("[TCP] RX 'STOP' (raw): TRACK active. Sending STOP (1,0,1,0,0).", flush=True)
                            immediate_cmd = stop_cmd_str
                        
                        q_put_latest(q_tx, immediate_cmd) # 1. 즉시 전송
                        time.sleep(1.0) # 2. 1초 대기
                        
                        # 3. '복귀 시퀀스' 시작
                        print("[TCP] RX 'STOP' (raw): Starting 'Return Sequence'.", flush=True)
                        track_controller.start_return_sequence()
                    
                    else:
                        print("[TCP] RX 'STOP' (raw): Ignored (not in CLEAN or TRACK mode).", flush=True)
                    # ################# [수정#블록#2#종료] #################
                    continue

                # --- [C] Beautician 등 개행 포함 메시지 처리 ---
                print(f"[TCP] recv len={len(data)} bytes  first20={data[:20]!r}", flush=True)
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    msg = line.decode('utf-8', 'ignore').strip()
                    if not msg:
                        continue

                    msg_lower = msg.lower()
                    if msg_lower == 'clean':
                        print(f"[TCP] RX 'clean\\n' (from {msg!r}): Starting CLEAN sequence...", flush=True)
                        motor_stop_event.clear()
                        clean_controller.start_sequence()
                        continue
                    if msg_lower == 'track':
                        print(f"[TCP] RX 'track\\n' (from {msg!r}): Starting TRACK sequence...", flush=True)
                        motor_stop_event.clear()
                        track_controller.start_sequence()
                        continue
                    if msg_lower == 'stop':
                        # ################# [수정#블록#3#시작] #################
                        is_clean_active = (clean_controller._th and clean_controller._th.is_alive())
                        is_track_active = (track_controller._th and track_controller._th.is_alive())
                        
                        if is_clean_active or is_track_active:
                            motor_stop_event.set()
                            
                            # [수정] 모드에 따라 즉시 전송할 명령 결정
                            if is_clean_active:
                                # [신규] clean_mode 내부 상태 확인
                                if clean_controller.is_cleaner_on():
                                    print(f"[TCP] RX 'stop\\n' (from {msg!r}): CLEAN (ON). Sending CLEANER (1,0,1,0,1).", flush=True)
                                    immediate_cmd = cleaner_cmd_str # "1,0,1,0,1"
                                else:
                                    print(f"[TCP] RX 'stop\\n' (from {msg!r}): CLEAN (OFF). Sending STOP (1,0,1,0,0).", flush=True)
                                    immediate_cmd = stop_cmd_str    # "1,0,1,0,0"
                            else: # is_track_active
                                print(f"[TCP] RX 'stop\\n' (from {msg!r}): TRACK active. Sending STOP (1,0,1,0,0).", flush=True)
                                immediate_cmd = stop_cmd_str    # "1,0,1,0,0"
                                
                            q_put_latest(q_tx, immediate_cmd) # 1. 즉시 전송
                            time.sleep(1.0) # 2. 1초 대기
                            
                            # 3. '복귀 시퀀스' 시작 (이전과 동일)
                            print(f"[TCP] RX 'stop\\n' (from {msg!r}): Starting 'Return Sequence'.", flush=True)
                            track_controller.start_return_sequence()
                        
                        else:
                            print(f"[TCP] RX 'stop\\n' (from {msg!r}): Ignored (not in CLEAN or TRACK mode).", flush=True)
                        # ################# [수정#블록#3#종료] #################
                        continue

                    # --- Beautician 파싱 ---
                    parts = msg.split(',')
                    if len(parts) >= 6 and parts[0] == "Beautician":
                        try:
                            conf = float(parts[1])
                            cx = float(parts[2])
                            cy = float(parts[3])
                            bw = float(parts[4])
                            bh = float(parts[5])
                            ang_deg = _pixel_to_angle_deg(cx)
                            print(
                                f"[TCP] RX beautician: cx={cx} -> angle={ang_deg:.1f}° (ts={time.time():.3f})",
                                flush=True)
                            camera_latest.set(ang_deg, cx, cy, bw, bh)
                        except ValueError:
                            pass
                    else:
                        # [수정] 기타 메시지 로깅
                        if msg: # 빈 줄이 아니면
                            print(f"[TCP] RX (Other): {msg}")

        except Exception as e:
            print(f"[TCP] recv error from {addr}: {e}", flush=True)
        finally:
            try:
                conn.close()
            except:
                pass

    # === 서버 소켓 초기화 ===
    srv = None
    try:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(8)  # ★ 여러 클라이언트 허용
        srv.settimeout(1.0)
        print(f"[TCP] Listening on {host}:{port} ...", flush=True)

        while not stop_event.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[TCP] accept error: {e}", flush=True)
                time.sleep(0.5)
                continue

            # --- 각 클라이언트별 스레드 생성 ---
            th = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            th.start()

    finally:
        if srv:
            try:
                srv.close()
            except:
                pass
        print("[TCP] 서버 워커 종료")

# --- Lidar 데이터 수집 ---
def lidar_collecting(stop_event, out_queue):
    """
    - 최초 1회 연결만 수행(재접속 없음)
    - 스캔 오류 시: stop() → clear_input() 만 수행하고 즉시 루프 재개
    - COLLECT_PERIOD 주기로 최신 프레임만 퍼블리시 (큐 최신우선)
    """
    lidar = None
    last_pub_ts = 0.0

    try:
        # --- 1) 단순 연결 & 버퍼 비우기 ---
        print("[LIDAR] 연결 시도 중...")
        lidar = RPLidar(LIDAR_PORT_NAME, baudrate=BAUD, timeout=SER_TIMEOUT)
        print("[LIDAR] 연결 완료")

        try:
            lidar.clean_input()
            print("[LIDAR] 버퍼 비움 완료")
        except Exception as e:
            print(f"[LIDAR] clear_input 예외(무시): {e}")

        # --- 2) 스캔 루프 ---
        while not stop_event.is_set():
            try:
                for scan in lidar.iter_scans(max_buf_meas=MAX_BUF_MEAS):
                    if stop_event.is_set():
                        break

                    now = time.perf_counter()
                    if (now - last_pub_ts) >= COLLECT_PERIOD:
                        # 최신만 유지
                        if out_queue.full():
                            try:
                                out_queue.get_nowait()
                            except queue.Empty:
                                pass
                        try:
                            out_queue.put_nowait(scan)
                        except queue.Full:
                            pass
                        last_pub_ts = now

            except (RPLidarException, SerialException) as e:
                print(f"[LIDAR][스캔 오류] {e} → stop() 후 clear_input()로 복구 시도")
                _soft_resync(lidar)
                # 즉시 while 루프로 돌아가 iter_scans 재시작

            except Exception as e:
                print(f"[LIDAR][예기치 않은 스캔 오류] {e} → stop() 후 clear_input()로 복구 시도")
                _soft_resync(lidar)

    except KeyboardInterrupt:
        print("[LIDAR] KeyboardInterrupt 수신")
    finally:
        _close_lidar(lidar)
        print("[LIDAR] 워커 종료")

def _soft_resync(ld):
    """재접속 없이 '멈추기 → 버퍼 비우기'만 수행."""
    if not ld:
        return
    try:
        ld.stop()
    except Exception:
        pass
    try:
        ld.clean_input()
    except Exception:
        pass
    time.sleep(0.05)  # 아주 짧은 쉼

def _close_lidar(ld):
    """종료 시 안전 정리 (모터 제어 없음)."""
    if not ld:
        return
    try:
        ld.stop()
    except Exception:
        pass
    try:
        ld.disconnect()
    except Exception:
        pass

def robot_angle_to_screen_rad(angle_deg: float) -> float:
    """
    로봇 좌표계 각도(정면=0°, +CCW)를 화면 그리기용 라디안으로 변환.
    [본질적 수정] LiDAR 점들과 동일한 좌표계(Y축 대칭만 적용)로 그립니다.
    """
    
    # 1. C1(카메라) 각도(0=Front, +ve=Left)를 라디안으로 변환
    a_rad_robot = math.radians(angle_deg + ANGLE_OFFSET_DEG)

    # 2. GUI 그리기 좌표계(a0)로 변환 (0=Up, +ve=Right(CW))
    # C1(+ve=Left)과 G(+ve=Right)는 부호가 반대이므로 뒤집습니다.
    a_rad_gui = -a_rad_robot
    
    # 3. LiDAR 포인트와 동일하게 MIRROR_X (Y축 대칭) 적용
    # GUI 좌표계에서 Y축 대칭은 각도의 부호를 뒤집습니다. (a = -a)
    if MIRROR_X:
        a_rad_gui = -a_rad_gui
        
    # 4. [수정] 180도 회전 제거
    # a_rad_gui = -a_rad_gui # (이전 수정으로 주석 처리됨)
    # a_rad_gui += math.pi   # [수정] 180도 반대 증상을 해결하기 위해 이 라인을 제거합니다.
    
    # 5. [추가] 요청하신 X축 대칭(각도 반전)을 추가로 적용합니다.
    a_rad_gui = -a_rad_gui
    
    return a_rad_gui



# --- Lidar 데이터 처리 (필터링, 변환, DBSCAN까지 모두 포함) ---
def lidar_processing(stop_event, in_queue, dbs_queue):
    """
    [통합됨]
    in_queue(라이다 원시 스캔)을 받아
    1. 필터링 (내장)
    2. 좌표변환 (내장)
    3. DBSCAN 군집화
    수행 후 결과를 dbs_queue로 보낸다.
    """
    latest_raw = None

    try:
        while not stop_event.is_set():
            # 1. [수정] 큐에서 데이터 가져오기 (데이터가 올 때까지 효율적으로 대기)
            try:
                # 데이터가 올 때까지 0.1초간 대기 (CPU 반납)
                latest_raw = in_queue.get(timeout=0.1) 
            except queue.Empty:
                continue # 0.1초 동안 데이터 없음 -> 루프 재시작

            # [수정] 큐에 쌓인 나머지 데이터 비우고, 가장 최신 것만 사용
            try:
                while True:
                    latest_raw = in_queue.get_nowait()
            except queue.Empty:
                pass # 큐가 비었으면 방금 가져온 latest_raw 사용

            # 2. [핵심 1단계] 필터링, 좌표변환, DBSCAN
            
            # --- 2-1. 필터링 ( _filter_data 로직 내장) ---
            num_bins = int(360 / BIN_DEG) # (전역 상수)
            seen = [False] * num_bins
            filtered = [] 

            for q, angle_deg, dist_mm in latest_raw:
                if q <= 0:
                    continue
                if dist_mm is None or dist_mm <= 0:
                    continue
                if dist_mm >= MAX_DIST_MM: # (전역 상수)
                    continue

                idx = int((angle_deg % 360.0) // BIN_DEG) # (전역 상수)
                if 0 <= idx < num_bins and not seen[idx]:
                    seen[idx] = True
                    filtered.append((q, angle_deg, dist_mm))
            # --- 필터링 끝 ---

            # --- [수정] 2-2. 극→직교(mm) (NumPy 벡터화) ---
            # (기존의 느린 Python 루프 삭제)
            if not filtered:
                xy_np = np.zeros((0, 2), np.float32)
                xy = []
            else:
                # filtered에서 각도와 거리만 NumPy 배열로 추출
                filtered_np = np.array(filtered, dtype=np.float32)
                angles_deg = filtered_np[:, 1]
                dists_mm = filtered_np[:, 2]
                
                # 벡터화된 연산
                a_rad = np.radians(angles_deg + ANGLE_OFFSET_DEG)
                
                # [수정] 로봇이 반대로 움직이는 문제를 해결하기 위해 180도 회전
                x = dists_mm * np.sin(a_rad)  # [수정] (원본: -dists_mm * np.sin(a_rad))
                y = -dists_mm * np.cos(a_rad) # [수정] (원본:  dists_mm * np.cos(a_rad))
                
                if MIRROR_X:
                    x = -x
                
                # (N, 2) 형태로 배열 결합
                xy_np = np.stack((x, y), axis=1).astype(np.float32)
                
                # xy 리스트는 하위 호환성을 위해 빈 리스트로 유지
                xy = xy_np.tolist()
                # --- 좌표 변환 끝 ---

            # 2-3. DBSCAN
            labels = np.array([], dtype=int)
            if xy_np.shape[0] >= DBSCAN_MIN_SAMPLES: # (전역 상수)
                db = DBSCAN(eps=DBSCAN_EPS_MM, min_samples=DBSCAN_MIN_SAMPLES, metric='euclidean') # (전역 상수)
                labels = db.fit_predict(xy_np)
            
            # 1단계 결과를 튜플로 묶음
            base_data = (filtered, xy, xy_np, labels)

            # 3. 처리 결과를 다음 워커(find_chair)에게 전달
            try:
                if dbs_queue.full():
                    _ = dbs_queue.get_nowait() # 꽉 찼으면 오래된 것 버림
                dbs_queue.put_nowait(base_data)
            except queue.Full:
                pass
            
            latest_raw = None
            # [수정] time.sleep(0.001) 제거 (CPU 낭비 방지)
            
    finally:
        print("[LIDAR_PROC] 워커 종료")
        
# --- 의자 탐색 (군집 분석, 원 피팅 통합) ---
def find_chair(stop_event, dbs_queue, cluster_queue, beautician_tracker):
    # """
    # [통합됨]
    # dbs_queue에서 1단계 데이터를 받아 군집을 분석하고,
    # '원 피팅'을 직접 수행하며,
    # '의자' 기준에 맞는 후보 중 *가장 가까운 1개*만 'is_chair=True'로 설정.
    # 최종 결과를 cluster_queue로 보낸다.
    # """
    base_data = None
    
    # [신규] 1초 디버그 로깅을 위한 타임스탬프
    last_debug_log_ts = 0.0
    
    try:
        while not stop_event.is_set():
            # 1. [수정] 큐에서 데이터 가져오기 (데이터가 올 때까지 효율적으로 대기)
            try:
                base_data = dbs_queue.get(timeout=0.1)
            except queue.Empty:
                continue # 0.1초간 데이터 없음 -> 다음 루프

            # [수정] 큐에 쌓인 나머지 데이터 비우고, 가장 최신 것만 사용
            try:
                while True:
                    base_data = dbs_queue.get_nowait()
            except queue.Empty:
                pass # 큐가 비었으면 방금 가져온 base_data 사용
            
            # 2. [핵심 2단계] 군집 분석 및 의자 판별
            
            # 2-1. 입력 튜플 풀기
            filtered, xy, xy_np, labels = base_data
            
            clusters = [] # 최종 결과물
            unique = sorted([l for l in set(labels.tolist()) if l != -1])

            # [신규] 1초마다 로깅할지 여부 결정
            now_ts_for_debug = time.time()
            do_debug_log = (DEBUG_CHAIR and (now_ts_for_debug - last_debug_log_ts) >= 1.0)

            # 2-2. 모든 군집을 순회하며 분석
            for cid in unique:
                pts = xy_np[labels == cid]
                idx_global = np.where(labels == cid)[0]
                n = int(pts.shape[0])
                if n == 0: continue

                # (1) PCA (생략)
                mu = pts.mean(axis=0)
                d = pts - mu
                try:
                    _, _, Vt = np.linalg.svd(d, full_matrices=False)
                    axis = Vt[0]
                except Exception:
                    axis = np.array([1.0, 0.0], dtype=np.float32)
                projs = d @ axis
                i_min = int(np.argmin(projs)); i_max = int(np.argmax(projs))
                
                # --- [신규] 끝점(Endpoint) 간 거리(Chord Length) 계산 ---
                p_min = pts[i_min]
                p_max = pts[i_max]
                endpoint_dist_mm = float(math.hypot(p_max[0] - p_min[0], p_max[1] - p_min[1]))
                # --- [신규 끝] ---
                
                # (2) 중앙점 (생략)
                mid_proj = 0.5 * (projs[i_min] + projs[i_max])
                i_mid = int(np.argmin(np.abs(projs - mid_proj)))
                mid_point = pts[i_mid]
                mid_idx_global = int(idx_global[i_mid])
                mid_x, mid_y = float(mid_point[0]), float(mid_point[1])
                distc = float(math.hypot(mid_x, mid_y))
                raw_angc = math.degrees(math.atan2(mid_x, mid_y))
                angc = _wrap180(raw_angc - ANGLE_OFFSET_DEG)
                
                # --- (3) [내장] 원 적합 (Taubin) + 호 계산 ---
                xc = yc = R = rmse = None
                arc_span_deg = None
                
                # [수정] is_wall 변수를 여기서 먼저 초기화합니다.
                is_wall = False
                
                if n >= 3:
                    # --- _fit_circle_taubin(pts) 로직 시작 ---
                    pts_np = pts # 변수명 일치 (pts는 이미 (N,2) numpy 배열)
                    N_fit = pts_np.shape[0] # (n과 동일)

                    pts_m = pts_np.astype(np.float64) / 1000.0
                    mean_m = np.mean(pts_m, axis=0)
                    uv_fit = pts_m - mean_m
                    u_fit = uv_fit[:, 0]
                    v_fit = uv_fit[:, 1]
                    
                    uu = u_fit * u_fit
                    vv = v_fit * v_fit
                    uv = u_fit * v_fit
                    
                    Suu = np.sum(uu)
                    Svv = np.sum(vv)
                    Suv = np.sum(uv)
                    Suuu = np.sum(uu * u_fit)
                    Svvv = np.sum(vv * v_fit)
                    Suuv = np.sum(uu * v_fit)
                    Suvv = np.sum(u_fit * vv)
                    
                    A = np.array([[Suu, Suv], [Suv, Svv]], dtype=np.float64)
                    b = 0.5 * np.array([Suuu + Suvv, Svvv + Suuv], dtype=np.float64)

                    try:
                        uc_fit, vc_fit = np.linalg.solve(A, b)
                        
                        xc_m = uc_fit + mean_m[0]
                        yc_m = vc_fit + mean_m[1]
                        R_m = np.sqrt(uc_fit*uc_fit + vc_fit*vc_fit + (Suu + Svv)/N_fit)

                        d_fit = np.sqrt((pts_m[:,0] - xc_m)**2 + (pts_m[:,1] - yc_m)**2)
                        rmse_m = np.sqrt(np.mean((d_fit - R_m)**2))

                        xc = float(xc_m * 1000.0)
                        yc = float(yc_m * 1000.0)
                        R = float(R_m * 1000.0)
                        rmse = float(rmse_m * 1000.0)

                    except np.linalg.LinAlgError:
                        pass
                    # --- _fit_circle_taubin(pts) 로직 끝 ---

                    # 호(Arc) 계산
                    if xc is not None:
                        thetas = np.degrees(np.arctan2(pts[:,1]-yc, pts[:,0]-xc))
                        t = np.sort((thetas + 360.0) % 360.0)
                        gaps = np.diff(np.concatenate([t, t[:1] + 360.0]))
                        max_gap = np.max(gaps) if gaps.size else 360.0
                        arc_span_deg = float(360.0 - max_gap)
                        
                # --- [수정] 벽(직선) 판정 (if n >= 3: 블록 밖으로 이동) ---
                # 1. 원 피팅 실패 (직선)
                if R is None:
                    if endpoint_dist_mm >= WALL_MIN_ENDPOINT_DIST_MM:
                        is_wall = True
                # 2. 반지름이 1.5m 이상 (직선)
                elif R >= 1500.0: # 1000.0 -> 1500.0
                    if endpoint_dist_mm >= WALL_MIN_ENDPOINT_DIST_MM:
                        is_wall = True
                # 3. 피팅 오차 큼 (L-shape 등)
                elif rmse is not None and rmse > WALL_RMSE_THRESHOLD: # (30.0)
                    if endpoint_dist_mm >= WALL_MIN_ENDPOINT_DIST_MM: # (500.0)
                        is_wall = True
                # --- [수정 끝] ---

                # (4) 의자 '후보' 판정
                is_chair = False
                
                # [디버깅 로직 수정]
                # (1초마다 모든 주요 군집의 상세 정보 로깅)
                if do_debug_log:
                    # [오류 수정된 print문]
                    print(f"[CHAIRDBG] id={cid} | n={n} | R={(f'{R:.1f}' if R is not None else 'N/A')} | rmse={(f'{rmse:.1f}' if rmse is not None else 'N/A')} | arc={(f'{arc_span_deg:.1f}' if arc_span_deg is not None else 'N/A')} | end_dist={endpoint_dist_mm:.0f} | distc={distc:.0f} | is_wall={is_wall}")

                if is_wall:
                    if do_debug_log: print(f"[CHAIRDBG] id={cid} REJECTED: is_wall=True")
                elif n < CHAIR_MIN_PTS:
                    if do_debug_log: print(f"[CHAIRDBG] id={cid} REJECTED: n={n} (Min={CHAIR_MIN_PTS})")
                elif R is None or R < CHAIR_R_MIN_MM or R > CHAIR_R_MAX_MM:
                    # [오류#수정된#줄]
                    if do_debug_log: print(f"[CHAIRDBG] id={cid} REJECTED: R={(f'{R:.1f}' if R is not None else 'N/A')} (Range=({CHAIR_R_MIN_MM:.0f}~{CHAIR_R_MAX_MM:.0f}))")
                elif rmse is None or rmse > CHAIR_RMSE_MAX_MM:
                    # [오류#수정된#줄]
                    if do_debug_log: print(f"[CHAIRDBG] id={cid} REJECTED: rmse={(f'{rmse:.1f}' if rmse is not None else 'N/A')} (Max={CHAIR_RMSE_MAX_MM:.1f})")
                elif arc_span_deg is None or arc_span_deg < CHAIR_ARC_SPAN_MIN_DEG or arc_span_deg > CHAIR_ARC_SPAN_MAX_DEG:
                    # [오류#수정된#줄]
                    if do_debug_log: print(f"[CHAIRDBG] id={cid} REJECTED: arc_span={(f'{arc_span_deg:.1f}' if arc_span_deg is not None else 'N/A')} (Range=({CHAIR_ARC_SPAN_MIN_DEG:.0f}~{CHAIR_ARC_SPAN_MAX_DEG:.0f}))")
                elif endpoint_dist_mm < CHAIR_ENDPOINT_DIST_MIN_MM:
                    if do_debug_log: print(f"[CHAIRDBG] id={cid} REJECTED: endpoint_dist={endpoint_dist_mm:.0f} (Min={CHAIR_ENDPOINT_DIST_MIN_MM:.0f})")
                elif not (distc >= 200.0 and distc <= 1100.0):
                    if do_debug_log: print(f"[CHAIRDBG] id={cid} REJECTED: distc={distc:.0f} (Range=(200~1100))")
                else:
                    # 모든#조건을#통과
                    is_chair = True
                    if do_debug_log: print(f"[CHAIRDBG] id={cid} ACCEPTED as Chair Candidate.")

                # --- [신규#추가]#미용사#배제#로직#(150mm) ---
                if is_chair:
                    b_lock_pos = beautician_tracker.lock_center
                    if b_lock_pos is not None:
                        # (중앙점#mid_x,#mid_y#기준으로#미용사와의#거리#계산)
                        dist_to_b = math.hypot(mid_x - b_lock_pos[0], mid_y - b_lock_pos[1])
                        
                        if dist_to_b < CHAIR_BEAUTICIAN_EXCLUSION_MM: # (150.0)
                            is_chair = False # 의자#후보#자격#박탈
                            if do_debug_log: print(f"[CHAIRDBG] id={cid} REJECTED: Too close to beautician (Dist: {dist_to_b:.0f}mm)")
                # --- [신규#추가#끝] ---

                # (5) 군집#결과#append
                clusters.append({
                    "id": int(cid), "n": n,
                    "trk_x": mid_x, "trk_y": mid_y,
                    "mid_idx": mid_idx_global,
                    "center_dist_mm": distc,
                    "center_angle_deg_robot": angc,
                    "fit_xc": xc, "fit_yc": yc,
                    "fit_R": R, "fit_rmse": rmse,
                    "arc_span_deg": arc_span_deg,
                    "endpoint_dist_mm": endpoint_dist_mm,
                    "is_wall": is_wall,
                    "is_chair": is_chair
                })
            
            if do_debug_log:
                last_debug_log_ts = now_ts_for_debug
                print("--- [CHAIRDBG] Log Cycle End ---")

            # --- 'is_chair'가 True인 것 중 가장 가까운 1개만 선택 ---
            chair_candidates = [c for c in clusters if c.get("is_chair")]
            
            if chair_candidates:
                best_chair = min(chair_candidates, key=lambda c: c["center_dist_mm"])
                best_chair_id = best_chair["id"]
                
                for c in clusters:
                    c["is_chair"] = (c["id"] == best_chair_id)

            proc_result = (filtered, xy, xy_np, labels, clusters)

            # 3. 최종 결과를 메인/다음 워커에게 전달
            try:
                if cluster_queue.full():
                    _ = cluster_queue.get_nowait()
                cluster_queue.put_nowait(proc_result)
            except queue.Full:
                pass

            base_data = None

    finally:
        print("[CHAIR_FIND] 워커 종료")
        
        
        
        
def _wrap180(deg):
    """각도를 -180° ~ +180° 범위로 래핑합니다."""
    return (deg + 180.0) % 360.0 - 180.0

# --- 미용사 탐색 (헬퍼 함수 로직 통합) ---
def find_beautician(stop_event, cluster_queue, fusion_queue, track_controller, chair_tracker):
    """
    [수정됨 - 워커 3]
    cluster_queue(라이다 처리 결과)를 받아,
    1. 'track_controller'가 활성 상태인지 확인합니다.
    2. [수정] 
        - 락(lock)이 없으면(스캔 모드): 카메라 각도를 기준으로 새 락을 찾습니다.
        - 락(lock)이 있으면(추적 모드): 카메라를 무시하고 라이다만으로 락을 갱신합니다.
    3. 전역 'beautician_tracker'를 업데이트합니다.
    """
    latest_proc_result = None # (filtered, xy, xy_np, labels, clusters) 튜플

    # [수정 1] 이전 TRACK 모드 상태를 기억 (반복 리셋 방지용)
    prev_track_mode_active = False

    # 미용사 후보 최대 거리 (mm)
    BEAUTICIAN_MAX_DIST_MM = 1500.0
    
    # [신규] 미용사 다리 최대 폭 (mm)
    BEAUTICIAN_MAX_LEG_WIDTH_MM = 200.0

    try:
        while not stop_event.is_set():
            # 1. 큐에서 데이터 가져오기
            try:
                latest_proc_result = cluster_queue.get(timeout=0.1)
            except queue.Empty:
                continue 

            try:
                while True:
                    latest_proc_result = cluster_queue.get_nowait()
            except queue.Empty:
                pass 
            
            # --- [핵심] 미용사 추적 로직 (요청사항 반영) ---

            # 2-1. 데이터 준비
            filtered, xy, xy_np, labels, clusters = latest_proc_result
            now_ts_for_tracking = time.time()

            # (GUI 표시용 변수 초기화)
            cam_data = None
            beautician_angle_from_cam = None

            # 2-2. TRACK 모드 활성화 여부 확인
            is_track_mode_active = (track_controller._th and
                                    track_controller._th.is_alive() and
                                    track_controller.allow_beautician_recognition)
            
            if not is_track_mode_active:
                # TRACK 모드가 아니면, 모든 플래그를 끄고 트래커를 리셋합니다.

                # --- [수정 2] TRACK 모드가 '방금' 비활성화되었을 때만 1회 리셋 ---
                if prev_track_mode_active:
                    print("[TRACK] TRACK mode deactivated. Resetting beautician tracker.")
                    beautician_tracker.reset()
                # --- [수정 끝] ---
                
                for c in clusters:
                    c["is_beautician_candidate"] = False
                # beautician_tracker.reset() # [수정] 이 라인을 위로 옮기고 조건부로 만들었으므로 삭제.
                selected_cluster, locked_center = None, None
            
            else:
                # TRACK 모드 활성
                
                # --- [신규 수정] 현재 락 상태를 먼저 확인 ---
                is_already_tracking = (beautician_tracker.lock_center is not None)
                
                beautician_candidates_in_fan = [] # 카메라 기반 후보 목록

                if not is_already_tracking:
                    # --- [상태 A: 스캔 모드] ---
                    # 락이 없으므로, 카메라로 '새로운' 락을 찾아야 함
                    
                    cam_data = camera_latest.get() # (GUI 표시용)
                    if cam_data is not None and (now_ts_for_tracking - cam_data["ts"]) < 1.0:
                        beautician_angle_from_cam = cam_data["angle_deg"]

                    if beautician_angle_from_cam is not None:
                        # (기존 필터링 로직...)
                        cam_angle_180 = _wrap180(beautician_angle_from_cam)
                        if MIRROR_X:
                            cam_angle_180 = -cam_angle_180
                        
                        if DEBUG_TRACK:
                            print(f"[TRACK_DBG] (SCAN) Camera Angle={cam_angle_180:.1f}°, searching clusters...")
                            
                        # 의자 관련 필터는 계속 유지
                        chair_lock_pos = chair_tracker.lock_center
                        chair_lock_is_warm = (
                            chair_lock_pos is not None and
                            chair_tracker.last_seen_ts is not None and
                            # [수정] 의자 hold_sec가 math.inf가 되었으므로, 10초 쿨다운만 적용
                            (now_ts_for_tracking - chair_tracker.last_seen_ts) <= (10.0) 
                        )
                        
                        for c in clusters:
                            # (1) 의자/벽/거리 필터
                            if c.get("is_chair"): continue
                            if c.get("is_wall", False): continue
                            if c.get("center_dist_mm", float('inf')) > BEAUTICIAN_MAX_DIST_MM: continue
                            
                            # (2) 의자 상호 배제 필터
                            if chair_lock_is_warm:
                                if c.get("id") == chair_tracker.locked_cid: continue # ID 기반
                                dist_to_chair = math.hypot(c['trk_x'] - chair_lock_pos[0], c['trk_y'] - chair_lock_pos[1])
                                if dist_to_chair < CHAIR_BEAUTICIAN_EXCLUSION_MM: continue # 위치 기반
                                
                            # (3) 카메라 각도 필터
                            cluster_angle_180 = c.get("center_angle_deg_robot", 0.0)
                            angle_diff = _wrap180(cluster_angle_180 - cam_angle_180)

                            if abs(angle_diff) <= CAMERA_TOL_DEG:
                                if DEBUG_TRACK: print(f"[TRACK_DBG] (SCAN) Cluster {c['id']} MATCHED (Angle={cluster_angle_180:.1f}°, Dist={c['center_dist_mm']:.0f})")
                                beautician_candidates_in_fan.append(c)
                            else:
                                if DEBUG_TRACK: print(f"[TRACK_DBG] (SCAN) Cluster {c['id']} rejected (Angle mismatch: {cluster_angle_180:.1f}°)")

                    # (카메라 기반 후보 목록 생성)
                    if beautician_candidates_in_fan:
                        best_candidate = min(beautician_candidates_in_fan, key=lambda c: c["center_dist_mm"])
                        for c in clusters:
                            c["is_beautician_candidate"] = (c["id"] == best_candidate["id"])
                    else:
                        for c in clusters:
                            c["is_beautician_candidate"] = False
                    
                    # (트래커 업데이트)
                    selected_cluster, locked_center = beautician_tracker.update(clusters, xy_np, labels, now_ts_for_tracking)
                
                else:
                    # --- [상태 B: 추적 모드] ---
                    # 이미 락이 걸려있음. 카메라는 무시.
                    # 트래커가 LiDAR만으로 추적을 이어가도록 함.
                    
                    # if DEBUG_TRACK:
                    #     print(f"[TRACK_DBG] (TRACKING) LiDAR-only tracking mode. (Camera Ignored)")
                    
                    # [중요] is_beautician_candidate 플래그를 설정하지 않습니다 (모두 False).
                    # 'relock'이나 'new lock'이 발동하지 않고,
                    # 'match_radius'와 'raw_point' 로직만 작동하도록 합니다.
                    for c in clusters:
                        c["is_beautician_candidate"] = False
                    
                    # (트래커 업데이트)
                    selected_cluster, locked_center = beautician_tracker.update(clusters, xy_np, labels, now_ts_for_tracking)

            # --- (상태 A, B 공통) ---

            # [수정 3] 현재 활성 상태를 '이전 상태'로 저장
            prev_track_mode_active = is_track_mode_active
            
            # 2-8. 전역 상태 변수(beautician_state) 및 최종 플래그 업데이트
            if locked_center is not None:
                beautician_state.set(locked_center[0], locked_center[1])
            else:
                beautician_state.clear() # 락을 잃으면(hold 만료) 상태를 비움

            beautician_cluster_id = None
            if selected_cluster is not None:
                beautician_cluster_id = selected_cluster['id']
                for c in clusters:
                    c['is_beautician'] = (c['id'] == beautician_cluster_id)
            else:
                for c in clusters:
                    c['is_beautician'] = False
            
            # --- 로직 수정 끝 ---

            # 3. 다음 워커(chair_tracker)로 모든 정보 전달
            output_payload = (
                latest_proc_result,
                cam_data, # [수정] cam_data 전달 (GUI용)
                beautician_cluster_id,
                beautician_angle_from_cam # [수정] beautician_angle_from_cam 전달 (GUI용)
            )

            try:
                if fusion_queue.full():
                    _ = fusion_queue.get_nowait()
                fusion_queue.put_nowait(output_payload)
            except queue.Full:
                pass
            
            latest_proc_result = None
            # [수정] time.sleep(0.001) 제거 (CPU 낭비 방지)
            
    finally:
        print("[FUSION] 워커 종료")
        
        
        
# --- 의자 추적 워커 ---
def track_chair(stop_event, fusion_queue, tracking_result_queue):
    """
    [워커 4]
    fusion_queue(라이다 + 카메라 융합 결과)를 받아,
    의자 추적을 *수행*하고, 모든 데이터를 tracking_result_queue로 넘긴다.
    """
    latest_fusion_data = None # (proc_result, cam_data, b_cid, b_angle) 튜플

    try:
        while not stop_event.is_set():
            # 1. 큐에서 (라이다+카메라) 융합 결과 가져오기
            try:
                while True:
                    latest_fusion_data = fusion_queue.get_nowait()
            except queue.Empty:
                if latest_fusion_data is None:
                    try:
                        latest_fusion_data = fusion_queue.get(timeout=0.01)
                    except queue.Empty:
                        continue
            
            if latest_fusion_data is None:
                continue

            # 2. 입력 데이터 풀기
            (
                latest_proc_result,  # (filtered, xy, xy_np, labels, clusters)
                cam_data,
                beautician_cid,
                beautician_angle
            ) = latest_fusion_data

            # 2-1. proc_result에서 clusters만 우선 분리
            _, _, xy_np, labels, clusters = latest_proc_result

            # 3. [핵심] 의자 추적 ( _update_chair_tracker 로직 내장)
            now_ts_for_tracking = time.time() 

            selected_cluster = None
            locked_center = None
            chair_cluster_ids = [] 

            # (chair_tracker는 전역 변수)
            if clusters or chair_tracker.lock_center is not None:
                selected_cluster, locked_center = chair_tracker.update(
                                    clusters, xy_np, labels, now_ts_for_tracking
                                )
            # (target_state는 전역 변수)
            if locked_center is not None:
                target_state.set(locked_center[0], locked_center[1])
            else:
                target_state.clear()

            # [중요] 'clusters' 리스트 자체의 'is_chair' 플래그를 덮어씀
            if selected_cluster is not None:
                chair_cluster_ids = [selected_cluster['id']]
                for c in clusters: 
                    c['is_chair'] = (c['id'] == selected_cluster['id'])
            else:
                chair_cluster_ids = []
                for c in clusters:
                    c['is_chair'] = False

            if DEBUG_TRACK:
                pass # (디버그 로그)
            
            # 4. 다음 워커(GUI 패키징)로 *모든* 데이터 전달
            output_payload = (
                latest_proc_result,
                cam_data,
                beautician_cid,
                beautician_angle,
                selected_cluster,
                locked_center,
                chair_cluster_ids,
                now_ts_for_tracking, # 추적이 수행된 시각

                # --- [ 추가] chair_tracker의 상태 정보 ---
                chair_tracker.last_seen_ts, # 마지막 발견 시각
                chair_tracker.lock_center,  # 현재 잠금된 내부 좌표 (locked_center와 같을 수 있음)
                chair_tracker.hold_sec      # 홀드 시간 (설정값)
                # (필요한 다른 상태가 있다면 여기에 추가)
            )

            try:
                if tracking_result_queue.full():
                    _ = tracking_result_queue.get_nowait()
                tracking_result_queue.put_nowait(output_payload)
            except queue.Full:
                pass
            
            latest_fusion_data = None
            time.sleep(0.001) # CPU 점유 방지
            
    finally:
        print("[TRACKING] 워커 종료")

# --- GUI 패키징 워커 ---
def gui_packaging(stop_event, tracking_result_queue, out_queue):
    """
    [ 워커 5]
    tracking_result_queue(모든 융합/추적 완료 데이터)를 받아,
    최종 GUI용 'out_queue'로 페이로드(dict)를 조립하여 보낸다.
    (GUI 주기에 맞춰 전송 속도를 조절한다)
    """
    last_pub_ts = 0.0
    latest_tracking_data = None # (매우 긴 튜플)

    try:
        while not stop_event.is_set():
            # 1. 큐에서 모든 결과 데이터 가져오기
            try:
                while True:
                    latest_tracking_data = tracking_result_queue.get_nowait()
            except queue.Empty:
                if latest_tracking_data is None:
                    try:
                        latest_tracking_data = tracking_result_queue.get(timeout=0.01)
                    except queue.Empty:
                        continue
            
            if latest_tracking_data is None:
                continue

            # 2. GUI 주기에 맞춰 전송 (PROCESS_PERIOD 사용)
            now = time.perf_counter()
            if (now - last_pub_ts) >= PROCESS_PERIOD:
                
                # 3. 입력 데이터 풀기
                (
                    latest_proc_result,
                    cam_data,
                    beautician_cid,
                    beautician_angle,
                    selected_cluster,
                    locked_center, # track_chair에서 계산된 최종 locked_center
                    chair_cids,
                    tracking_timestamp,

                    # --- [ 추가] 전달받은 chair_tracker 상태 ---
                    tracker_last_seen_ts,
                    tracker_internal_lock_center, # 참고용 (locked_center와 같을 수 있음)
                    tracker_hold_sec
                ) = latest_tracking_data

                # 3-1. proc_result 다시 풀기 (동일)
                filtered_pts, xy_pts, xy_np, labels, clusters = latest_proc_result

                # --- 4. GUI 페이로드 조립 (전달받은 값 사용) ---
                selected_mid_idx = (selected_cluster.get("mid_idx") if selected_cluster else None)

                lock_age_s_payload = None
                # chair_tracker.last_seen_ts -> tracker_last_seen_ts
                if tracker_last_seen_ts is not None:
                    lock_age_s_payload = tracking_timestamp - tracker_last_seen_ts

                locked_alive_payload = (
                    # chair_tracker.lock_center -> tracker_internal_lock_center 사용 또는 locked_center 사용
                    # 여기서는 track_chair에서 최종 결정된 locked_center를 사용하는 것이 더 명확해 보입니다.
                    locked_center is not None and
                    # chair_tracker.last_seen_ts -> tracker_last_seen_ts
                    tracker_last_seen_ts is not None and
                    # chair_tracker.hold_sec -> tracker_hold_sec
                    (tracking_timestamp - tracker_last_seen_ts) <= tracker_hold_sec
                )

# --- [NEW] 미용사 추적 상태 (전역 트래커에서 읽기) ---
                b_tracker_center = beautician_tracker.lock_center
                b_tracker_last_seen = beautician_tracker.last_seen_ts
                b_tracker_hold = beautician_tracker.hold_sec
                
                b_lock_age_s_payload = None
                if b_tracker_last_seen is not None:
                    b_lock_age_s_payload = tracking_timestamp - b_tracker_last_seen # (의자 추적 시간 기준으로 비교)

                b_locked_alive_payload = (
                    b_tracker_center is not None and
                    b_tracker_last_seen is not None and
                    b_lock_age_s_payload is not None and
                    b_lock_age_s_payload <= b_tracker_hold
                )

                payload = {
                    "points": filtered_pts,
                    "xy": xy_pts,
                    "labels": labels.tolist() if labels.size else [],
                    "clusters": clusters, # 'is_chair'와 'is_beautician'이 모두 갱신된 버전

                    # 의자 정보
                    "chair_cluster_ids": chair_cids,
                    "selected_chair_id": (selected_cluster["id"] if selected_cluster else None),
                    "selected_mid_idx": selected_mid_idx,
                    "locked_center": (list(locked_center) if locked_center is not None else None),
                    "locked_alive": bool(locked_alive_payload),
                    "lock_age_s": (float(lock_age_s_payload) if lock_age_s_payload is not None else None),

                    # 카메라/미용사 정보
                    "camera": cam_data,
                    "beautician_angle_deg": beautician_angle,
                    "beautician_cluster_id": beautician_cid, # 추적된 ID
                    "beautician_tol_deg": CAMERA_TOL_DEG,

                    # [NEW] 미용사 추적 상태
                    "beautician_locked_center": (list(b_tracker_center) if b_tracker_center else None),
                    "beautician_locked_alive": bool(b_locked_alive_payload),
                }

                # 5. out_queue 전송
                try:
                    if out_queue.full():
                        _ = out_queue.get_nowait()
                    out_queue.put_nowait(payload)
                except queue.Full:
                    pass
                
                last_pub_ts = now
            
            latest_tracking_data = None
            time.sleep(0.001) # (이 sleep은 주기가 아닐 때 CPU 점유 방지용)
            
    finally:
        print("[GUI_PACK] 워커 종료")

# 제어 스레드(Clean/Track)들이 모터로 명령 전송 시 사용
def q_put_latest(q, cmd: str):
    # 큐가 꽉 차 있으면 오래된 것부터 버리고 새 명령을 넣는다
    while q.full():
        try:
            q.get_nowait()
        except queue.Empty:
            break
    q.put_nowait(cmd)

# 카메라 데이터 수신 및 저장
class CameraData:
    """
    카메라 스레드가 인식한 미용사 데이터를 저장하고,
    다른 스레드가 최신값을 안전하게 읽을 수 있도록 하는 스레드 안전 객체.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.data = None  # {"angle_deg": float, "center_x": float, "center_y": float, "w": float, "h": float, "ts": float}

    def set(self, angle_deg, cx, cy, bw, bh):
        """카메라가 인식한 인물의 각도·중심 좌표·바운딩박스 크기를 현재 시각과 함께 저장."""
        with self.lock:
            self.data = {
                "angle_deg": float(angle_deg) % 360.0,
                "center_x": float(cx), "center_y": float(cy),
                "w": float(bw), "h": float(bh),
                "ts": time.time()
            }

    def get(self):
        """Lock을 걸고 가장 최근에 저장된 카메라 데이터 복사본을 안전하게 반환"""
        with self.lock:
            return None if self.data is None else dict(self.data)

# 의자 목표의 좌표와 거리를 저장하고, 스레드 안전하게 조회·해제
class ChairState:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = None  # {"x_mm": float, "y_mm": float, "dist_mm": float, "ts": float}

    def set(self, x_mm, y_mm):
        """로봇 기준 좌표(mm)로 타겟 설정"""
        with self.lock:
            dist_mm = math.hypot(x_mm, y_mm)
            self.data = {
                "x_mm": float(x_mm),
                "y_mm": float(y_mm),
                "dist_mm": dist_mm,
                "ts": time.time()
            }
            
    def clear(self):
        """타겟 해제"""
        with self.lock:
            self.data = None

    def get(self):
        """현재 타겟 정보 반환 (없으면 None)"""
        with self.lock:
            return None if self.data is None else dict(self.data)

# 의자 군집의 위치를 지속적으로 추적하고, 일정 시간 동안 사라져도 자동으로 추적
class ChairTracker:
    """
    - lock_center: (x,y) 현재 잠금된 의자 중심(mm)
    - last_seen_ts: 마지막으로 실제 라이다 클러스터와 매칭된 시각
    - hold_sec: 매칭이 끊겨도 이 시간 동안 가상 타겟으로 유지
    - match_radius: 잠금 중심에서 이 반경 이내의 클러스터만 같은 의자로 인정
    - max_step: 한 프레임에서 중심이 이동할 수 있는 최대 거리(점프 방지)
    - relock_grace_sec: 락이 살아있어도, 반경 내 매칭이 없고 의자 후보가 존재하면
      이 시간 이후에는 가장 좋은 후보로 '즉시 재락' (무한 hold 방지)
    """
    def __init__(self,
                match_radius_mm=300.0,
                hold_sec=3.0,
                max_step_mm=300.0,
                relock_grace_sec=1.0,
                raw_point_radius_mm=250.0,
                raw_point_min_samples=3
                ):
        self.match_radius = float(match_radius_mm)
        self.hold_sec = float(hold_sec) # [수정] math.inf를 float으로 변환
        self.max_step = float(max_step_mm)
        self.relock_grace = float(relock_grace_sec)
        self.raw_point_radius = float(raw_point_radius_mm)
        self.raw_point_min_samples = int(raw_point_min_samples)
        self.lock_center = None
        self.last_seen_ts = None
        self.locked_cid = None

    def _clamp_step(self, prev, new):
        """이전 위치와 새 위치 간 이동 거리를 제한해 한 번에 이동할 최대 보폭(max_step)을 넘지 않도록 보정"""
        if prev is None:
            return new
        px, py = prev; nx, ny = new
        dx = nx - px; dy = ny - py
        d = math.hypot(dx, dy)
        if d <= self.max_step or d == 0.0:
            return (nx, ny)
        k = self.max_step / d
        return (px + dx * k, py + dy * k)

    def update(self, all_clusters, xy_np, labels, now_ts):
        """현재 잠금 유지/갱신·원시포인트 보강·유예시간 후 재락·신규 락 획득까지 포함한 의자 중심 추적 메인 로직 실행 후 (선택된 클러스터, 출력 중심)을 반환."""
        # 현재 락이 살아있는지
        lock_alive = (
            self.lock_center is not None and
            self.last_seen_ts is not None and
            (now_ts - self.last_seen_ts) <= self.hold_sec
        )

        # 의자 판별된 후보와, 아닌 후보를 분리
        chair_candidates = [c for c in all_clusters if c.get("is_chair")]

        # 후보가 하나도 없을 때 (all_clusters와 xy_np가 모두 비어있을 때)
        if not all_clusters and xy_np.shape[0] == 0:
            if lock_alive:
                return None, self.lock_center # 가상 타겟 유지
            # 완전 해제
            self.lock_center = None
            self.last_seen_ts = None
            self.locked_cid = None
            return None, None

        # 후보가 있거나, 락이 살아있을 때
        if lock_alive:
            # 1) [기존] 반경(10cm) 내 '클러스터' 찾기
            lx, ly = self.lock_center
            in_radius = []
            for c in all_clusters:
                # --- [수정] 벽(is_wall=True)인 클러스터는 락 유지/갱신에 사용하지 않음 ---
                if c.get("is_wall", False):
                    continue
                # --- [수정 끝] ---
                
                d = math.hypot(c['trk_x'] - lx, c['trk_y'] - ly)
                if d <= self.match_radius: 
                    in_radius.append((d, c))

            if in_radius:
                # [기존] 반경 내 클러스터로 락 갱신
                in_radius.sort(key=lambda t: t[0])
                best = in_radius[0][1]
                new_center = (best['trk_x'], best['trk_y'])
                out_center = self._clamp_step(self.lock_center, new_center)
                self.lock_center = out_center
                self.last_seen_ts = now_ts
                self.locked_cid = best['id']
                return best, out_center
            
            # --- [수정] 원시 점(Raw Point) 추적 로직 비활성화 ---
            # (이 로직이 벽을 추적하게 만드는 원인임)
            # if xy_np.shape[0] > 0:
            #     # 모든 원시 포인트와 락 중심 간의 거리 계산
            #     dists_sq = np.sum((xy_np - np.array([lx, ly]))**2, axis=1)
            #     nearby_indices = np.where(dists_sq <= self.raw_point_radius**2)[0]
            #
            #     if nearby_indices.size >= self.raw_point_min_samples:
            #         # 5cm 내 원시 포인트가 3개 이상 있으면
            #         nearby_points = xy_np[nearby_indices]
            #         # 그 점들의 평균으로 락을 "끌고" 감
            #         new_center_raw = tuple(np.mean(nearby_points, axis=0))
            #         
            #         if DEBUG_TRACK:
            #             print(f"[TRACK] Raw point lock! ({nearby_indices.size} pts) -> {new_center_raw}")
            #
            #         out_center = self._clamp_step(self.lock_center, new_center_raw)
            #         self.lock_center = out_center
            #         self.last_seen_ts = now_ts # ⬅ [중요] 락 시간 갱신!
            #         self.locked_cid = -2 # (임의의 ID, '원시점 추적' 의미)
            #         
            #         # 선택된 '클러스터'는 없지만 락은 유지/갱신됨
            #         return None, out_center
            # --- [수정 끝] ---
            
            # 2) 반경 내 매칭이 없지만 '의자 후보(chair_candidates)'는 있음
            #    (relock_grace 로직은 '의자 후보'를 기준으로 동작해야 함)
            age = now_ts - self.last_seen_ts if self.last_seen_ts is not None else 1e9
            
            # --- [수정] chair_candidates가 있을 때만 재락(relock) ---
            if age >= self.relock_grace and chair_candidates:
                
                # --- [신규] 재락 후보 중 벽(is_wall)인 것은 제외 ---
                valid_relock_candidates = [
                    c for c in chair_candidates if not c.get("is_wall", False)
                ]
                
                if valid_relock_candidates:
                    # 가장 "로봇에 가까운" *유효한 의자 후보*로 스냅
                    valid_relock_candidates.sort(key=lambda c: c['center_dist_mm'])
                    best = valid_relock_candidates[0]
                    # --- [수정 끝] ---
                    
                    new_center = (best['trk_x'], best['trk_y'])
                    self.lock_center = new_center
                    self.last_seen_ts = now_ts
                    self.locked_cid = best['id']
                    return best, new_center

            # 3) 아직 유예시간 이내면 hold 유지
            return None, self.lock_center
        
        # 락이 없거나 만료되었으면 새로 획득:
        # *의자 후보(chair_candidates)* 중에서 로봇에 가장 가까운 것 1개
        if not chair_candidates:
            # 새로 락을 걸 '의자'가 없으면
            self.lock_center = None
            self.last_seen_ts = None
            self.locked_cid = None
            return None, None
            
        valid_new_lock_candidates = [
            c for c in chair_candidates if not c.get("is_wall", False)
        ]
        
        if not valid_new_lock_candidates:
            # (is_chair=True로 판별된 것이 모두 벽이었던 예외적인 경우)
            self.lock_center = None
            # ... (이하 동일) ...
            return None, None
        
        valid_new_lock_candidates.sort(key=lambda c: c['center_dist_mm'])
        best = valid_new_lock_candidates[0]
        # --- [수정 끝] ---
        
        new_center = (best['trk_x'], best['trk_y'])
        # 초기 획득은 점프 제한 없이
        self.lock_center = new_center
        self.last_seen_ts = now_ts
        self.locked_cid = best['id']
        return best, new_center

# 미용사의 좌표와 거리를 저장하고 스레드 안전하게 조회·해제
class BeauticianState:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = None  # {"x_mm": float, "y_mm": float, "dist_mm": float, "ts": float}

    def set(self, x_mm, y_mm):
        """로봇 기준 좌표(mm)로 미용사 타겟 설정"""
        with self.lock:
            dist_mm = math.hypot(x_mm, y_mm)
            self.data = {
                "x_mm": float(x_mm),
                "y_mm": float(y_mm),
                "dist_mm": dist_mm,
                "ts": time.time()
            }
            
    def clear(self):
        """미용사 타겟 해제"""
        with self.lock:
            self.data = None

    def get(self):
        """현재 미용사 타겟 정보 반환 (없으면 None)"""
        with self.lock:
            return None if self.data is None else dict(self.data)

# 미용사의 위치를 지속적으로 추적하고, 일정 시간 동안 사라져도 자동으로 추적
class BeauticianTracker:
    """
    [NEW] 미용사 추적기 (ChairTracker 복사 및 수정)
    - 미용사는 의자보다 빠르게 움직이므로 match_radius와 max_step을 더 크게 설정
    - 'is_beautician_candidate' 플래그를 사용해 후보군 탐색
    """
    def __init__(self,
                match_radius_mm=500.0,
                hold_sec=3.0,      
                max_step_mm=500.0,
                relock_grace_sec=1.0,
                raw_point_radius_mm=500.0,
                raw_point_min_samples=2
                ):
        self.match_radius = float(match_radius_mm)
        self.hold_sec = float(hold_sec)
        self.max_step = float(max_step_mm)
        self.relock_grace = float(relock_grace_sec)
        self.raw_point_radius = float(raw_point_radius_mm)
        self.raw_point_min_samples = int(raw_point_min_samples)
        self.lock_center = None
        self.last_seen_ts = None
        self.locked_cid = None

    # --- [수정 2] 추적 상태 초기화 함수 ---
    def reset(self):
        """추적 상태를 강제로 초기화합니다."""
        # print("[TRACKER] BeauticianTracker state reset.")
        self.lock_center = None
        self.last_seen_ts = None
        self.locked_cid = None
    # --- [수정 끝] ---

    def _clamp_step(self, prev, new):
        # (이 함수는 ChairTracker와 동일)
        if prev is None:
            return new
        px, py = prev; nx, ny = new
        dx = nx - px; dy = ny - py
        d = math.hypot(dx, dy)
        if d <= self.max_step or d == 0.0:
            return (nx, ny)
        k = self.max_step / d
        return (px + dx * k, py + dy * k)

    def update(self, all_clusters, xy_np, labels, now_ts):
        """[수정됨] 'is_beautician_candidate' 플래그를 기준으로 추적 업데이트"""
        lock_alive = (
            self.lock_center is not None and
            self.last_seen_ts is not None and
            (now_ts - self.last_seen_ts) <= self.hold_sec
        )

        # [수정] 'is_chair' -> 'is_beautician_candidate'
        beautician_candidates = [c for c in all_clusters if c.get("is_beautician_candidate")]

        if not all_clusters and xy_np.shape[0] == 0:
            if lock_alive:
                return None, self.lock_center
            self.lock_center = None
            self.last_seen_ts = None
            self.locked_cid = None
            return None, None

        if lock_alive:
            lx, ly = self.lock_center
            in_radius = []
            for c in all_clusters:
                # [신규] 의자 군집(is_chair=True)으로는 락을 점프하지 않음
                if c.get("is_chair", False):
                    continue
                
                # --- [수정] 벽(is_wall=True)인 클러스터도 락 점프에 사용하지 않음 ---
                if c.get("is_wall", False):
                    continue
                # --- [수정 끝] ---
                        
                d = math.hypot(c['trk_x'] - lx, c['trk_y'] - ly)
                if d <= self.match_radius:
                    in_radius.append((d, c))

            if in_radius:
                in_radius.sort(key=lambda t: t[0])
                best = in_radius[0][1]
                new_center = (best['trk_x'], best['trk_y'])
                out_center = self._clamp_step(self.lock_center, new_center)
                self.lock_center = out_center
                self.last_seen_ts = now_ts
                self.locked_cid = best['id']
                return best, out_center
            
            # --- [수정] 원시 점(Raw Point) 추적 로직 (복구 및 50cm 적용) ---
            # (클러스터 매칭(in_radius)이 실패했을 때만 실행됨)
            if xy_np.shape[0] > 0:
                # 0.1초 전 락 위치(lx, ly)에서 50cm(self.raw_point_radius) 반경 내 모든 점 탐색
                dists_sq = np.sum((xy_np - np.array([lx, ly]))**2, axis=1)
                nearby_indices = np.where(dists_sq <= self.raw_point_radius**2)[0]
            
                # 50cm 내 2개(min_samples) 이상의 점이 있으면
                if nearby_indices.size >= self.raw_point_min_samples:
                    nearby_points = xy_np[nearby_indices]
                    new_center_raw = tuple(np.mean(nearby_points, axis=0))
                    
                    if DEBUG_TRACK:
                        print(f"[TRACK_DBG] Beautician Raw Point Lock! ({nearby_indices.size} pts) -> {new_center_raw}")

                    out_center = self._clamp_step(self.lock_center, new_center_raw)
                    self.lock_center = out_center
                    self.last_seen_ts = now_ts # ⬅ [중요] 락 시간 갱신!
                    self.locked_cid = -2 # (임의의 ID, '원시점 추적' 의미)
                    
                    # 선택된 '클러스터'는 없지만 락은 유지/갱신됨
                    return None, out_center
            # --- [수정 끝] ---
            
            age = now_ts - self.last_seen_ts if self.last_seen_ts is not None else 1e9
            
            # [수정] chair_candidates -> beautician_candidates
            if age >= self.relock_grace and beautician_candidates:
                beautician_candidates.sort(key=lambda c: c['center_dist_mm'])
                best = beautician_candidates[0]
                new_center = (best['trk_x'], best['trk_y'])
                self.lock_center = new_center
                self.last_seen_ts = now_ts
                self.locked_cid = best['id']
                return best, new_center

            return None, self.lock_center
        
        # [수정] chair_candidates -> beautician_candidates
        if not beautician_candidates:
            self.lock_center = None
            self.last_seen_ts = None
            self.locked_cid = None
            return None, None
            
        beautician_candidates.sort(key=lambda c: c['center_dist_mm'])
        best = beautician_candidates[0]
        new_center = (best['trk_x'], best['trk_y'])
        self.lock_center = new_center
        self.last_seen_ts = now_ts
        self.locked_cid = best['id']
        return best, new_center
# 모터 정지 이벤트를 감지하며 별도 스레드에서 CLEAN 시퀀스를 실행하고
# 정렬·목표거리 접근·하드코딩 회전/전진(현 이동 포함)을 UART 명령으로 단계별 관리하는 컨트롤러.




class CleanController:
    """
    CLEAN 시퀀스
    - motor_stop_event를 수신하여 모든 동작(sleep, drive)을 즉시 중단
    - start_sequence()를 통해 별도 스레드에서 시퀀스 실행 (GUI 차단 방지)
    - [PID] 75도 회전에 PID 제어 적용
    """
    
    def __init__(self, tx_queue, motor_stop_event, # motor_stop_event 추가
                    full_turn_s=5.98,
                    resend_period_s=0.20,
                    align_tol_deg=4.5,
                    approach_mm=700.0,
                    wait_after_align_s=1.0,

                    # --- [수정] 원운동 파라미터 (150 근처, 130~170) ---
                    orbit_target_deg=-90.0,      # 의자 정렬 후 회전할 목표 각도 (의자가 오른쪽)
                    orbit_target_dist_mm=700.0,  # 원운동 시 유지할 거리
                    orbit_duration_s=30.0,       # 원운동 지속 시간
                    orbit_loop_dt=0.1,           # 원운동 제어 루프 주기 (0.1초)
                    orbit_base_pwm_l=150,        # [수정] 원운동 기본 좌측 PWM (제어 기준)
                    orbit_base_pwm_r=130,        # [수정] 원운동 기본 우측 PWM (CW 회전을 위해 L보다 낮게)
                    orbit_kp=0.15,                # [수정] 거리 오차 비례 게인 (PWM / mm)
                    orbit_max_pwm=300,           # [수정] L/R 최대 PWM
                    orbit_min_pwm=100,           # [수정] L/R 최소 PWM
                    orbit_accel_pwm_per_s=300.0, # 원운동 가감속률 (PWM/sec)
                    # --- [수정 완료] ---

                    wait_after_forward_s=1.0,
                    rotate_after_forward_deg=-75.0,
                    # --- 현(Chord) 이동 설정 ---
                    chord_use_hardcoded=True,
                    chord_hardcoded_mm=362.0,    # 36.2 cm
                    chord_r_mm=700.0,            # r*=700 mm
                    chord_angle_div=12,          # 360/12 = 30°
                    # --- 반복 스텝(15° 보정 회전) 설정 ---
                    step_base_deg=15.0,          # 기본 15°
                    step_gain_deg_per_mm=0.01,   # 보정 강도(°/mm)
                    step_max_adjust_deg=5.0,     # 보정 한계 ±5°
                    step_repeat=12,              # 12회 반복
                    ):
            self.q_tx = tx_queue
            self.motor_stop_event = motor_stop_event
            self._th = None
            self._is_cleaner_on = False # [신규] 청소기 전원 상태
            self._lock = threading.Lock() # [신규] 상태 보호용 Lock

            # (기존 파라미터들...)
            self.full_turn_s = float(full_turn_s)
            self.resend_period_s = float(resend_period_s)
            self.align_tol_deg = float(align_tol_deg)
            self.approach_mm = float(approach_mm)
            self.forward_speed_mmps = (0.435 / 1.7) * 1000.0
            self.wait_after_align_s = float(wait_after_align_s)
            self.wait_after_forward_s = float(wait_after_forward_s)
            self.rotate_after_forward_deg = float(rotate_after_forward_deg)
            self.chord_use_hardcoded = bool(chord_use_hardcoded)
            self.chord_hardcoded_mm = float(chord_hardcoded_mm)
            self.chord_r_mm = float(chord_r_mm)
            self.chord_angle_div = int(chord_angle_div)
            self.step_base_deg = float(step_base_deg)
            self.step_gain_deg_per_mm = float(step_gain_deg_per_mm)
            self.step_max_adjust_deg = float(step_max_adjust_deg)
            self.step_repeat = int(step_repeat)

            # --- [NEW] 새 파라미터 저장 ---
            self.orbit_target_deg = float(orbit_target_deg)
            self.orbit_target_dist_mm = float(orbit_target_dist_mm)
            self.orbit_duration_s = float(orbit_duration_s)
            self.orbit_loop_dt = float(orbit_loop_dt)
            self.orbit_base_pwm_l = int(orbit_base_pwm_l)
            self.orbit_base_pwm_r = int(orbit_base_pwm_r)
            self.orbit_kp = float(orbit_kp)
            self.orbit_max_pwm = int(orbit_max_pwm)
            self.orbit_min_pwm = int(orbit_min_pwm)
            self.orbit_accel_pwm_per_s = float(orbit_accel_pwm_per_s)

            # [추가: 선형 주행 PWM 한계]
            self.move_min_pwm = 150     # 스틱션 극복 최소 PWM
            self.move_max_pwm = 300     # 최대 PWM

            # [추가: 선형 주행 슬루 상태]
            self._lin_pwm_prev = 0.0

            # --- [NEW] 원운동 슬루 상태 ---
            self._orbit_l_pwm_prev = 0.0
            self._orbit_r_pwm_prev = 0.0
            # --- [NEW] ---

            # [추가: 모터 전환/연속 주행 관리 상태]
            self._last_cmd = None
            self._last_switch_ts = 0.0

            # [추가: r, θ 필터 상태]
            self._filt_ang = None
            self._filt_r = None
            self._filt_ts = 0.0

            # 속도 슬루 제한(가감속)
            self._lin_pwm_prev = 0.0

            # [추가: 접근 제어 상태 변수]
            self._last_r = None
            self._last_r_ts = 0.0
            self._r_med = []          # 최근 r 샘플(미디안용)
            self._stop_latched = False
            self._stop_latch_until = 0.0
            self._last_dir = 0        # -1:후진, 0:정지, +1:전진
            self._last_dir_change = 0.0

    def is_cleaner_on(self):
        """[신규] 청소기가 켜져 있는지 스레드 안전하게 확인합니다."""
        with self._lock:
            return self._is_cleaner_on

    def start_sequence(self):
        """[수정] CLEAN 시퀀스를 별도 데몬 스레드로 시작합니다."""
        if self._th and self._th.is_alive():
            print("[CLEAN] 이미 시퀀스가 실행 중입니다.") # [수정] CLEAN
            return

        print("[CLEAN] 새 시퀀스 스레드 시작...") # [수정] CLEAN
        
        with self._lock: # [신규]
            self._is_cleaner_on = False # [신규]
        
        self.motor_stop_event.clear()
        
        # [중요] CleanController는 allow_beautician_recognition 플래그가 없습니다.
        # self.allow_beautician_recognition = False 
        
        self._th = threading.Thread(target=self.clean_mode, daemon=True, name="clean_seq") # [수정] track_mode -> clean_mode
        self._th.start()

    def _run_orbit_step(self, target_dist_mm, loop_dt):
            """
            [NEW HELPER for 2-sec realign]
            지정된 목표 거리(target_dist_mm)를
            유지하며 P제어 원운동을 1 스텝(loop_dt) 수행합니다.
            - Slew 상태(_orbit_l/r_pwm_prev)를 연속적으로 갱신합니다.
            - True 반환 시 성공, False 반환 시 중단/오류.
            """
            # 5-1. 중단 신호 확인 (clean_mode의 메인 루프에서 처리)
            # 5-2. 현재 거리 측정
            r_current = self._snapshot_range_mm()
            if r_current is None:
                print(f"[CLEAN] Orbit Step (->{target_dist_mm}mm) target lost. Stopping.")
                return False # (finally에서 STOP 처리)

            # 5-3. 거리 오차 계산
            err_dist = r_current - target_dist_mm
            
            # 5-4. P-제어 (L/R Wheel Target)
            adjustment = self.orbit_kp * err_dist
            
            l_pwm_target = self.orbit_base_pwm_l + adjustment
            r_pwm_target = self.orbit_base_pwm_r - adjustment
            
            # 5-5. Slew (가감속 제한) 적용
            l_pwm_slew = self._slew(
                l_pwm_target, 
                self._orbit_l_pwm_prev, 
                self.orbit_accel_pwm_per_s, 
                loop_dt # <-- duration_s 대신 loop_dt 사용
            )
            
            r_pwm_slew = self._slew(
                r_pwm_target,
                self._orbit_r_pwm_prev,
                self.orbit_accel_pwm_per_s,
                loop_dt # <-- duration_s 대신 loop_dt 사용
            )

            # 5-6. PWM 값 클리핑 (최소/최대)
            l_pwm_final = int(np.clip(
                l_pwm_slew, 
                self.orbit_min_pwm, 
                self.orbit_max_pwm
            ))
            
            r_pwm_final = int(np.clip(
                r_pwm_slew,
                self.orbit_min_pwm, 
                self.orbit_max_pwm
            ))

            # 5-7. 이전 값 갱신 (중요: Slew 상태가 갱신됨)
            self._orbit_l_pwm_prev = l_pwm_slew
            self._orbit_r_pwm_prev = r_pwm_slew

            # 5-8. UART 명령 생성 (항상 전진)
            cmd = f"1,{l_pwm_final},1,{r_pwm_final},0"
            
            q_put_latest(self.q_tx, cmd)
            
            if DEBUG_TRACK:
                print(f"[CLEAN] Orbit(->{target_dist_mm}): r={r_current:.0f}(Err:{err_dist:.0f}) -> L_Tgt:{l_pwm_target:.0f} -> L_Cmd:{l_pwm_final} | R_Cmd:{r_pwm_final}")

            # 5-9. 제어 루프 대기 (clean_mode의 메인 루프에서 처리)
            
            return True # 스텝 성공

    def _run_orbit_segment(self, target_dist_mm, duration_s):
        """
        [NEW HELPER]
        지정된 시간(duration_s) 동안 목표 거리(target_dist_mm)를
        유지하며 P제어 원운동을 1세그먼트 수행합니다.
        - Slew 상태(_orbit_l/r_pwm_prev)를 연속적으로 갱신합니다.
        - True 반환 시 성공, False 반환 시 중단/오류.
        """
        t_start_segment = time.time()
        
        # (Slew 이전 값은 이 함수 밖(clean_mode)에서 관리/초기화되어야 함)

        print(f"[CLEAN] Segment Start: Target {target_dist_mm:.0f}mm for {duration_s:.0f}s...")
        
        while (time.time() - t_start_segment) < duration_s:
            # 5-1. 중단 신호 확인
            if self.motor_stop_event.is_set():
                print(f"[CLEAN] Orbit Segment (->{target_dist_mm}mm) interrupted by STOP.")
                return False # (finally에서 STOP 처리)

            # 5-2. 현재 거리 측정
            r_current = self._snapshot_range_mm()
            if r_current is None:
                print(f"[CLEAN] Orbit Segment (->{target_dist_mm}mm) target lost. Stopping.")
                return False # (finally에서 STOP 처리)

            # 5-3. 거리 오차 계산
            err_dist = r_current - target_dist_mm
            
            # 5-4. P-제어 (L/R Wheel Target) [수정됨]
            adjustment = self.orbit_kp * err_dist
            
            # 멀어지면(adj > 0): L 증가, R 감소
            # 가까워지면(adj < 0): L 감소, R 증가
            l_pwm_target = self.orbit_base_pwm_l + adjustment
            r_pwm_target = self.orbit_base_pwm_r - adjustment # [수정] R도 오차에 따라 함께 조절
            
            # 5-5. Slew (가감속 제한) 적용
            l_pwm_slew = self._slew(
                l_pwm_target, 
                self._orbit_l_pwm_prev, 
                self.orbit_accel_pwm_per_s, 
                self.orbit_loop_dt
            )
            
            r_pwm_slew = self._slew(
                r_pwm_target,
                self._orbit_r_pwm_prev,
                self.orbit_accel_pwm_per_s,
                self.orbit_loop_dt
            )

            # 5-6. PWM 값 클리핑 (최소/최대)
            l_pwm_final = int(np.clip(
                l_pwm_slew, 
                self.orbit_min_pwm, # 130
                self.orbit_max_pwm  # 170
            ))
            
            r_pwm_final = int(np.clip(
                r_pwm_slew,
                self.orbit_min_pwm, # 130
                self.orbit_max_pwm  # 170
            ))

            # 5-7. 이전 값 갱신 (중요: Slew 상태가 갱신됨)
            self._orbit_l_pwm_prev = l_pwm_slew
            self._orbit_r_pwm_prev = r_pwm_slew

            # 5-8. UART 명령 생성 (항상 전진)
            cmd = f"1,{l_pwm_final},1,{r_pwm_final},0"
            
            q_put_latest(self.q_tx, cmd)
            
            if DEBUG_TRACK:
                print(f"[CLEAN] Orbit(->{target_dist_mm}): r={r_current:.0f}(Err:{err_dist:.0f}) -> L_Tgt:{l_pwm_target:.0f} -> L_Cmd:{l_pwm_final} | R_Cmd:{r_pwm_final}")

            # 5-9. 제어 루프 대기
            if not self._interruptible_sleep(self.orbit_loop_dt):
                print(f"[CLEAN] Orbit Segment (->{target_dist_mm}mm) sleep interrupted.")
                return False
        
        # (While 루프 종료)
        print(f"[CLEAN] Orbit Segment (->{target_dist_mm}mm) complete.")
        return True # 세그먼트 성공

    def _ramp_down_orbit(self, ramp_duration_s, loop_dt=0.05):
        """
        [NEW HELPER]
        현재 궤도 속도(_orbit_l/r_pwm_prev)에서 0까지
        ramp_duration_s 동안 부드럽게 감속합니다.
        (Slew 가속도 설정값[orbit_accel_pwm_per_s]을 사용합니다)
        """
        print(f"[CLEAN] Ramping down orbit speed over {ramp_duration_s}s...")
        t_start = time.time()
        
        # 현재 PWM 값이 0이 될 때까지 또는 시간이 다 될 때까지
        while (time.time() - t_start) < ramp_duration_s:
            if self.motor_stop_event.is_set():
                print("[CLEAN] Ramp down interrupted.")
                return False # (finally에서 STOP 처리됨)

            # 1. 목표 속도 0.0으로 Slew 적용
            l_pwm_slew = self._slew(
                0.0, 
                self._orbit_l_pwm_prev, 
                self.orbit_accel_pwm_per_s, 
                loop_dt
            )
            r_pwm_slew = self._slew(
                0.0, 
                self._orbit_r_pwm_prev, 
                self.orbit_accel_pwm_per_s, 
                loop_dt
            )

            # 2. Slew 내부 상태 갱신
            self._orbit_l_pwm_prev = l_pwm_slew
            self._orbit_r_pwm_prev = r_pwm_slew

            # 3. PWM 값 클리핑 (0 ~ Max)
            # (감속 중이므로 orbit_min_pwm 대신 0을 사용)
            l_pwm_final = int(np.clip(l_pwm_slew, 0, self.orbit_max_pwm))
            r_pwm_final = int(np.clip(r_pwm_slew, 0, self.orbit_max_pwm))

            # 4. UART 명령 생성 (항상 전진)
            cmd = f"1,{l_pwm_final},1,{r_pwm_final},0"
            q_put_latest(self.q_tx, cmd)

            # 5. 둘 다 0에 도달했으면 일찍 종료
            if l_pwm_final == 0 and r_pwm_final == 0:
                print("[CLEAN] Ramp down complete (early exit).")
                break # while 루프 탈출
            
            # 6. 제어 루프 대기
            if not self._interruptible_sleep(loop_dt):
                return False # 중단됨
        
        # 7. 루프 종료 후 (시간이 다 됐거나, 0에 도달했거나)
        # 최종 정지 명령 및 Slew 상태 강제 리셋
        q_put_latest(self.q_tx, CMD_STOP) # [수정] self.STOP -> CMD_STOP
        self._orbit_l_pwm_prev = 0.0
        self._orbit_r_pwm_prev = 0.0
        print("[CLEAN] Ramp down finalized (STOP sent, Slew reset).")
        return True

    def clean_mode(self):
        """ [수정됨] 의자정렬 > 100mm 접근 > -90도 회전 > 다단계 원운동 (2초마다 -90도 재정렬) """
        try:
            # === 1. 의자 정렬 (목표 각도 0도) ===
            print("[CLEAN] 1. Starting Alignment (Target: 0 deg)...")
            ok = self._rotate_to_target_deg(
                target_err_deg=0.0, # 정면(0도) 맞춤
                stop_tol_deg=self.align_tol_deg,
                loop_dt=0.15,
                timeout_s=15.0
            )
            if not ok:
                print("[CLEAN] 1. Alignment failed or stopped.")
                return # (finally에서 STOP 처리)

            # === 2. 의자 접근 (100mm) [수정됨] ===
            target_approach_dist = 100.0
            print(f"[CLEAN] 2. Range approach to {target_approach_dist:.0f}mm (Open-Loop)...")
            ok = self._close_to_chair(
                r_target=target_approach_dist, # [수정] 100.0
                target_tolerance_mm=50.0, 
                timeout_s=20.0
            )
            if not ok:
                print("[CLEAN] 2. Approach failed or stopped.")
                return
                    
            # === 3. -90도 회전 (의자를 오른쪽에 두기) ===
            print(f"[CLEAN] 3. Rotating CCW to target (Target: {self.orbit_target_deg:.0f} deg)...")
            ok = self._rotate_to_target_deg(
                target_err_deg=self.orbit_target_deg, # -90.0
                stop_tol_deg=self.align_tol_deg,
                loop_dt=0.15,
                timeout_s=15.0
            )
            if not ok:
                print("[CLEAN] 3. Rotation failed or stopped.")
                return
            
            # === 4. 청소 모드 ON ===
            print("[CLEAN] 4. Wait 1s, then CLEANER ON")
            if not self._interruptible_sleep(1.0):
                print("[CLEAN] 4. Interrupted during pre-clean wait.")
                return
            
            with self._lock: # [신규]
                self._is_cleaner_on = True # [신규]
            
            # CleanController에 정의된 self.CLEANER ("1,0,1,0,1") 사용
            q_put_latest(self.q_tx, CMD_CLEANER) # 청소 모드 ON (토글)
            
            if not self._interruptible_sleep(1.0):
                print("[CLEAN] 4. Interrupted after clean ON.")
                return
            
            # === 5. 다단계 원운동 (2초마다 -90도 재정렬) [로직 변경됨] ===
            print(f"[CLEAN] 5. Starting Multi-Stage Orbit with 2-sec Re-Alignment...")
            
            # Slew 상태 초기화
            self._orbit_l_pwm_prev = 0.0
            self._orbit_r_pwm_prev = 0.0 

            # 각 세그먼트 정의 (목표 거리, 지속 시간)
            segments = [
                (350.0, 35.0),
                (650.0, 55.0),
                (950.0, 65.0)
            ]

            realign_interval_s = 2.0 # 2초마다 재정렬
            last_realign_ts = time.time() # 마지막 재정렬 시각
            loop_dt = self.orbit_loop_dt # 0.1초 (P제어 루프 주기)

            # 전체 세그먼트 순회
            for target_dist_mm, duration_s in segments:
                
                print(f"[CLEAN] Orbit Segment Start: Target {target_dist_mm:.0f}mm for {duration_s:.0f}s...")
                t_segment_start = time.time()
                
                # 현재 세그먼트의 지속 시간(duration_s)만큼 루프 실행
                while (time.time() - t_segment_start) < duration_s:
                    
                    # 1. 중단 신호 확인
                    if self.motor_stop_event.is_set():
                        print(f"[CLEAN] Orbit Segment (->{target_dist_mm}mm) interrupted by STOP.")
                        return # (finally에서 STOP 처리)

                    now = time.time()
                    
                    # 2. 2초마다 재정렬 확인
                    if (now - last_realign_ts) >= realign_interval_s:
                        print(f"[CLEAN] Re-Aligning to {self.orbit_target_deg:.0f} deg (2-sec interval)")
                        
                        # [수정] 재정렬 전 1초간 부드럽게 감속
                        # (기존: q_put_latest(self.q_tx, self.STOP) 및 0.2초 대기)
                        if not self._ramp_down_orbit(ramp_duration_s=1.0):
                            print("[CLEAN] Ramp down failed or stopped.")
                            return # (STOP은 ramp_down에서 이미 처리됨)
                        
                        # [수정] Slew 리셋 및 정지 대기는 _ramp_down_orbit이 수행하므로
                        # 기존 0.2초 대기(_interruptible_sleep(0.2))는 제거함.
                        
                        # -90도 재정렬 수행
                        ok = self._rotate_to_target_deg(
                            target_err_deg=self.orbit_target_deg,
                            stop_tol_deg=self.align_tol_deg,
                            loop_dt=0.15,
                            timeout_s=15.0
                        )
                        if not ok:
                            print("[CLEAN] 2-sec Re-Alignment failed or stopped.")
                            return
                        
                        # [중요] 재정렬 후 Slew 리셋 및 타이머 갱신
                        self._orbit_l_pwm_prev = 0.0
                        self._orbit_r_pwm_prev = 0.0
                        last_realign_ts = time.time() # 재정렬 시각 갱신
                        
                        # 재정렬 후 짧은 대기
                        if not self._interruptible_sleep(0.3): return

                    # 3. P-제어 원운동 1 스텝 수행
                    # (새 헬퍼 함수 _run_orbit_step 사용)
                    if not self._run_orbit_step(target_dist_mm, loop_dt):
                        # 타겟 상실
                        return 
                    
                    # 4. P-제어 루프(0.1초) 대기
                    if not self._interruptible_sleep(loop_dt):
                        print(f"[CLEAN] Orbit Segment (->{target_dist_mm}mm) sleep interrupted.")
                        return
                
                print(f"[CLEAN] Orbit Segment (->{target_dist_mm}mm) complete.")
                # (세그먼트 종료)

            print("[CLEAN] 5. Multi-Stage Orbit complete.")
            
            # === [추가] 6. 청소 모드 OFF ===
            print("[CLEAN] 6. Wait 1s, then CLEANER OFF")
            if not self._interruptible_sleep(1.0):
                print("[CLEAN] 6. Interrupted during post-clean wait.")
                return

            with self._lock: # [신규]
                self._is_cleaner_on = False # [신규]

            # self.CLEANER ("1,0,1,0,1")를 다시 호출하여 토글 OFF
            q_put_latest(self.q_tx, CMD_CLEANER) # 청소 모드 OFF (토글)

            if not self._interruptible_sleep(1.0):
                print("[CLEAN] 6. Interrupted after clean OFF.")
                return
            
            # ################# [신규 추가 시작] #################
            # 7. (1초 대기는 위에서 이미 완료됨)
            
            # 8. 의자 정렬 (목표 각도 0도)
            print("[CLEAN] 8. Starting Final Alignment (Target: 0 deg)...")
            ok = self._rotate_to_target_deg(
                target_err_deg=0.0,
                stop_tol_deg=self.align_tol_deg, # 4.5도
                loop_dt=0.15,
                timeout_s=15.0
            )
            if not ok:
                print("[CLEAN] 8. Final Alignment failed or stopped.")
                return # (finally에서 STOP 처리)
            
            if not self._interruptible_sleep(0.5): return

            # 9. 의자 접근 (800mm)
            target_dist_mm = 800.0
            print(f"[CLEAN] 9. Final Range approach to {target_dist_mm:.0f}mm...")
            # CleanController에 있는 헬퍼 함수 재사용
            ok = self._drive_until_target_range_fwd(
                r_target=target_dist_mm,
                tol_mm=50.0, # +- 5cm
                loop_dt=0.15,
                timeout_s=20.0
            )
            if not ok:
                print("[CLEAN] 9. Final approach failed or stopped.")
                return
            # ################# [신규 추가 끝] #################
            
            print("[CLEAN] 9. Sequence finished.") # [수정] 6 -> 9
            
        except Exception as e:
            print(f"[CLEAN] 시퀀스 실행 중 예외 발생: {e}")
        finally:
            with self._lock: # [신규]
                self._is_cleaner_on = False # [신규]
            q_put_latest(self.q_tx, CMD_STOP) 
            self._orbit_l_pwm_prev = 0.0 # 상태 리셋
            self._orbit_r_pwm_prev = 0.0
            print("[CLEAN] 스레드 종료 (최종 STOP 전송)")

    def _drive_until_target_range_fwd(self, r_target=700.0, tol_mm=10.0, loop_dt=0.15, timeout_s=10.0):
        """
        [수정됨] 목표 거리(r_target)를 중심으로, 오차(tol_mm)
        범위에 들어갈 때까지 FWD/REV 양방향으로 제어합니다.
        (첫 1회는 무조건 움직인 후 거리 비교 시작)
        """
        start_time = time.time()
        
        print(f"[CLEAN] Range Adjust (FWD/REV): Target r={r_target:.0f}mm (Tol: ±{tol_mm:.0f}mm)")

        # [수정] 첫 번째 루프에서는 거리 비교를 건너뛰기 위한 플래그
        is_first_loop = True

        while (time.time() - start_time) < timeout_s:
            if self.motor_stop_event.is_set():
                self._stop_and_clear(); return False

            r_current = self._snapshot_range_mm()
            if r_current is None:
                print("[CLEAN] Range Adjust: Target lost.")
                return False

            remaining_mm = r_current - r_target 
            print(f"[CLEAN] Range Step: Current={r_current:.1f}mm, Target={r_target:.0f}mm (Err: {remaining_mm:.1f} mm)")

            # 2. 정지 조건 (목표 오차 범위 내)
            # [수정] 첫 번째 루프(is_first_loop=True)가 아닐 때만 정지 조건을 검사
            if not is_first_loop and abs(remaining_mm) <= tol_mm:
                print(f"[CLEAN] Range Adjust Success: Reached r={r_current:.1f}mm (Err: {remaining_mm:.1f}).")
                self._stop_and_clear()
                return True

            # 3. 오차에 따라 전/후진 방향 결정
            
            # [수정] 첫 루프에서 Err 0.0일 때 FWD로 강제 (REV로 가는 것 방지)
            if is_first_loop and abs(remaining_mm) <= tol_mm:
                cmd = CMD_FWD # [수정] 733mm에서 FWD로 출발하도록 강제
            elif remaining_mm > 0:
                cmd = CMD_FWD # [수정] 너무 멈. 전진.
            else:
                cmd = CMD_REV  # [수정] 너무 가까움. 후진.
                        
            # 4. 명령 전송
            q_put_latest(self.q_tx, cmd)
            
            # 5. 다음 루프 전 대기 (이 대기 시간(0.15초)이 "0.1초 측정 안 함"을 보장)
            if not self._interruptible_sleep(loop_dt): return False
            
            # 6. [수정] 플래그 해제
            is_first_loop = False

        # 타임아웃 발생 시
        print(f"[CLEAN] Range Adjust Failed: Timeout.")
        return False

    def _snapshot_err_deg(self):
        """최신 타깃으로부터 현재 조향 오차각(도)을 스냅샷."""
        tgt = target_state.get()
        if not tgt:
            return None
        return self._target_angle_from_front_deg(tgt["x_mm"], tgt["y_mm"])

    def _snapshot_range_mm(self):
        """최신 타깃으로부터 현재 거리(mm)을 스냅샷."""
        tgt = target_state.get()
        if not tgt:
            return None
        return math.hypot(tgt["x_mm"], tgt["y_mm"])

    def _interruptible_sleep(self, duration_s):
        """ duration_s 동안 대기. motor_stop_event가 set되면 즉시 False 반환 """
        if duration_s <= 0:
            return True # 대기할 시간 없음

        # time.sleep() 대신 event.wait()를 사용하여 CPU 효율 증대
        # event.wait(timeout)는
        # - timeout이 만료되면 False 반환 (정상 종료)
        # - event가 set되면 True 반환 (중단됨)
        
        was_interrupted = self.motor_stop_event.wait(timeout=duration_s)
        
        if was_interrupted:
            return False # 중단됨
        else:
            return True # 대기 완료 (시간 만료)

    def _close_to_chair(self, r_target=700.0, target_tolerance_mm=100.0, timeout_s=15.0):
        """현재 r 측정→필요 시간 계산→전/후진을 시간 기반 개루프로 실행."""
        print(f"[CLEAN] Open-Loop drive → {r_target:.0f}mm.")
        t0 = time.time()
        
        # 1. 현재 거리 측정 (초기 오차 계산)
        r_current = self._snapshot_range_mm()
        if r_current is None:
            print("[CLEAN] Open-Loop drive: target lost at start → STOP")
            self._stop_and_clear(); return False

        r_error = r_current - r_target
        
        # 2. 이미 목표 범위 내에 있는지 확인
        if abs(r_error) <= target_tolerance_mm:
            print(f"[CLEAN] Open-Loop drive: already within tolerance ({r_current:.0f}mm)")
            self._stop_and_clear(); return True
        
        # 3. 전진 또는 후진 결정
        if r_error > 0: # 현재 거리가 목표보다 멈, 전진 필요
            distance_to_travel_mm = r_error
            cmd = CMD_FWD # [수정] (self.FWD 아님)
        else: # 현재 거리가 목표보다 가까움, 후진 필요 (절대값 사용)
            distance_to_travel_mm = abs(r_error)
            cmd = CMD_REV # [수정] (self.REV 아님)
            
        # 4. 시간 계산 (속도: FORWARD_SPEED_MMPS)
        if self.forward_speed_mmps <= 0:
            print("[CLEAN] Open-Loop drive: forward speed is zero!")
            self._stop_and_clear(); return False
            
        required_time_s = distance_to_travel_mm / self.forward_speed_mmps
        
        # [수정] self.FWD -> CMD_FWD
        print(f"[CLEAN] Open-Loop drive: travel {distance_to_travel_mm:.0f}mm ({'FWD' if cmd == CMD_FWD else 'REV'}) in {required_time_s:.2f}s")
        
        # 5. 하드코딩된 시간 동안 명령 실행
        self._drive_for(cmd, required_time_s)
        
        # 6. 타임아웃/중단 확인
        if self.motor_stop_event.is_set() or (time.time() - t0) > timeout_s:
            self._stop_and_clear(); return False
            
        # 개루프이므로 목표 도달 여부는 확인하지 않고 시간 종료로 간주
        print("[CLEAN] Open-Loop drive: time elapsed, stopping.")
        self._stop_and_clear()
        return True

    def _drive_for(self, cmd, duration_s):
        """[수정됨] 주어진 UART 명령을 duration_s 동안 유지 전송 후 정지 (즉시 중단 가능)."""
        if duration_s <= 0:
            q_put_latest(self.q_tx, CMD_STOP)
            return

        t_end = time.time() + duration_s
        
        # 명령 재전송 주기
        resend_period = 0.05 
        
        while True:
            now = time.time()
            if now >= t_end:
                break # 시간 만료
                
            q_put_latest(self.q_tx, cmd)
            
            # 남은 시간과 재전송 주기 중 더 짧은 시간만큼 대기
            sleep_dur = min(resend_period, t_end - now)
            
            # [수정] time.sleep() 대신 _interruptible_sleep 사용
            if not self._interruptible_sleep(sleep_dur):
                break # STOP 이벤트로 즉시 중단됨
                
        q_put_latest(self.q_tx, CMD_STOP)
        
    def _stop_and_clear(self):
        """즉시 STOP을 두 번 전송하고 내부 전송 상태를 초기화."""
        q_put_latest(self.q_tx, CMD_STOP) # [수정]
        time.sleep(0.05)
        q_put_latest(self.q_tx, CMD_STOP) # [수정]
        self._last_cmd = None

    def _pre_adjust_radius(self, r_target=700.0, band=15.0, max_time_s=3.0):
        """r≈목표±band 범위에 들도록 짧게 전/후진 예비 보정."""
        t0 = time.time()
        while True:
            if self.motor_stop_event.is_set(): q_put_latest(self.q_tx, CMD_STOP); return False # [수정]
            if (time.time() - t0) > max_time_s: q_put_latest(self.q_tx, CMD_STOP); return True  # [수정] 여기선 관대히 통과

            r = self._snapshot_range_mm()
            if r is None: q_put_latest(self.q_tx, CMD_STOP); return False # [수정]

            if abs(r - r_target) <= band:
                q_put_latest(self.q_tx, CMD_STOP); return True # [수정]

            cmd = CMD_FWD if r > r_target else CMD_REV # [수정]
            self._drive_for(cmd, 0.08)
            if not self._interruptible_sleep(0.02): return False

    def _interruptible_sleep(self, duration_s):
        """ duration_s 동안 대기. motor_stop_event가 set되면 즉시 중단 """
        if duration_s <= 0:
            return True # 대기할 시간 없음

        # time.sleep() 대신 event.wait()를 사용하여 CPU 효율 증대
        # event.wait(timeout)는
        # - timeout이 만료되면 False 반환 (정상 종료)
        # - event가 set되면 True 반환 (중단됨)
        
        was_interrupted = self.motor_stop_event.wait(timeout=duration_s)
        
        if was_interrupted:
            return False # 중단됨
        else:
            return True # 대기 완료 (시간 만료)

    def _send_cmd_hold(self, cmd):
        """같은 명령은 주기 반복, 바뀔 땐 짧게 STOP→전환해 노이즈/스틱션 감소."""
        now = time.time()
        if cmd == self._last_cmd:
            q_put_latest(self.q_tx, cmd)
            return
        # 명령 전환 최소 간격
        if (now - self._last_switch_ts) < 0.05:
            return
        q_put_latest(self.q_tx, CMD_STOP) # [수정]
        time.sleep(0.02)
        q_put_latest(self.q_tx, cmd)
        self._last_cmd = cmd
        self._last_switch_ts = now

    def _rotate_to_target_deg(self, target_err_deg, stop_tol_deg, loop_dt, timeout_s):
        """
        [수정됨] 목표 각도(target_err_deg)를 중심으로, 오차(stop_tol_deg)
        범위에 들어갈 때까지 CW/CCW 양방향으로 제어합니다.
        """
        start_time = time.time()
        
        print(f"[CLEAN] Closed-Loop Rotate: Target {target_err_deg:.1f}° (Tol: ±{stop_tol_deg:.1f}°)")

        while (time.time() - start_time) < timeout_s:
            if self.motor_stop_event.is_set():
                self._stop_and_clear(); return False

            current_err_deg = self._snapshot_err_deg() 
            if current_err_deg is None:
                print("[CLEAN] Rotation Failed: Cannot get target angle.")
                return False # 시퀀스 중단

            # 1. 실제 오차 계산 (남은 각도)
            # 예: (목표 75) - (현재 60) = +15° (CCW 필요)
            # 예: (목표 75) - (현재 80) = -5°  (CW 필요)
            remaining_deg = target_err_deg - current_err_deg
            print(f"[CLEAN] Rotate Step: Current={current_err_deg:.1f}°, Target={target_err_deg:.1f}° (Err: {remaining_deg:.1f}°)")

            # 2. 정지 조건 확인 (목표 오차 범위 내)
            if abs(remaining_deg) <= stop_tol_deg:
                print(f"[CLEAN] Rotation Success: Reached {current_err_deg:.1f}° (Err: {remaining_deg:.1f}°).")
                self._stop_and_clear()
                return True

            # 3. 오차에 따라 회전 방향 결정
            # [수정] 시스템의 좌표계(CCW가 음수)에 맞춰 로직을 반전합니다.
            if remaining_deg > 0:
                cmd = CMD_CW  # [수정] (기존 CCW) -> 오차(Err)가 +면 CW(양의 각도)로
            else: 
                cmd = CMD_CCW # [수정] (기존 CW) -> 오차(Err)가 -면 CCW(음의 각도)로
                
            # 4. 명령 전송
            q_put_latest(self.q_tx, cmd)
            
            # 5. 센서 갱신 대기 (loop_dt는 clean_mode에서 0.15로 호출됨)
            if not self._interruptible_sleep(loop_dt): return False
            
        # 타임아웃
        print(f"[CLEAN] Rotation Failed: Timeout after {timeout_s}s.")
        return False

    @staticmethod
    def _wrap180(deg):
        """각도를 -180°~+180° 범위로 래핑."""
        return (deg + 180.0) % 360.0 - 180.0

    def _advance_chord_open_loop(self, distance_mm):
        """지정 거리만큼 시간 기반 전진(현 이동)."""
        if distance_mm <= 0:
            print("[CLEAN] Open-Loop chord: distance is zero.")
            self._stop_and_clear(); return True
            
        required_time_s = distance_mm / self.forward_speed_mmps
        
        print(f"[CLEAN] Open-Loop chord: travel {distance_mm:.0f}mm (FWD) in {required_time_s:.2f}s")
        
        self._drive_for(CMD_FWD, required_time_s) # [수정] self.FWD -> CMD_FWD

        if self.motor_stop_event.is_set():
                 self._stop_and_clear(); return False
                 
        return True

    def _chord_len_mm(self, r_mm, angle_div):
        """반지름 r과 분할각으로 현 길이(mm) 계산."""
        if angle_div <= 0 or r_mm <= 0:
            return 0.0
        return 2.0 * r_mm * math.sin(math.pi / float(angle_div))

    def _lin_to_cmd(self, v_pwm: float) -> str:
        """선형 속도 PWM(부호=방향)을 좌/우 바퀴 UART 문자열로 매핑."""
        v = int(abs(v_pwm))
        if v == 0:
            return CMD_STOP # [수정]
        if v < self.move_min_pwm: v = self.move_min_pwm
        if v > self.move_max_pwm: v = self.move_max_pwm
        if v_pwm > 0:
            # 전진: 좌/우 정방향
            return f"1,{v},1,{v},0"
        else:
            # 후진: 좌/우 역방향
            return f"0,{v},0,{v},0"

    def _slew(self, target: float, prev: float, rate_per_s: float, dt: float) -> float:
        """1초당 변경 한계를 둔 가감속 제한(연속 슬루)."""
        if dt <= 0:
            return target
        max_step = rate_per_s * dt
        if target > prev + max_step:  return prev + max_step
        if target < prev - max_step:  return prev - max_step
        return target

    def _approach_to_range_smooth(self,
                                r_target=700.0,
                                tol_mm=6.0,
                                vmax_pwm=280,
                                err_scale_mm=120.0,   # tanh 스케일: 120mm 정도 권장
                                accel_pwm_per_s=800,  # 슬루율
                                flip_hyst_mm=22.0,    # 전↔후 바꾸기 전 요구하는 추가 오차
                                latch_time_s=0.6,     # 정지 래치 유지 시간
                                max_rate_stop_mmps=25.0,  # 정지 판정용 |d(r)/dt| 한계
                                loop_dt=0.05,
                                timeout_s=20.0,
                                kp_pwm_per_mm=0.9,    # 기존의 인자 추가
                                v_pwm_max=280):       # 추가된 인자: v_pwm_max
        """tanh 맵+슬루+히스테리시스+정지 래치로 부드럽게 700mm 접근(폐루프)."""
        print(f"[CLEAN] Smooth approach → {r_target:.0f}mm (tol ±{tol_mm:.0f}mm)")
        t0 = time.time()
        self._lin_pwm_prev = 0.0
        self._stop_latched = False
        self._last_dir = 0
        self._last_dir_change = 0.0

        # 리프랙토리(방향 변경 후 최소 유지) 시간
        refractory_s = 0.40

        while True:
            # 인터럽트/타임아웃
            if self.motor_stop_event.is_set():
                self._stop_and_clear(); return False
            if (time.time() - t0) > timeout_s:
                print("[CLEAN] Smooth approach: timeout")
                self._stop_and_clear(); return False

            r_f = self._filtered_range_mm()
            if r_f is None:
                print("[CLEAN] Smooth approach: target lost")
                self._stop_and_clear(); return False

            # 절대값 차이로 오차 계산
            err = abs(r_target - r_f)  
            rate = self._range_rate_mmps(r_f)

            now = time.time()

            # ---- 정지 래치: 한번 멈추면 소폭 흔들림 무시 ----
            if self._stop_latched:
                if now < self._stop_latch_until:
                    # 래치 유지 구간: 명령 반복 전송만
                    self._send_cmd_hold(self.STOP)
                    if not self._interruptible_sleep(loop_dt):
                        self._stop_and_clear(); return False
                    continue
                else:
                    # 래치 해제
                    self._stop_latched = False

            # ---- 정지 판정: tol 안 + 속도도 거의 0 ----
            if (abs(err) <= tol_mm) and (abs(rate) <= max_rate_stop_mmps) and (abs(self._lin_pwm_prev) <= self.move_min_pwm):
                print(f"[CLEAN] Smooth approach: stop at {r_f:.1f}mm (err={err:.1f}, |dr/dt|≈{rate:.1f})")
                self._stop_and_clear()
                self._stop_latched = True
                self._stop_latch_until = now + latch_time_s
                if not self._interruptible_sleep(loop_dt):
                    self._stop_and_clear(); return False
                continue

            # ---- 방향 히스테리시스/리프랙토리 ----
            want_dir = 0
            if err > tol_mm:     # 전진 필요
                want_dir = +1
            elif err < -tol_mm:  # 후진 필요
                want_dir = -1
            else:
                want_dir = 0

            # 방향을 바꾸려면 더 큰 오차(히스테리시스) 필요 + 최근 변경 후 일정 시간
            if (want_dir != 0) and (self._last_dir != 0) and (want_dir != self._last_dir):
                if (abs(err) < (tol_mm + flip_hyst_mm)) or ((now - self._last_dir_change) < refractory_s):
                    # 바로 뒤집지 말고 '정지 또는 아주 작은 pwm' 유지
                    want_dir = self._last_dir  # 유지

            # ---- 비선형 속도 맵 (tanh) ----
            v_des = vmax_pwm * math.tanh(err / max(1e-6, err_scale_mm))
            # deadzone: tol 근방에서는 0으로 수렴(바닥 PWM 강제 진입 방지)
            if abs(err) <= tol_mm:
                v_des = 0.0

            # 방향 적용
            if want_dir == 0:
                v_des = 0.0
            else:
                v_des = abs(v_des) * (1 if want_dir > 0 else -1)

            # 슬루(연속 가감속 제한)
            v_cmd = self._slew(v_des, self._lin_pwm_prev, accel_pwm_per_s, loop_dt)

            # 스틱션 킥: 정지→움직임 전환 시 아주 짧게 최소보다 조금 크게
            if (abs(self._lin_pwm_prev) < 1e-3) and (abs(v_cmd) > 0):
                kick = max(self.move_min_pwm + 30, min(self.move_max_pwm, int(abs(v_cmd))+10))
                kick_cmd = self._lin_to_cmd(kick if v_cmd > 0 else -kick)
                self._send_cmd_hold(kick_cmd)
                time.sleep(0.08)  # 짧게 킥
                # 킥 후 목표 v_cmd로 이어감

            self._lin_pwm_prev = v_cmd

            # 실제 전송
            self._send_cmd_hold(self._lin_to_cmd(v_cmd))

            # 방향 변경 시각 기록
            new_dir = 0 if abs(v_cmd) < 1e-3 else (1 if v_cmd > 0 else -1)
            if new_dir != self._last_dir:
                self._last_dir = new_dir
                self._last_dir_change = now

            if not self._interruptible_sleep(loop_dt):
                self._stop_and_clear(); return False

    def _filtered_range_mm(self, alpha=0.18, med_win=5):
        """거리 r에 미디안+IIR 결합 필터 적용값 반환."""
        r = self._snapshot_range_mm()
        if r is None:
            return None

        # 미디안
        self._r_med.append(r)
        if len(self._r_med) > med_win:
            self._r_med.pop(0)
        r_med = sorted(self._r_med)[len(self._r_med)//2]

        # IIR
        if self._filt_r is None:
            self._filt_r = r_med
        else:
            self._filt_r = (1 - alpha)*self._filt_r + alpha*r_med
        return self._filt_r

    def _snapshot_filtered(self, alpha_ang=0.17, alpha_r=0.17):
        """각도/거리 1차 IIR로 저역통과 필터링 스냅샷 반환."""
        ang = self._snapshot_err_deg()
        r   = self._snapshot_range_mm()
        if ang is None or r is None:
            return None, None
        if self._filt_ang is None:
            self._filt_ang, self._filt_r = ang, r
        else:
            self._filt_ang = (1 - alpha_ang)*self._filt_ang + alpha_ang*ang
            self._filt_r   = (1 - alpha_r  )*self._filt_r   + alpha_r  *r
        self._filt_ts = time.time()
        return self._filt_ang, self._filt_r

    @staticmethod
    def _target_angle_from_front_deg(x_mm, y_mm):
        """(x,y) 타깃 좌표를 로봇 정면 기준 방위각(도)으로 변환."""
        return CleanController._wrap180(-math.degrees(math.atan2(x_mm, y_mm)))

    def _spin_proportional(self, err_deg):
        """전체 회전 시간 비례로 오차 각도만큼 좌/우 회전(개루프)."""
        dur = (abs(err_deg) / 360.0) * self.full_turn_s
        cmd = (CMD_CCW if err_deg > 0 else CMD_CW) # [수정]
        self._drive_for(cmd, dur)

    def _spin_ccw_deg(self, deg):
        """CCW 방향으로 절대 각도만큼 시간 비례 회전."""
        if deg <= 0:
            return
        dur = (deg / 360.0) * self.full_turn_s
        self._drive_for(CMD_CCW, dur) # [수정]

    def _range_rate_mmps(self, r_now):
        """최근 r 변화로 dr/dt(mm/s) 추정."""
        now = time.time()
        if (self._last_r is None) or (now == self._last_r_ts):
            self._last_r, self._last_r_ts = r_now, now
            return 0.0
        dr = r_now - self._last_r
        dt = now - self._last_r_ts
        self._last_r, self._last_r_ts = r_now, now
        return dr / dt if dt > 1e-3 else 0.0

class TrackController:
    def __init__(self, tx_queue, motor_stop_event):
        self.q_tx = tx_queue
        self.motor_stop_event = motor_stop_event
        self._th = None

        # --- [신규] CleanController의 궤도(Orbit) 파라미터 이식 ---
        # [수정] self.orbit_target_deg는 track_mode에서 동적으로 설정됩니다.
        self.orbit_target_dist_mm = 800.0    # [요청] 목표 거리 800mm
        self.orbit_loop_dt = 0.1             # 궤도 제어 루프 주기 (10Hz)
        # [수정] L/R -> Outer/Inner로 변경
        self.orbit_base_pwm_outer = 150 # 바깥쪽 바퀴 (빠른 쪽)
        self.orbit_base_pwm_inner = 130 # 안쪽 바퀴 (느린 쪽)
        self.orbit_kp = 0.15              # 거리 오차 비례 게인 (PWM / mm)
        self.orbit_max_pwm = 300
        self.orbit_min_pwm = 100
        self.orbit_accel_pwm_per_s = 300.0
        
        # --- [신규] 궤도 선회 상태 변수 ---
        self._orbit_l_pwm_prev = 0.0
        self._orbit_r_pwm_prev = 0.0
        self._is_orbiting = False # 현재 궤도 선회 중인지 여부
        self.allow_beautician_recognition = False # [신규] 미용사 인식 허용 플래그

        # [수정] 동적으로 설정될 변수 (track_mode에서 덮어씀)
        self.orbit_target_deg = -90.0
        
        # --- [수정] 스캔 상태 변수 (이전 방식으로 복귀) ---
        self._scan_index = 0 # 스캔할 각도 인덱스 (0, 1, 2)
        self._scan_angles_deg = [0, 30, -30] # 스캔할 각도 리스트
        self._scan_wait_until_ts = 0.0 # 스캔 대기 종료 시각

    def start_sequence(self):
        """[수정] TRACK 시퀀스를 별도 데몬 스레드로 시작합니다."""
        if self._th and self._th.is_alive():
            print("[TRACK] 이미 시퀀스가 실행 중입니다.")
            return

        print("[TRACK] 새 시퀀스 스레드 시작...")
        
        # --- [신규] TRACK 버튼 클릭 시 미용사 추적 상태 초기화 ---
        beautician_tracker.reset()
        # --- [신규 끝] ---
        
        self.motor_stop_event.clear() 
        self.allow_beautician_recognition = False # [신규] 시퀀스 시작 시 반드시 인식 중지
        self._th = threading.Thread(target=self.track_mode, daemon=True, name="track_seq")
        self._th.start()
        
    def _interruptible_sleep(self, duration_s):
        """ duration_s 동안 대기. motor_stop_event가 set되면 즉시 False 반환 """
        if duration_s <= 0:
            return True
        was_interrupted = self.motor_stop_event.wait(timeout=duration_s)
        return not was_interrupted # True: 대기 완료, False: 중단됨

    # --- [공통 헬퍼] ---
    def _stop_and_clear(self):
        """즉시 STOP을 전송."""
        q_put_latest(self.q_tx, CMD_STOP)

    @staticmethod
    def _wrap180(deg):
        return (deg + 180.0) % 360.0 - 180.0

    @staticmethod
    def _target_angle_from_front_deg(x_mm, y_mm):
        return TrackController._wrap180(-math.degrees(math.atan2(x_mm, y_mm)))
    
    # --- [의자(Chair) 상태 읽기 헬퍼] ---
    def _snapshot_err_deg_chair(self):
        tgt = target_state.get() # 전역 의자 상태
        if not tgt:
            return None
        return self._target_angle_from_front_deg(tgt["x_mm"], tgt["y_mm"])

    def _snapshot_range_mm_chair(self):
        tgt = target_state.get() # 전역 의자 상태
        if not tgt:
            return None
        return math.hypot(tgt["x_mm"], tgt["y_mm"])

    # --- [미용사(Beautician) 상태 읽기 헬퍼] ---
    def _snapshot_err_deg_beautician(self):
        tgt = beautician_state.get() 
        if not tgt:
            return None
        return self._target_angle_from_front_deg(tgt["x_mm"], tgt["y_mm"])

    def _snapshot_range_mm_beautician(self):
        """[신규] 미용사 타깃(beautician_state)으로부터 현재 거리(mm)을 스냅샷."""
        tgt = beautician_state.get() # 전역 미용사 상태
        if not tgt:
            return None
        return math.hypot(tgt["x_mm"], tgt["y_mm"])
        
    # --- [의자(Chair) 제어 헬퍼] ---
    def _rotate_to_target_deg_chair(self, target_err_deg, stop_tol_deg, loop_dt, timeout_s):
        """[신규] 의자(target_state)를 기준으로 목표 각도까지 회전"""
        start_time = time.time()
        print(f"[TRACK] (Chair) Closed-Loop Rotate: Target {target_err_deg:.1f}° (Tol: ±{stop_tol_deg:.1f}°)")

        while (time.time() - start_time) < timeout_s:
            if self.motor_stop_event.is_set():
                self._stop_and_clear(); return False

            current_err_deg = self._snapshot_err_deg_chair() # 의자 헬퍼 사용
            if current_err_deg is None:
                print("[TRACK] (Chair) Rotation Failed: Cannot get target angle.")
                self._stop_and_clear(); return False

            remaining_deg = target_err_deg - current_err_deg
            
            if abs(remaining_deg) <= stop_tol_deg:
                print(f"[TRACK] (Chair) Rotation Success.")
                self._stop_and_clear()
                return True

            cmd = CMD_CW if remaining_deg > 0 else CMD_CCW
            q_put_latest(self.q_tx, cmd)
            if not self._interruptible_sleep(loop_dt): return False
            
        print(f"[TRACK] (Chair) Rotation Failed: Timeout.")
        return False

    def _drive_until_target_range_fwd(self, r_target, tol_mm, loop_dt, timeout_s):
        """[신규] 의자(target_state)를 기준으로 목표 거리(r_target)까지 전/후진"""
        start_time = time.time()
        print(f"[TRACK] (Chair) Range Adjust (FWD/REV): Target r={r_target:.0f}mm (Tol: ±{tol_mm:.0f}mm)")
        is_first_loop = True

        while (time.time() - start_time) < timeout_s:
            if self.motor_stop_event.is_set():
                self._stop_and_clear(); return False

            r_current = self._snapshot_range_mm_chair() # 의자 헬퍼 사용
            if r_current is None:
                print("[TRACK] (Chair) Range Adjust: Target lost.")
                self._stop_and_clear(); return False

            remaining_mm = r_current - r_target 
            
            if not is_first_loop and abs(remaining_mm) <= tol_mm:
                print(f"[TRACK] (Chair) Range Adjust Success.")
                self._stop_and_clear()
                return True

            if is_first_loop and abs(remaining_mm) <= tol_mm:
                cmd = CMD_FWD 
            elif remaining_mm > 0:
                cmd = CMD_FWD # 너무 멈. 전진.
            else:
                cmd = CMD_REV  # 너무 가까움. 후진.
                        
            q_put_latest(self.q_tx, cmd)
            if not self._interruptible_sleep(loop_dt): return False
            is_first_loop = False

        print(f"[TRACK] (Chair) Range Adjust Failed: Timeout.")
        return False
        
    # --- [신규] 궤도(Orbit) 제어 헬퍼 (CleanController에서 이식) ---
    def _slew(self, target: float, prev: float, rate_per_s: float, dt: float) -> float:
        """1초당 변경 한계를 둔 가감속 제한(연속 슬루)."""
        if dt <= 0:
            return target
        max_step = rate_per_s * dt
        if target > prev + max_step:  return prev + max_step
        if target < prev - max_step:  return prev - max_step
        return target

    def _run_orbit_step(self, target_dist_mm, loop_dt, is_cw=True):
        """[수정] 지정된 목표 거리(target_dist_mm)를 유지하며 P제어 원운동을 1 스텝(loop_dt) 수행합니다. (방향 플래그 추가)"""
        
        # 1. 현재 (의자) 거리 측정
        r_current = self._snapshot_range_mm_chair()
        if r_current is None:
            print(f"[TRACK] Orbit Step (->{target_dist_mm}mm) target lost. Stopping.")
            return False 

        # 2. 거리 오차 계산
        err_dist = r_current - target_dist_mm
        
        # 3. P-제어 (L/R Wheel Target)
        adjustment = self.orbit_kp * err_dist
        
        # [수정] CW/CCW 방향에 따라 P제어 로직 분기
        if is_cw:
            # 시계방향 (L: Outer, R: Inner)
            base_l = self.orbit_base_pwm_outer
            base_r = self.orbit_base_pwm_inner
            # 멀어지면(adj > 0): L 증가, R 감소 (더 안쪽으로 휨)
            l_pwm_target = base_l + adjustment
            r_pwm_target = base_r - adjustment
        else:
            # 반시계방향 (L: Inner, R: Outer)
            base_l = self.orbit_base_pwm_inner
            base_r = self.orbit_base_pwm_outer
            # 멀어지면(adj > 0): L 감소, R 증가 (더 안쪽으로 휨)
            l_pwm_target = base_l - adjustment
            r_pwm_target = base_r + adjustment
        
        # 4. Slew (가감속 제한) 적용
        l_pwm_slew = self._slew(
            l_pwm_target, 
            self._orbit_l_pwm_prev, 
            self.orbit_accel_pwm_per_s, 
            loop_dt
        )
        r_pwm_slew = self._slew(
            r_pwm_target,
            self._orbit_r_pwm_prev,
            self.orbit_accel_pwm_per_s,
            loop_dt
        )

        # 5. PWM 값 클리핑 (최소/최대)
        l_pwm_final = int(np.clip(
            l_pwm_slew, 
            self.orbit_min_pwm, 
            self.orbit_max_pwm
        ))
        r_pwm_final = int(np.clip(
            r_pwm_slew,
            self.orbit_min_pwm, 
            self.orbit_max_pwm
        ))

        # 6. 이전 값 갱신 (중요: Slew 상태가 갱신됨)
        self._orbit_l_pwm_prev = l_pwm_slew
        self._orbit_r_pwm_prev = r_pwm_slew

        # 7. UART 명령 생성 (항상 전진)
        cmd = f"1,{l_pwm_final},1,{r_pwm_final},0"
        q_put_latest(self.q_tx, cmd)
        
        if DEBUG_TRACK:
            print(f"[TRACK] Orbit(->{target_dist_mm}, CW={is_cw}): r={r_current:.0f}(Err:{err_dist:.0f}) -> L_Cmd:{l_pwm_final} | R_Cmd:{r_pwm_final}")

        return True # 스텝 성공

    def _ramp_down_orbit(self, ramp_duration_s, loop_dt=0.05):
        """현재 궤도 속도(_orbit_l/r_pwm_prev)에서 0까지 부드럽게 감속합니다."""
        print(f"[TRACK] Ramping down orbit speed over {ramp_duration_s}s...")
        t_start = time.time()
        
        while (time.time() - t_start) < ramp_duration_s:
            if self.motor_stop_event.is_set():
                print("[TRACK] Ramp down interrupted.")
                return False 

            l_pwm_slew = self._slew(0.0, self._orbit_l_pwm_prev, self.orbit_accel_pwm_per_s, loop_dt)
            r_pwm_slew = self._slew(0.0, self._orbit_r_pwm_prev, self.orbit_accel_pwm_per_s, loop_dt)

            self._orbit_l_pwm_prev = l_pwm_slew
            self._orbit_r_pwm_prev = r_pwm_slew

            l_pwm_final = int(np.clip(l_pwm_slew, 0, self.orbit_max_pwm))
            r_pwm_final = int(np.clip(r_pwm_slew, 0, self.orbit_max_pwm))

            cmd = f"1,{l_pwm_final},1,{r_pwm_final},0"
            q_put_latest(self.q_tx, cmd)

            if l_pwm_final == 0 and r_pwm_final == 0:
                break # while 루프 탈출
            
            if not self._interruptible_sleep(loop_dt):
                return False # 중단됨
        
        # 7. 루프 종료 후 최종 정지 명령 및 Slew 상태 강제 리셋
        q_put_latest(self.q_tx, CMD_STOP)
        self._orbit_l_pwm_prev = 0.0
        self._orbit_r_pwm_prev = 0.0
        print("[TRACK] Ramp down finalized.")
        return True
    # --- [헬퍼 끝] ---
    
    def _wait_for_beautician(self, duration_s, check_dist_mm=1000.0):
        """
        [신규] 지정된 시간(duration_s) 동안 미용사가 인식되는지(check_dist_mm 이내) 
        0.1초마다 확인합니다.
        인식되면 True, 시간이 만료되면 False를 반환합니다.
        """
        t_start = time.time()
        while (time.time() - t_start) < duration_s:
            if self.motor_stop_event.is_set():
                print("[TRACK] Wait for beautician interrupted.")
                return False # 중단됨

            tgt = beautician_state.get() 
            if tgt is not None:
                dist_mm = tgt["dist_mm"]
                if dist_mm <= check_dist_mm:
                    # 찾았음!
                    angle = self._snapshot_err_deg_beautician()
                    print(f"[TRACK] Scan SUCCESS: Beautician found at {dist_mm:.0f}mm (Angle: {angle}).")
                    return True # 찾았음

            # 0.1초 대기
            if not self._interruptible_sleep(0.1): 
                return False # 중단됨
        
        return False # 타임아웃

    def start_return_sequence(self):
        """[신규] '정렬 후 800mm 접근' 시퀀스를 별도 스레드로 시작합니다."""
        
        # 다른 시퀀스가 종료되는 것을 보장하기 위해 1초 지연은
        # 이 함수를 호출하는 쪽 (GUI, TCP)에서 이미 처리했습니다.
        if self._th and self._th.is_alive():
            print(f"[RETURN] 경고: 이전 시퀀스({self._th.name})가 아직 실행 중일 수 있습니다.")
            # return # (일단 강행)

        print("[RETURN] 새 '정렬 및 접근' 시퀀스 스레드 시작...")
        
        # 새 동작이므로 motor_stop_event를 다시 해제합니다.
        self.motor_stop_event.clear()
        
        # 이 시퀀스 중에는 미용사를 인식할 필요가 없습니다.
        self.allow_beautician_recognition = False 
        
        self._th = threading.Thread(target=self._run_return_sequence, daemon=True, name="return_seq")
        self._th.start()

    def _run_return_sequence(self):
        """[신규] (스레드 워커) 0도 정렬 -> 800mm 접근"""
        try:
            print("[RETURN] 시퀀스 시작...")
            
            # --- 공용 파라미터 ---
            LOOP_DT = self.orbit_loop_dt # 0.1초
            ALIGN_TOL_DEG = 5.0
            
            # --- 1. 의자 정렬 (목표 각도 0도) ---
            print("[RETURN] 1. (Chair) 0도 정렬 시도...")
            
            # 이전 STOP 명령 후 모터가 완전히 설 때까지 잠시 대기
            if not self._interruptible_sleep(0.5): return 

            ok = self._rotate_to_target_deg_chair(
                target_err_deg=0.0,
                stop_tol_deg=ALIGN_TOL_DEG,
                loop_dt=LOOP_DT,
                timeout_s=15.0
            )
            if not ok: 
                print("[RETURN] 1. 정렬 실패 또는 중지됨.")
                return # (finally에서 STOP 처리)
            
            if not self._interruptible_sleep(0.5): return

            # --- 2. 의자 접근 (800mm) ---
            target_dist_mm = 800.0
            print(f"[RETURN] 2. (Chair) {target_dist_mm:.0f}mm로 거리 조절 시도...")
            ok = self._drive_until_target_range_fwd(
                r_target=target_dist_mm,
                tol_mm=50.0, # +- 5cm
                loop_dt=LOOP_DT,
                timeout_s=20.0
            )
            if not ok:
                print("[RETURN] 2. 거리 조절 실패 또는 중지됨.")
                return # (finally에서 STOP 처리)

            print("[RETURN] 3. 시퀀스 완료.")

        except Exception as e:
            print(f"[RETURN] 시퀀스 실행 중 예외 발생: {e}")
        finally:
            print("[RETURN] 스레드 종료 (최종 STOP 전송)")
            q_put_latest(self.q_tx, CMD_STOP)

    def track_mode(self):
        """ [수정됨] TRACK 시퀀스: 1.정렬 -> 2.접근 -> 3.최초 방향설정 -> [ (최우선 방향전환) -> (스캔/추적) ] 반복 """
        try:
            print("[TRACK] 시퀀스 시작...")
            
            # --- 공용 파라미터 ---
            LOOP_DT = self.orbit_loop_dt # 0.1초 (10Hz)
            ALIGN_TOL_DEG = 5.0
            
            DIRECTION_SWITCH_ABS_DEG = 105.0
            
            # --- 1. 의자 정렬 (목표 각도 0도) ---
            print("[TRACK] 1. (Chair) Starting Alignment (Target: 0 deg)...")
            ok = self._rotate_to_target_deg_chair(
                target_err_deg=0.0,
                stop_tol_deg=ALIGN_TOL_DEG,
                loop_dt=LOOP_DT,
                timeout_s=15.0
            )
            if not ok: return
            if not self._interruptible_sleep(0.5): return

            # --- 2. 의자 접근 (800mm) ---
            target_dist_mm = 800.0 # self.orbit_target_dist_mm 값 사용
            print(f"[TRACK] 2. (Chair) Range approach to {target_dist_mm:.0f}mm...")
            ok = self._drive_until_target_range_fwd(
                r_target=target_dist_mm,
                tol_mm=50.0, # +- 5cm
                loop_dt=LOOP_DT,
                timeout_s=20.0
            )
            if not ok: return
            if not self._interruptible_sleep(0.5): return

            # --- 3. [신규] 통합 추적/스캔 루프 ---
            print("[TRACK] 3. Enabling Beautician Recognition and starting unified loop.")
            self.allow_beautician_recognition = True
            
            dynamic_orbit_r_mm = self._snapshot_range_mm_chair()
            if dynamic_orbit_r_mm is None:
                dynamic_orbit_r_mm = target_dist_mm # 2단계 목표값
                print(f"[TRACK] 3. Chair lost. Falling back to default r={dynamic_orbit_r_mm:.0f}mm")
            else:
                print(f"[TRACK] 3. Measured r={dynamic_orbit_r_mm:.0f}mm. Using this as orbit target.")

            # [신규] 궤도 방향 (CW/CCW) 및 목표 각도 (-90/90)
            orbit_is_cw = False
            self.orbit_target_deg = 90.0

            realign_interval_s = 2.0
            
            self._is_orbiting = False # 궤도 선회 정지 상태 (스캔 모드)에서 시작
            self._scan_index = 0
            
            last_realign_ts = time.time() # [중요] 재정렬 타이머는 *지금* 시작
            self._orbit_l_pwm_prev = 0.0 # Slew 리셋
            self._orbit_r_pwm_prev = 0.0
            # self._is_orbiting는 루프 안에서 결정됨
            
            # --- [메인 루프 시작] ---
            while not self.motor_stop_event.is_set():
                
                # A. 미용사 상태 스냅샷 (Angle FIRST)
                angle_beautician = self._snapshot_err_deg_beautician()
                dist_beautician = self._snapshot_range_mm_beautician()
                
                if angle_beautician is not None and DEBUG_TRACK:
                    print(f"[TRACK_DBG] Current Beautician Angle: {angle_beautician:.1f} deg")

                # --- [상태 A: 미용사 발견 (추적 모드)] ---
                if dist_beautician is not None:
                    
                    # [수정] B. 궤도 선회 중인지를 *먼저* 확인
                    if self._is_orbiting:
                        
                        # B-1. 거리 기반 정지 (650mm)
                        if dist_beautician < 650.0:
                            # (ORBIT -> STOP)
                            print(f"[TRACK] State Change: ORBIT -> STOP (Dist: {dist_beautician:.0f}mm)")
                            if not self._ramp_down_orbit(ramp_duration_s=1.0, loop_dt=LOOP_DT):
                                return
                            self._is_orbiting = False
                            continue # 다음 루프 (정지 상태로)

                        # B-2. [유지] 2초마다 재정렬
                        now = time.time()
                        if (now - last_realign_ts) >= realign_interval_s:
                            print(f"[TRACK] Orbit: Re-aligning (2-sec interval).")
                            if not self._ramp_down_orbit(0.5, LOOP_DT): return
                            
                            # [신규] 재정렬 직전에 의자 거리 재측정
                            measured_r = self._snapshot_range_mm_chair()
                            if measured_r is not None:
                                dynamic_orbit_r_mm = measured_r
                                print(f"[TRACK] Re-measured r={dynamic_orbit_r_mm:.0f}mm. Using this.")
                            
                            ok = self._rotate_to_target_deg_chair(
                                target_err_deg=self.orbit_target_deg, stop_tol_deg=ALIGN_TOL_DEG,
                                loop_dt=LOOP_DT, timeout_s=10.0
                            )
                            if not ok: return
                            self._orbit_l_pwm_prev = 0.0
                            self._orbit_r_pwm_prev = 0.0
                            last_realign_ts = time.time()
                        else:
                            # B-3. 궤도 스텝 실행
                            # [신규] 궤도 스텝 직전에 의자 거리 재측정
                            measured_r = self._snapshot_range_mm_chair()
                            if measured_r is not None:
                                dynamic_orbit_r_mm = measured_r
                            
                            if not self._run_orbit_step(dynamic_orbit_r_mm, LOOP_DT, orbit_is_cw):
                                print("[TRACK] Orbit: run_orbit_step failed (Chair lost?). Stopping.")
                                if not self._ramp_down_orbit(0.5, LOOP_DT): return
                                self._is_orbiting = False
                                print("[TRACK] Orbit stopped. Holding position until chair is re-acquired.")
                                continue # 메인 루프로
                    
                    # [수정] C. 궤도 선회 중이 *아닐* 때 (정지/스캔 상태)
                    else: 
                        # C-1. 105도 방향 전환 (정지 상태에서만 체크)
                        switch_triggered = False
                        if angle_beautician is not None and abs(angle_beautician) > DIRECTION_SWITCH_ABS_DEG:
                            if angle_beautician > 0: # (> +105)
                                if not orbit_is_cw: # 현재 CCW면 CW로 전환
                                    print(f"[TRACK] Direction Switch (STOPPED): angle {angle_beautician:.1f} > {DIRECTION_SWITCH_ABS_DEG}. Switching CCW -> CW.")
                                    orbit_is_cw = True
                                    self.orbit_target_deg = -90.0
                                    switch_triggered = True
                            else: # (< -105)
                                if orbit_is_cw: # 현재 CW면 CCW로 전환
                                    print(f"[TRACK] Direction Switch (STOPPED): angle {angle_beautician:.1f} < -{DIRECTION_SWITCH_ABS_DEG}. Switching CW -> CCW.")
                                    orbit_is_cw = False
                                    self.orbit_target_deg = 90.0
                                    switch_triggered = True

                        # ################# [수정#블록#시작] #################
                        # C-2. 거리 기반 궤도 시작 (650mm)
                        if dist_beautician >= 650.0:
                            # (STOP -> ORBIT)
                            print(f"[TRACK] State Change: STOP -> ORBIT (Resuming) (Dist: {dist_beautician:.0f}mm)")
                            
                            # [수정] 궤도를 시작할 때는(STOP -> ORBIT) 2초 대기 없이 즉시 재정렬을 수행합니다.
                            print(f"[TRACK] Re-aligning to new target angle: {self.orbit_target_deg} deg")
                            
                            # [신규] 재정렬 직전에 의자 거리 재측정
                            measured_r = self._snapshot_range_mm_chair()
                            if measured_r is not None:
                                dynamic_orbit_r_mm = measured_r
                                print(f"[TRACK] Re-measured r={dynamic_orbit_r_mm:.0f}mm. Using this.")
                            
                            ok = self._rotate_to_target_deg_chair(
                                target_err_deg=self.orbit_target_deg, stop_tol_deg=ALIGN_TOL_DEG,
                                loop_dt=LOOP_DT, timeout_s=10.0
                            )
                            if not ok: return
                        
                            self._orbit_l_pwm_prev = 0.0
                            self._orbit_r_pwm_prev = 0.0
                            self._is_orbiting = True
                            last_realign_ts = time.time() # 2초 재정렬 타이머 리셋
                        # ################# [수정#블록#종료] #################
                        
                        # C-3. 정지 상태 유지
                        else:
                            q_put_latest(self.q_tx, CMD_STOP)
                            # (정지 상태에서는 2초 타이머 갱신 안함)

                # --- [상태 B: 미용사 놓침 (스캔 모드)] ---
                else:
                    # (1) 궤도 선회 중이었다면 즉시 정지 (모드 전환)
                    if self._is_orbiting:
                        print(f"[TRACK] Beautician LOST. Stopping orbit and switching to SCAN mode.")
                        if not self._ramp_down_orbit(ramp_duration_s=1.0, loop_dt=LOOP_DT):
                            return # (Ramp down interrupted by STOP)
                        self._is_orbiting = False # Ramp down이 Slew만 리셋하므로, 상태 플래그는 여기서 변경
                        
                        print("[TRACK] Scan: Resetting scan index to 0 (Front).")
                        self._scan_index = 0
                        last_realign_ts = time.time() # 스캔 시작 시 타이머 리셋
                    
                    # (2) 3-Point 스캔 로직 (0, 30, -30)
                    target_angle = self._scan_angles_deg[self._scan_index]

                    # 2-2. 해당 각도로 회전
                    print(f"[TRACK] Scan: Rotating to scan angle {target_angle} deg...")
                    ok = self._rotate_to_target_deg_chair(
                        target_err_deg=target_angle,
                        stop_tol_deg=ALIGN_TOL_DEG,
                        loop_dt=LOOP_DT,
                        timeout_s=15.0
                    )
                    if not ok:
                        print("[TRACK] Scan failed: Chair LOST during rotation. Waiting here...")
                        if not self._interruptible_sleep(3.0): return
                    else:
                        # (의자가 있으면 2초 대기하며 미용사 찾기)
                        print(f"[TRACK] Scan: Holding at {target_angle} deg for 2 seconds...")
                        found = self._wait_for_beautician(duration_s=2.0, check_dist_mm=1500.0)

                        if found:
                            print("[TRACK] Scan SUCCESS. Switching back to TRACK mode.")
                            self._scan_index = 0
                            continue # 메인 while 루프의 처음으로 (상태 A로 진입)
                    
                    # 2-5. (못 찾았거나 의자가 없었으면) 다음 스캔 인덱스로 이동
                    print(f"[TRACK] Scan: Not found at {target_angle} deg. Moving to next angle.")
                    self._scan_index = (self._scan_index + 1) % len(self._scan_angles_deg)

                # --- [메인 루프 대기] ---
                if not self._interruptible_sleep(LOOP_DT):
                    break # (STOP 눌림)
            
            # --- (메인 'while' 루프 끝) ---
            
        except Exception as e:
            print(f"[TRACK] 시퀀스 실행 중 예외 발생: {e}")
        finally:
            print("[TRACK] Loop exiting. Ramping down...")
            self.allow_beautician_recognition = False # [신규] 스레드 종료 시 인식 중지
            
            self._ramp_down_orbit(0.5) # (상태와 관계없이 0으로 감속)
            
            q_put_latest(self.q_tx, CMD_STOP)
            print("[TRACK] 스레드 종료 (최종 STOP 전송)")
            
            
            
            
# STM32로 보낼 문자열을 큐에서 꺼내 최소 간격을 지켜 전송하고, 오류 시 포트를 닫아 재연결
class UartTxWorker(threading.Thread):
    """tx_q에서 문자열 명령을 꺼내 STM32로 전송하는 전용 스레드"""
    def __init__(self, stop_event, port, baud, in_q, reconnect_wait=1.0, min_interval_s=0.05):
        super().__init__(daemon=False, name="uart_tx")
        self.stop_event = stop_event
        self.port = port
        self.baud = baud
        self.in_q = in_q
        self.reconnect_wait = reconnect_wait
        self.ser = None
        self.min_interval_s = min_interval_s
        self._last_send = 0.0

    def _ensure_open(self):
        """시리얼 포트가 닫혀 있으면 재연결을 시도하고 성공 여부를 반환."""
        if self.ser and self.ser.is_open:
            return True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print(f"[UART] Opened {self.port} @ {self.baud}")
            return True
        except Exception as e:
            print(f"[UART] open error: {e}")
            return False

    def run(self):
        """큐(in_q)에서 명령 문자열을 꺼내 STM32로 주기적·안전하게 전송하는 메인 송신 루프, 오류 시 자동 재연결 및 종료 시 포트 정리 수행."""
        while not self.stop_event.is_set():
            if not self._ensure_open():
                time.sleep(self.reconnect_wait)
                continue
            try:
                msg = self.in_q.get(timeout=0.1)
            except queue.Empty:
                continue

            if not msg.endswith("\n"):
                msg += "\n"
            try:
                now = time.perf_counter()
                gap = now - self._last_send
                if gap < self.min_interval_s:
                    time.sleep(self.min_interval_s - gap)
                self.ser.write(msg.encode("utf-8"))
                self._last_send = time.perf_counter()

                # 필요하면 flush: self.ser.flush()
                print(f"[UART] sent: {msg.strip()}")
            except Exception as e:
                print(f"[UART] write error: {e}")
                try:
                    self.ser.close()
                except:
                    pass
                self.ser = None  # 재연결 루프로
        # 종료 시 포트 정리
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("[UART] closed")
        except:
            pass

# 라이다와 카메라 데이터를 받아 로봇 주변 점, 군집, 의자·미용사 각도 등을 화면에 실시간으로 시각화
class LidarView(QWidget):
    """gui_queue에서 받은 payload(dict)를 시각화:
    - points: [(q, ang_deg, dist_mm)]
    - xy:     [(x_mm, y_mm)]  with ANGLE_OFFSET 적용
    - labels: DBSCAN 라벨(-1=노이즈)
    - clusters: 요약 리스트"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.payload = {"points": [], "xy": [], "labels": [], "clusters": []}
        self.max_dist_mm = MAX_DIST_MM
        self.setMinimumSize(720, 540)

    def set_points(self, payload):
        """GUI 큐에서 받은 최신 페이로드를 저장하고 update()로 다시 그리기 트리거."""
        self.payload = payload or {"points": [], "xy": [], "labels": [], "clusters": []}
        self.update()

    def paintEvent(self, e):
        """배경·그리드·축·로봇점과 함께 점/군집/의자 하이라이트·카메라 각도 부채꼴 등을 화면 좌표로 변환해 실시간 렌더링."""
        if not self.isVisible():
            return

        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        pad = 20
        r_pix = min(cx, cy) - pad
        if r_pix <= 0:
            return

        scale = r_pix / float(self.max_dist_mm)

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        try:
            # 배경
            p.fillRect(self.rect(), QColor(18, 18, 18))

            # 그리드
            p.setPen(QPen(QColor(70, 70, 70), 1))
            for m in (1000, 2000, 3000):
                rr = m * scale
                p.drawEllipse(cx - rr, cy - rr, rr * 2, rr * 2)

            # 축선
            p.setPen(QPen(QColor(60, 60, 60), 1))
            p.drawLine(cx, cy - r_pix, cx, cy + r_pix)
            p.drawLine(cx - r_pix, cy, cx + r_pix, cy)

            # +Y(정면) 표시
            p.setPen(QPen(QColor(120, 120, 120), 1))
            p.drawLine(cx, cy - r_pix, cx, cy - r_pix + 14)
            p.setPen(QPen(QColor(200, 200, 200), 1))
            p.setFont(QFont("", 10))
            p.drawText(cx + 6, cy - r_pix + 16, "Front (+Y)")

            # 로봇 중심점
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(120, 180, 255))
            p.drawEllipse(cx - 4, cy - 4, 8, 8)

            # --- 점 그리기 ---
            xy = self.payload.get("xy", [])
            labels = self.payload.get("labels", [])
            clusters = self.payload.get("clusters", [])
            chair_ids = set(self.payload.get("chair_cluster_ids", []))

            # [수정] 추적된 미용사 ID를 미리 가져옴
            beautician_cid_payload = self.payload.get("beautician_cluster_id", None)            
            # --- [신규] 벽(Wall)으로 판별된 클러스터 ID 세트 생성 ---
            wall_ids = set(c['id'] for c in clusters if c.get('is_wall', False))
            # --- [신규 끝] ---
            
            if xy:
                for i, (x_mm, y_mm) in enumerate(xy):
                    # px = cx - x_mm * scale # [원본]
                    px = cx + x_mm * scale # [수정] Y축 대칭 추가
                    py = cy - y_mm * scale # (이전 수정 유지)
                    lab = labels[i] if i < len(labels) else -1

                    if lab == -1:
                        # 노이즈: 흰색
                        p.setPen(QPen(QColor(255, 255, 255), 2))
                    
                    # --- [신규] 벽 클러스터 우선 확인 (보라색) ---
                    elif lab in wall_ids:
                        # ★ 벽 군집: 보라색
                        p.setPen(QPen(QColor(180, 100, 255), 2))
                    # --- [신규 끝] ---
                        
                    elif lab in chair_ids:
                        # ★ 의자 군집: 빨간색(굵게)
                        p.setPen(QPen(QColor(255, 80, 80), 3))
                        
                    elif lab == beautician_cid_payload:
                        # ★ 미용사 군집: 초록색(굵게)
                        p.setPen(QPen(QColor(0, 255, 0), 3))
                    
                    else:
                        # 일반 군집: 주황색
                        p.setPen(QPen(QColor(255, 160, 0), 2))

                    p.drawPoint(int(px), int(py))
            
            # --- [수정] 카메라 각도 시각화 (로직은 유지하되, 데이터가 없으면 안 그림) ---
            beautician_angle = self.payload.get("beautician_angle_deg", None)
            beautician_tol = self.payload.get("beautician_tol_deg", 3.0)
            # selected_cid = self.payload.get("beautician_cluster_id", None) # [수정] 위로 이동함
            
            if beautician_angle is not None:
                a0 = robot_angle_to_screen_rad(beautician_angle)
                # x2 = cx - r_pix * math.sin(a0) # [원본]
                x2 = cx + r_pix * math.sin(a0) # [수정] Y축 대칭 추가
                y2 = cy - r_pix * math.cos(a0) # (이전 수정 유지)
                p.setPen(QPen(QColor(0, 220, 120), 2))
                p.drawLine(cx, cy, int(x2), int(y2))

            # (기존 의자 코드)
            chair_ids = self.payload.get("chair_cluster_ids", [])
            selected_cid = self.payload.get("selected_chair_id", None)
            locked_center = self.payload.get("locked_center", None) # <-- 의자 데이터
            locked_alive = bool(self.payload.get("locked_alive", False)) # <-- 의자 데이터
            selected_mid_idx = self.payload.get("selected_mid_idx", None)
            
            # (이 코드는 '현재 프레임'에 잡힌 군집의 중간점을 그리는 코드입니다)
            if (selected_mid_idx is not None) and xy and (0 <= selected_mid_idx < len(xy)):
                x_mm, y_mm = xy[selected_mid_idx]
                # px = cx - x_mm * scale # [원본]
                px = cx + x_mm * scale # [수정] Y축 대칭 추가
                py = cy - y_mm * scale # (이전 수정 유지)
                p.setPen(QPen(QColor(255, 0, 0), 4))
                p.drawPoint(int(px), int(py))

            if locked_center and locked_alive:
                bx, by = locked_center
                # px = cx - bx * scale # [원본]
                px = cx + bx * scale # [수정] Y축 대칭 추가
                py = cy - by * scale # (이전 수정 유지)
                p.setPen(QPen(QColor(255, 0, 0), 3))  
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(int(px) - 9, int(py) - 9, 18, 18)
            # --- [신규 끝] ---
            
            # --- [신규] 미용사 '현재 프레임' 군집 중심 그리기 (초록색 점) ---
            # (selected_cid는 payload에서 'beautician_cluster_id'로 이미 가져옴)
            
            beautician_mid_idx = None
            if beautician_cid_payload is not None:
                # clusters 리스트에서 ID가 일치하는 군집을 찾음
                for c in clusters:
                    if c.get("id") == beautician_cid_payload:
                        beautician_mid_idx = c.get("mid_idx")
                        break

            # (의자의 점 그리기 로직과 동일하게 굵은 점으로 그림)
            if (beautician_mid_idx is not None) and xy and (0 <= beautician_mid_idx < len(xy)):
                x_mm, y_mm = xy[beautician_mid_idx]
                # px = cx - x_mm * scale # [원본]
                px = cx + x_mm * scale # [수정] Y축 대칭 추가
                py = cy - y_mm * scale # (이전 수정 유지)
                p.setPen(QPen(QColor(0, 255, 0), 4)) # Green dot
                p.drawPoint(int(px), int(py))
                    
            # --- [기존] 미용사 추적 중심 그리기 (초록색 링) ---
            b_locked_center = self.payload.get("beautician_locked_center", None)
            b_locked_alive = bool(self.payload.get("beautician_locked_alive", False))

            if b_locked_center and b_locked_alive:
                bx, by = b_locked_center
                # px = cx - bx * scale # [원본]
                px = cx + bx * scale # [수정] Y축 대칭 추가
                py = cy - by * scale # (이전 수정 유지)
                # (진한 초록색 링)
                p.setPen(QPen(QColor(0, 255, 0), 3))  
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(int(px) - 9, int(py) - 9, 18, 18)
                    # --- [신규 끝] ---

        finally:
            p.end()
# ... (이전 클래스 동일) ...

# 라이다 시각화 창과 CLEAN·TRACK·STOP 버튼을 포함해 GUI 갱신과 시퀀스 실행을 관리
class MainWindow(QMainWindow):
    def __init__(self, gui_queue, q_tx, motor_stop_event, clean_controller, track_controller, parent=None): # [수정] 인자 추가
        super().__init__(parent)
        self.setWindowTitle("LiDAR Debug UI (DBSCAN Clustering)")
        self.view = LidarView(self)
        self.setCentralWidget(self.view)
        
        self.gui_queue = gui_queue
        self.q_tx = q_tx                  # UART 큐 저장
        self.motor_stop_event = motor_stop_event # 모터 중지 이벤트 저장
            
        # --- 버튼 생성 ---
        self.clean_button = QPushButton('CLEAN', self)
        self.clean_button.clicked.connect(self.on_clean_clicked)

        self.track_button = QPushButton('TRACK', self)
        self.track_button.clicked.connect(self.on_track_clicked)
        
        # STOP 버튼 생성
        self.stop_button = QPushButton('STOP', self)
        # 눈에 띄게 빨간색으로 스타일링
        self.stop_button.setStyleSheet("background-color: #A00000; color: white; font-weight: bold;")
        self.stop_button.clicked.connect(self.on_stop_clicked)
        
        # --- [CLEANER 버튼] ---
        self.cleaner_button = QPushButton('CLEANER', self)
        self.cleaner_button.clicked.connect(self.on_cleaner_clicked)
        # --- [추가 끝] ---

        self.timer = QTimer(self)
        self.timer.setInterval(int(GUI_PERIOD * 1000))
        self.timer.timeout.connect(self.on_gui_tick)
        self.timer.start()
        
        # [수정] 컨트롤러를 주입받아 사용
        self.clean = clean_controller
        self.track = track_controller

    def on_gui_tick(self):
        """gui_queue에서 최신 페이로드만 뽑아 화면에 반영(리렌더)한다."""
        latest = None
        while True:
            try:
                latest = self.gui_queue.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            self.view.set_points(latest)

    # --- 버튼 위치 조정을 위한 resizeEvent ---
    def resizeEvent(self, event):
        """창 크기 변경 시 우측 상단에 버튼 3개를 고정 배치로 재배치한다."""
        button_width = 100
        button_height = 40
        margin = 10
        spacing = margin // 2 # 버튼 사이 간격
        
        # CLEAN 버튼 (우측 상단)
        self.clean_button.setGeometry(
            self.width() - button_width - margin,  # x
            margin,                                # y
            button_width,                          # width
            button_height                          # height
        )
        
        # TRACK 버튼 (CLEAN 버튼 바로 아래)
        self.track_button.setGeometry(
            self.width() - button_width - margin,  # x
            margin + button_height + spacing,      # y
            button_width,                          # width
            button_height                          # height
        )
        
        # STOP 버튼 (TRACK 버튼 바로 아래)
        self.stop_button.setGeometry(
            self.width() - button_width - margin,      # x
            margin + (button_height + spacing) * 2,    # y
            button_width,                              # width
            button_height                              # height
        )
        
        # --- [CLEANER 버튼 위치 추가] ---
        self.cleaner_button.setGeometry(
            self.width() - button_width - margin,      # x
            margin + (button_height + spacing) * 3,  # y (STOP 버튼 아래)
            button_width,                              # width
            button_height                              # height
        )
        
        # 부모 클래스의 resizeEvent 호출
        super().resizeEvent(event)

    # --- CLEAN 버튼 클릭 핸들러 ---
    def on_clean_clicked(self):
        """STOP 플래그를 해제하고 CleanController의 시퀀스를 별도 스레드로 시작한다."""
        print("CLEAN 버튼 클릭: 시퀀스 요청")
        # 새 동작 시작 전, 중지 플래그를 해제
        self.motor_stop_event.clear()
        self.clean.start_sequence() # 스레드를 시작하는 함수 호출

    # --- TRACK 버튼 클릭 핸들러 ---
    def on_track_clicked(self):
        """STOP 플래그를 해제하고 TrackController의 시퀀스를 별도 스레드로 시작한다."""
        print("TRACK 버튼 클릭: 시퀀스 요청") # [MODIFY]
        # 새 동작 시작 전, 중지 플래그를 해제
        self.motor_stop_event.clear()
        self.track.start_sequence() # [MODIFY] run_forward_burst() 대신 start_sequence() 호출

    # ################# [수정된#메서드#시작] #################
    # --- STOP 버튼 클릭 핸들러 ---
    def on_stop_clicked(self):
        """[수정] CLEAN/TRACK 동작 중에만 중지 신호를 보내고, 모드에 맞는 즉시 명령을 보낸 후, 1초 뒤 '복귀' 시퀀스를 시작한다."""
        print("STOP 버튼 클릭: 중지 요청")
        
        is_clean_active = (self.clean._th and self.clean._th.is_alive())
        is_track_active = (self.track._th and self.track._th.is_alive())

        if is_clean_active or is_track_active:
            
            # 1. 실행 중인 CLEAN/TRACK 스레드에 중지 신호 전송
            self.motor_stop_event.set()
            
            # 2. [수정] 모드에 따라 즉시 전송할 명령 결정
            if is_clean_active:
                # [신규] clean_mode 내부 상태 확인
                if self.clean.is_cleaner_on():
                    print("[STOP] CLEAN 활성 (청소기 ON). 즉시 CLEANER(1,0,1,0,1) 전송.")
                    immediate_cmd = CMD_CLEANER # "1,0,1,0,1"
                else:
                    print("[STOP] CLEAN 활성 (청소기 OFF). 즉시 STOP(1,0,1,0,0) 전송.")
                    immediate_cmd = CMD_STOP    # "1,0,1,0,0"
            
            else: # is_track_active
                print("[STOP] TRACK 활성. 즉시 STOP(1,0,1,0,0) 전송.")
                immediate_cmd = CMD_STOP    # "1,0,1,0,0"
            
            q_put_latest(self.q_tx, immediate_cmd) # 즉시 전송
            
            # 3. 1초 뒤 '복귀 시퀀스' 시작 (이 부분은 이전과 동일)
            print("[STOP] (1초) > '복귀 시퀀스' 시작 예약.")
            # self.track이 TrackController 인스턴스입니다.
            QTimer.singleShot(1000, self.track.start_return_sequence)
        
        else:
            print("[STOP] 무시 (CLEAN/TRACK 동작 중 아님)")
    # ################# [수정된#메서드#종료] #################

    # --- [CLEANER 버튼 클릭 핸들러 추가] ---
    def on_cleaner_clicked(self):
        """CLEANER 버튼 클릭 시 '1,0,1,0,1' 명령을 UART 큐로 전송한다."""
        cmd = CMD_CLEANER # [수정]
        print(f"CLEANER 버튼 클릭: {cmd} 전송")
        
        # 참고: 이 버튼은 motor_stop_event를 제어하지 않고 
        # 단일 명령만 전송합니다.
        
        # UART 큐에 명령 전송
        q_put_latest(self.q_tx, cmd)



camera_latest = CameraData()
target_state = ChairState()
chair_tracker = ChairTracker()
beautician_state = BeauticianState()
beautician_tracker = BeauticianTracker()

def main():
    stop_event = threading.Event()
    motor_stop_event = threading.Event()

    # --- 1. 큐 생성 ---
    lidar_raw_queue = queue.Queue(maxsize=3)     # (1) Lidar Collecting -> Lidar Processing
    dbs_result_queue = queue.Queue(maxsize=3)      # (2) Lidar Processing -> Find Chair
    final_cluster_queue = queue.Queue(maxsize=3)  # (3) Find Chair -> Find Beautician (Fusion)
    fusion_result_queue = queue.Queue(maxsize=3)  # (4) Find Beautician -> Chair Tracking
    
    # Tracking -> GUI Packaging 큐 추가
    tracking_result_queue = queue.Queue(maxsize=3) # (5) Chair Tracking -> GUI Packaging
    
    gui_queue = queue.Queue(maxsize=3)             # (6) GUI Packaging -> GUI
    q_tx = queue.Queue(maxsize=10)                 # (모터)

    # --- 2. 스레드 생성 (최신 함수 이름 사용) ---
    
    # (1) Lidar 수집 워커
    lidar_thread = threading.Thread(
        target=lidar_collecting, # <- 함수 이름 변경됨
        name="lidar_collecting", 
        args=(stop_event, lidar_raw_queue), 
        daemon=True
    )

    # (2) Lidar 처리 워커 (필터링, 변환, DBSCAN)
    proc_thread = threading.Thread(
        target=lidar_processing, # <- 함수 이름 변경됨
        name="lidar_processing", 
        args=(stop_event, lidar_raw_queue, dbs_result_queue), 
        daemon=True
    )

    # (3) 의자 탐색 워커 (PCA, 원피팅, 단일 의자 선택)
    chair_thread = threading.Thread(
        target=find_chair, # <- 함수 이름 변경됨
        name="find_chair", 
        args=(stop_event, dbs_result_queue, final_cluster_queue, beautician_tracker), # [수정] beautician_tracker 추가
        daemon=True
    )
    
    # (8) UART 전송 워커
    t_uart = UartTxWorker(stop_event, STM_PORT_NAME, 115200, q_tx)
    
    # --- [수정] 2-bis. 컨트롤러 생성 ---
    # MainWindow와 camera_collecting 스레드가 공유할 컨트롤러 인스턴스
    clean_controller = CleanController(q_tx, motor_stop_event)
    track_controller = TrackController(q_tx, motor_stop_event)
    # --- [수정 끝] ---

    # (4) 카메라 수집 워커
    t_camera = threading.Thread(
        target=camera_collecting, # <- 함수 이름 변경됨
        name="camera_collecting", 
        # [수정] track_controller와 q_tx 추가
        args=(stop_event, HOST, PORT, clean_controller, track_controller, q_tx, motor_stop_event), 
        daemon=True
    )
    
    # (5) 카메라 융합 워커 (미용사 탐색)
    t_fusion = threading.Thread(
        target=find_beautician, # <- 함수 이름 변경됨
        name="find_beautician", 
        # [수정] chair_tracker 인자 추가
        args=(stop_event, final_cluster_queue, fusion_result_queue, track_controller, chair_tracker), 
        daemon=True
    )
    
    # (6) 의자 추적 워커 (상태 업데이트)
    t_tracking = threading.Thread(
        target=track_chair, 
        name="chair_tracking", 
        # 출력 큐 변경: gui_queue -> tracking_result_queue
        args=(stop_event, fusion_result_queue, tracking_result_queue), 
        daemon=True
    )
    
    # (7) GUI 패키징 워커
    t_gui_packer = threading.Thread(
        target=gui_packaging, 
        name="gui_packaging", 
        args=(stop_event, tracking_result_queue, gui_queue), # 입력: tracking_result_queue, 출력: gui_queue
        daemon=True
    )
        
    # --- 3. 스레드 시작 ---
    lidar_thread.start()
    proc_thread.start()
    chair_thread.start()
    t_camera.start()
    t_fusion.start()
    t_tracking.start()
    t_gui_packer.start()
    t_uart.start()
    
    # --- 4. PyQt GUI 실행 ----
    app = QApplication(sys.argv)
    # MainWindow는 이제 gui_packaging_worker의 출력을 받는 gui_queue를 구독합니다.
    # [수정] 생성한 컨트롤러 인스턴스 주입
    win = MainWindow(gui_queue, q_tx, motor_stop_event, clean_controller, track_controller) 
    win.show()

    # --- 5. 종료 핸들러 ---
    def on_quit():
        print("[MAIN] Qt 종료 신호 → 스레드 종료 요청")
        motor_stop_event.set()
        stop_event.set()
        
        t_uart.join(timeout=5.0) 
        lidar_thread.join(timeout=1.0)
        proc_thread.join(timeout=1.0)
        chair_thread.join(timeout=1.0)
        t_camera.join(timeout=1.0)
        t_fusion.join(timeout=1.0)
        t_tracking.join(timeout=1.0)
        t_gui_packer.join(timeout=1.0)
        
        print("[MAIN] 종료 완료")

    app.aboutToQuit.connect(on_quit)
    
    # (Ctrl+C 핸들러 등은 동일)
    def sigint_handler(signum, frame):
        print("\n[MAIN] Ctrl+C 감지 → 앱 종료 요청")
        QTimer.singleShot(0, app.quit)

    signal.signal(signal.SIGINT, sigint_handler)
    
    timer = QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None) 
    
    print("[MAIN] GUI 시작")
    app.exec()
    
    if not stop_event.is_set():
        print("[MAIN] GUI 비정상 종료. 스레드 정리 시작...")
        on_quit()

if __name__ == "__main__":
    main()