# -*- coding: utf-8 -*-

# --- 0. 경고 메시지 무시 ---
# YOLOv5가 CPU 환경에서도 torch.cuda.amp.autocast 관련 경고를 
# 계속 출력하는 것을 무시하도록 설정합니다.
import warnings
warnings.filterwarnings("ignore", message=".*torch.cuda.amp.autocast.*")

# --- 1. 라이브러리 임포트 ---
import torch  # 파이토치(PyTorch) 라이브러리, YOLOv5 모델을 사용하기 위해 필요
import cv2    # OpenCV 라이브러리, 이미지 처리 및 화면 표시를 위해 필요
import numpy as np  # Numpy 라이브러리, 배열 계산(특히 HSV 색상 범위)을 위해 필요
from picamera2 import Picamera2  # 라즈베리파이 카메라 제어를 위한 Picamera2 라이브러리
import socket, threading, queue, time

class CameraSender(threading.Thread):
    def __init__(self, host, port, max_q=1):
        super().__init__(daemon=True)
        self.host, self.port = host, port
        self.q = queue.Queue(maxsize=max_q)  # 최신만 유지
        self.stop = threading.Event()

    def run(self):
        sock = None
        backoff = 0.5
        while not self.stop.is_set():
            # 1) 연결 확보
            if sock is None:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # 지연 최소화
                    s.settimeout(3.0)
                    s.connect((self.host, self.port))
                    s.settimeout(1.0)  # send 타임아웃
                    sock = s
                    backoff = 0.5
                    # print("[TX] connected")
                except Exception:
                    # 실패 → 백오프 후 재시도
                    time.sleep(backoff)
                    backoff = min(backoff * 2, 5.0)
                    continue

            # 2) 전송
            try:
                msg = self.q.get(timeout=0.2)  # 없으면 잠깐 대기
            except queue.Empty:
                continue

            try:
                sock.sendall(msg.encode("utf-8"))
            except Exception:
                # 소켓 문제 → 닫고 재연결 루프로
                try:
                    sock.close()
                except:
                    pass
                sock = None

        # 종료 정리
        try:
            if sock:
                sock.close()
        except:
            pass

    def send_latest(self, message: str):
        # 최신만 유지
        if not message.endswith("\n"):
            message += "\n"
        try:
            if self.q.full():
                _ = self.q.get_nowait()
            self.q.put_nowait(message)
        except queue.Full:
            pass  # 드물게 경합 시 무시

    def shutdown(self):
        self.stop.set()

# --- 2. YOLOv5 모델 로드 ---
# 울트라리티קס(Ultralytics) 저장소에서 미리 학습된 'yolov5n' 모델을 로드합니다. (n은 nano로 가장 가벼운 모델)
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
model.to('cpu')     # 모델을 CPU에서 실행하도록 설정 (라즈베리파이에는 GPU가 없으므로)
model.eval()        # 모델을 추론(evaluation) 모드로 설정 (학습 모드가 아님을 명시)

# --- [추가] 2-1. 서버 접속 정보 ---
HOST = '192.168.137.20'  # 서버 IP 주소
PORT = 9999              # 서버 포트 번호

# --- 3. 카메라 설정 ---
picam2 = Picamera2()  # Picamera2 객체 생성

# 카메라 설정을 정의합니다.
# main={"size": (320, 320)}: 카메라 해상도를 320x320으로 설정.
# "format": "RGB888": 카메라 이미지의 색상 순서를 RGB로 설정.
config = picam2.create_preview_configuration(main={"size": (320, 320), "format": "RGB888"})
picam2.configure(config) # 카메라에 설정을 적용
picam2.start()           # 카메라 작동 시작

# 화면에 표시될 창의 이름을 설정하고 생성합니다.
cv2.namedWindow("Salon Assistant Vision", cv2.WINDOW_NORMAL)

# 미용사 식별을 위한 연두색의 HSV 색상 범위를 정의합니다.
# H(색상), S(채도), V(명도) 순서. 이 값은 조명 환경에 따라 조절이 필요할 수 있습니다.
lower_green = np.array([35, 80, 80])
upper_green = np.array([85, 255, 255])

# ★ 1. 목표 FPS 설정
TARGET_FPS = 2.0  # 1초에 2번 (0.5초 간격)
TARGET_INTERVAL = 1.0 / TARGET_FPS  # 목표 간격 (0.5초)

# ★ 2. 실제 FPS 표시 및 시간 계산을 위한 변수
loop_start_time = time.time() # 루프 시작 시간 (초기화)
actual_fps = 0.0              # 화면에 표시할 실제 FPS 값

sender = CameraSender(HOST, PORT)
sender.start()

# --- 4. 메인 루프 ---
try:
    # 프로그램이 종료될 때까지 무한 반복
    while True:
        # ★ 3. 이번 프레임 '처리' 시작 시간 기록
        processing_start_time = time.time()
        
        # 4-1. 프레임 캡처
        # 카메라에서 현재 프레임을 NumPy 배열 형태로 가져옵니다. (설정에 따라 RGB 순서)
        frame = picam2.capture_array()
        
        # 4-3. 모델 추론
        # YOLOv5 모델에 원본(RGB) 프레임을 입력하여 객체 탐지를 수행합니다.
        results = model(frame)
        
        # 탐지된 객체들의 정보를 추출합니다.
        # detections는 [x1, y1, x2, y2, confidence, class_id] 리스트를 담고 있습니다.
        detections = results.xyxy[0]
        class_names = results.names  # 클래스 이름 리스트 (예: 'person', 'car' 등)

        # 이번 프레임에서 찾은 미용사 정보를 저장할 변수를 초기화합니다.
        beautician_info = None      # 미용사 정보
        max_color_pixels = 0        # 사람 중 최대 색상 픽셀 수

        # 4-4. 모든 탐지된 객체 분석
        # 탐지된 모든 객체에 대해 반복 처리
        for *xyxy, conf, cls in detections:
            class_name = class_names[int(cls)] # 탐지된 객체의 클래스 이름 확인
            
            # 탐지된 객체가 '사람'일 경우에만 처리
            if class_name == 'person' and conf > 0.4:
                # 해당 사람의 바운딩 박스 좌표를 가져옴
                x1, y1, x2, y2 = map(int, xyxy)
                # (수정) 원본(RGB) 프레임에서 사람 영역(ROI)을 잘라냄
                # (주의: y1:y2, x1:x2 순서)
                person_roi_rgb = frame[y1:y2, x1:x2]

                # (추가) 잘라낸 작은 ROI만 HSV로 변환
                # (예외 처리: ROI가 비어있으면(크기 0) 건너뜀)
                if person_roi_rgb.size == 0:
                    continue
                person_roi_hsv = cv2.cvtColor(person_roi_rgb, cv2.COLOR_RGB2HSV)
                
                # (기존) person_roi = hsv_frame[y1:y2, x1:x2] # <--- 이 줄을 위의 3줄로 대체

                # (수정) 변환된 작은 ROI(hsv)로 마스크 생성
                mask = cv2.inRange(person_roi_hsv, lower_green, upper_green)
                # 마스크에서 흰색 픽셀(연두색 부분)의 개수를 셈
                color_pixels = cv2.countNonZero(mask)
                
                # 현재까지 발견된 사람보다 연두색 픽셀이 많으면, '미용사' 정보 업데이트
                if color_pixels > max_color_pixels:
                    max_color_pixels = color_pixels
                    beautician_info = (xyxy, conf)

        # 4-5. 결과 처리 및 출력
        # '미용사'가 발견되었을 경우 (최소 100픽셀 이상의 색상이 감지될 때만)
        if beautician_info and max_color_pixels > 100:
            xyxy, conf = beautician_info
            x1, y1, x2, y2 = map(int, xyxy)

            # 중심 좌표 및 너비, 높이 계산
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            w = x2 - x1
            h = y2 - y1

            # 터미널에 요청된 형식으로 미용사 정보 출력
            # 형식: 객체이름,인식확률,중심x,중심y,너비,높이
            message = f"Beautician,{float(conf):.2f},{center_x},{center_y},{w},{h}\n"
            print(f"보낸 메시지: {message}", end="")  # 줄바꿈 이미 message에 포함
            sender.send_latest(message)
            


            # 화면에 미용사를 파란색 사각형으로 그림 (RGB 색상 코드 기준)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            label = f"Beautician {float(conf):.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # ★ 4. (새로운 위치) 실제 FPS를 화면에 표시
        # (이전 루프에서 계산된 actual_fps 값을 사용)
        cv2.putText(frame, f"FPS: {actual_fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 4-6. 최종 화면 표시
        # cv2.imshow는 BGR 이미지를 기대하므로, 화면에 표시하기 직전에 RGB를 BGR로 변환
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("Salon Assistant Vision", frame)
        
        # --- 종료 조건 확인 ---

        # OpenCV 창 업데이트를 위해 1ms 대기 (이 줄이 없으면 영상이 안 나옴)
        cv2.waitKey(1)
        
        # -----------------------------------------------
        # ★ 5. FPS 고정을 위한 시간 계산 및 sleep
        # -----------------------------------------------

        # (A) 이번 프레임 처리에 걸린 시간 계산
        processing_time = time.time() - processing_start_time

        # (B) 목표 시간(0.5초)에서 처리 시간을 뺀 '대기 시간' 계산
        wait_time = TARGET_INTERVAL - processing_time

        if wait_time > 0:
            # 처리 시간이 0.5초보다 빨랐으면, 남은 시간만큼 대기
            time.sleep(wait_time)

        # -----------------------------------------------
        # ★ 6. (다음 프레임에 표시할) 실제 FPS 계산
        # -----------------------------------------------

        # (sleep을 포함한) 이번 루프의 총 소요 시간 계산
        actual_loop_time = time.time() - loop_start_time 
        actual_fps = 1.0 / actual_loop_time # 실제 FPS
        loop_start_time = time.time()     # 다음 루프를 위해 시작 시간 갱신

        # 2. (추가) 창의 'X' 버튼을 눌렀는지(창이 닫혔는지) 확인
        try:
            # 창의 속성을 확인하여, '보이는 상태'가 아니면(값이 1 미만이면) 종료
            if cv2.getWindowProperty("Salon Assistant Vision", cv2.WND_PROP_VISIBLE) < 1:
                break
        except cv2.error:
            # 창이 너무 빨리 닫혀 속성 확인이 불가능할 때(오류 발생 시)에도 종료
            break

# --- 5. 종료 처리 ---
finally:
    # 프로그램 종료 시 카메라 리소스를 해제
    picam2.stop()
    # 모든 OpenCV 창을 닫음
    cv2.destroyAllWindows()
    sender.shutdown()
    try:
        sender.join(timeout=1.0)
    except:
        pass