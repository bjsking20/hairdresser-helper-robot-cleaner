# 🤖 hairdresser-helper-robot-cleaner

> **비전 AI와 라이다 센서 융합을 통한 미용실 맞춤형 자율주행 보조 로봇**

미용사(이동 객체)와 의자(고정 객체)를 실시간으로 구분하여, 시술 중에는 미용사를 추적하고, 작업 직후 의자 주변을 선회하며 청소하는 지능형 보조 로봇 시스템입니다.

## 🎥 Demo
청소모드: https://youtu.be/KfdacVqhzOo
추적모드: https://youtu.be/zWpixU9N7ag
전체영상: https://youtu.be/NC82-YxeQQA

## 🛠 Tech Stack
* **Hardware:** Raspberry Pi 4, STM32 Nucleo-F767ZI, RPLidar A1M8, Pi Camera Module 3
* **Language & Library:** Python 3, C/C++, PyQt6, scikit-learn
* **Algorithm:** DBSCAN Clustering, Taubin Circle Fitting, P-Control, Sensor Fusion

## 🏗 System Architecture
본 시스템은 I/O 병목 현상을 방지하기 위해 8개의 독립 워커(Worker) 기반 멀티스레드 비동기 파이프라인으로 설계되었습니다.

1. **Sensor Acquisition:** LiDAR 스캔 및 Vision 데이터 TCP 비동기 수신
2. **Data Processing:** DBSCAN 기반 포인트 클라우드 군집화 및 공간 필터링
3. **Target Tracking:** 기하학적 형상 분석(의자) 및 센서 퓨전(미용사)을 통한 객체 식별
4. **Autonomous Driving:** 상태 머신(Clean/Track) 기반 폐루프(Closed-Loop) 정밀 모터 제어
5. **Visualization:** PyQt6 기반 2D 실시간 데이터 렌더링 및 제어 대시보드

## ⚙️ Core Features
* **센서 퓨전 (Sensor Fusion):** 카메라 픽셀 좌표를 물리적 방위각으로 정규화하여 LiDAR 군집과 교차 검증 (오차 허용 범위 30도).
* **비선형 스무스 모터 제어 (Smooth Motor Control):** Tanh 함수 기반 속도 매핑 및 슬루 레이트(Slew Rate) 제한을 통한 오버슈트 방지 및 스틱션 극복.
* **무중단 통신 인프라:** STM32와 Raspberry Pi 간의 UART 통신 단절 시 1초 대기 후 즉각 재접속을 시도하는 자가 복구(Auto-Recovery) 로직 구현.

## 📁 Repository Structure
* `101_FINAL_CODE.py` : 데이터 수집, 알고리즘 가공, 하드웨어 제어 및 GUI가 통합된 메인 실행 파일.
