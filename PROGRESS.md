# Piper Web UI 프로젝트 진행 기록

> 기준: 2026-05-18 (프로젝트 목표 변경 이후)

---

## 1. 프로젝트 목표 변경

### 기존 목표 (폐기)
- 드래그 티칭 모드 소프트웨어 구현
- `piper_teaching_ctrl_node.py` + `start_teaching_piper.launch.py` 신규 제작
- `grag_teach_ctrl=0x01` CAN 명령으로 드래그 티칭 모드 진입 시도
- **결과**: HW 버튼으로는 `ctrl_mode=0x02(Teaching_mode)` 진입 확인됐으나, 소프트웨어 CAN 명령으로는 진입 불가 → 제조사 문의 필요

### 새로운 목표
- **웹 UI**에서 로봇팔 EE(End Effector) X/Y/Z/Rx/Ry/Rz 조작
- 모바일 브라우저에서 접속 가능
- 카메라 실시간 영상 표시
- 포즈 저장 및 시퀀스 실행

---

## 2. 시스템 아키텍처

```
모바일 브라우저
    ↕ WebSocket / HTTP
NiceGUI + FastAPI 서버 (PC)
    ↕ ROS2 topics
piper_single_ctrl_node
    ↕ Piper SDK (CAN)
Piper 로봇팔
```

### 네트워크 조건
- PC와 모바일이 **동일 WiFi** 네트워크에 있어야 함
- 접속 주소: `http://{PC_IP}:{PORT}`

---

## 3. 기술 스택 결정

| 항목 | 결정 내용 |
|---|---|
| UI 프레임워크 | NiceGUI + FastAPI |
| ROS2 버전 | Humble |
| 카메라 | RealSense D435 (`/image_raw/compressed` 토픽) |
| 포즈 저장 형식 | JSON 파일 |
| EE 스텝 크기 | 1 / 5 / 10 mm·° 선택 가능 (기본 5mm / 5°) |
| launch 파일 | `start_single_piper.launch.py` (기존) |

---

## 4. UI 기능 요구사항

### 4-1. EE 조작
- X / Y / Z / Rx / Ry / Rz 각각 +/- 버튼
- 스텝 크기 선택: 1 / 5 / 10 mm·°

### 4-2. 카메라
- RealSense D435 실시간 영상
- `/image_raw/compressed` 토픽 subscribe

### 4-3. 포즈 저장
- 현재 EE 좌표 저장 (이름 지정)
- 저장 형식:
```json
{
  "poses": [
    {
      "id": 0,
      "name": "pick",
      "pose": {"x": 0.3, "y": 0.0, "z": 0.2, "rx": 0.0, "ry": 0.0, "rz": 0.0}
    }
  ]
}
```

### 4-4. 시퀀스 실행
- 저장된 포즈 목록 조회 / 삭제
- 저장된 포즈 순서대로 자동 실행

---

## 5. 워크스페이스 구조

### 결정 배경
- 기존 `piper_ws`에는 불필요한 패키지 다수 존재 (용량, 관리 문제)
- `piper_sdk`는 pip 전역 설치 패키지 (`piper-sdk==0.6.1`) → 복사 불필요
- `realsense2_camera`는 apt 전역 설치 → 복사 불필요 (버전 충돌 이슈로 제외)

### 최종 구조
```
piper_ui_ws/
    src/
        piper_msgs/       ← piper_ws에서 복사 (커스텀 메시지/서비스)
        piper/            ← piper_ws에서 필요한 파일만 복사
            scripts/
                piper_single_ctrl_node.py
            launch/
                start_single_piper.launch.py
            CMakeLists.txt  ← C++ 의존성 제거 후 정리
            package.xml     ← moveit 등 불필요 의존성 제거
        piper_web_ui/     ← 신규 생성 (NiceGUI + FastAPI)
    install/
    build/
    log/
```

---

## 6. 패키지 설정

### piper/CMakeLists.txt (정리 후)
- C++ 빌드 관련 전부 제거 (moveit, rclcpp 등)
- Python 스크립트 설치만 유지

### piper/package.xml (정리 후)
- `moveit`, `tf2_geometry_msgs` 등 불필요 의존성 제거
- `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `piper_msgs` 만 유지

### realsense-ros 제외 이유
- 클론한 버전이 librealsense2 **2.56.6** 요구
- 시스템 설치 버전은 **2.56.5** → 버전 불일치 빌드 에러
- 이미 apt로 설치된 `realsense2_camera` 패키지 사용으로 대체
- README에 apt 설치 명령어 명시 예정

---

## 7. 빌드 현황

```bash
cd ~/piper_ui_ws
colcon build --symlink-install
```

| 패키지 | 상태 |
|---|---|
| `piper_msgs` | ✅ 빌드 성공 |
| `piper` | ✅ 빌드 성공 |
| `piper_web_ui` | ✅ 빌드 성공 (아직 코드 없음) |

---

## 8. 다음 단계

- [ ] `start_single_piper.launch.py` 실행 확인 (piper_ui_ws 기준)
- [ ] `piper_web_ui` 코드 작성 (NiceGUI + FastAPI)
- [ ] 카메라 토픽 스트리밍 구현
- [ ] EE 조작 버튼 구현
- [ ] 포즈 저장 / 시퀀스 실행 구현
- [ ] README 작성

---

## 9. 실행 방법 (예정)

```bash
# 워크스페이스 source
source ~/piper_ui_ws/install/setup.bash

# 카메라 실행 (별도 터미널)
ros2 launch realsense2_camera rs_launch.py

# 로봇 + 웹 UI 실행
ros2 launch piper start_single_piper.launch.py
ros2 run piper_web_ui web_ui_node.py
```
