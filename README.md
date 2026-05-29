# piper_ui_ws

Piper 로봇팔을 스마트폰/브라우저에서 제어할 수 있는 모바일 웹 UI 워크스페이스입니다.

---

## 패키지 구성

| 패키지 | 레포 | 설명 |
|--------|------|------|
| `piper_web_ui` | 본 레포 | 모바일 웹 UI 노드 |
| `piper` (ctrl_node) | [radius-piper](https://github.com/WeGo-Robotics-dev/radius-piper) | Piper 하드웨어 제어 노드 |
| `wego_msgs` | [radius-wego-msgs](https://github.com/WeGo-Robotics-dev/radius-wego-msgs) | 공용 메시지/서비스/액션 타입 |

---

## 주요 기능

- **카메라 스트리밍**: ROS2 카메라 토픽 실시간 영상 표시
- **TCP 위치 제어**: X / Y / Z 축 이동 (버튼 누르는 시간에 비례)
- **TCP 자세 제어**: Rx / Ry / Rz 관절 제어 (joint 4 / 5 / 6)
- **그리퍼 제어**: 열기 / 닫기
- **속도 조절**: 슬라이더로 이동 속도 조절
- **E-STOP**: 즉시 정지 (잔류 토크 유지, 낙하 방지)
- **복구**: E-STOP 이후 정상 상태 복구 (약 6초 소요)
- **자세 저장/실행**: 현재 joint 각도를 이름과 함께 YAML에 저장, 저장된 자세로 이동
  - 측정 자세 (`measurement`) — 자유 이름 지정
  - Stow 자세 (`stow`) — `default` / `low` / `back_swept` 고정

---

## 환경 요구사항

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

### Python 패키지

```bash
pip install nicegui scipy
```

---

## 설치

### 1. 워크스페이스 생성

```bash
mkdir -p ~/piper_ui_ws/src
cd ~/piper_ui_ws/src
```

### 2. 패키지 clone

```bash
# 본 레포
git clone https://github.com/WeGo-Robotics-Daegu/piper_ui_ws.git
cd piper_ui_ws

# 의존 패키지
git clone https://github.com/WeGo-Robotics-dev/radius-wego-msgs.git
git clone -b min https://github.com/WeGo-Robotics-dev/radius-piper.git
```

### 3. 빌드

```bash
cd ~/piper_ui_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 실행 방법

### 1. CAN 활성화

```bash
cd ~/piper_ui_ws
bash can_activate.sh
```

### 2. Piper ctrl_node 실행

```bash
ros2 launch piper start_single_piper.launch.py
```

### 3. Web UI 노드 실행

```bash
ros2 run piper_web_ui web_ui_node
```

### 4. 브라우저 접속

```
http://<로봇_IP>:8080
```

---

## 포트 정보

| 포트 | 용도 |
|------|------|
| 8080 | 메인 Web UI |
| 8081 | 카메라 MJPEG 스트림 |

---

## 사용 순서

1. 브라우저 접속 후 **Enable** 버튼으로 로봇 활성화
2. **초기 위치** 버튼으로 초기 자세 이동
3. 화살표 버튼으로 로봇 제어 (짧게 누르면 조금, 길게 누르면 계속 이동)
4. 비상 시 **E-STOP** 버튼 → 재사용 시 **복구** 버튼

---

## 자세 저장 사용

### 측정 자세
1. 원하는 자세로 팔 이동
2. **자세 저장** 섹션 → **측정 자세** 탭 선택
3. 이름 입력 후 **현재 자세 저장**
4. 저장된 자세 옆 **실행** 버튼으로 해당 자세로 이동

저장 위치: `~/piper_poses/measurement_poses.yaml`

### Stow 자세
1. 원하는 stow 자세로 팔 이동
2. **자세 저장** 섹션 → **Stow 자세** 탭 선택
3. `default` / `low` / `back_swept` 중 선택 후 **현재 자세 저장**
4. 저장된 자세 옆 **실행** 버튼으로 stow 동작 실행

저장 위치: `~/piper_poses/stow_poses.yaml`

> **주의**: `stow_poses.yaml`에 `default` 항목이 없으면 ctrl_node가 시작되지 않습니다.

---

## ROS2 인터페이스

상세 내용은 [PIPER_INTERFACES.md](PIPER_INTERFACES.md) 참조.
