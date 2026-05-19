# piper_web_ui

Piper 로봇팔을 스마트폰/브라우저에서 제어할 수 있는 모바일 웹 UI 패키지입니다.

---

## 주요 기능

- **카메라 스트리밍**: ROS2 카메라 토픽 실시간 영상 표시
- **TCP 위치 제어**: X / Y / Z 축 이동 (버튼 누르는 시간에 비례)
- **TCP 자세 제어**: Rx / Ry / Rz 관절 제어 (joint 4 / 5 / 6)
- **그리퍼 제어**: 열기 / 닫기
- **속도 조절**: 슬라이더로 이동 속도 조절 (1~10단계)
- **E-STOP**: 즉시 정지 (잔류 토크 유지, 낙하 방지)
- **복구**: E-STOP 이후 정상 상태 복구 (약 6초 소요)
- **웨이포인트**: 위치 저장 및 순서 실행

---

## 환경 요구사항

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

### Python 패키지

```bash
pip install nicegui scipy
```

### ROS2 패키지

- `piper_msgs` — Piper 전용 메시지 타입
- `piper` (ctrl_node) — Piper 하드웨어 제어 노드
- 카메라 노드 — `/camera/camera/color/image_raw/compressed` 토픽을 퍼블리시하는 카메라 드라이버 필요

---

## 빌드

의존성 패키지 일괄 설치:

```bash
cd ~/piper_ui_ws
rosdep install --from-paths src --ignore-src -r -y
```

빌드:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 실행 방법

### 1. 카메라 노드 실행 (선택 사항)

카메라가 없어도 로봇 제어는 정상적으로 사용 가능합니다.

카메라를 사용할 경우 카메라 노드를 실행합니다. 현재 설정된 토픽명은 다음과 같으며, 사용하는 카메라에 따라 `src/piper_web_ui/piper_web_ui/web_ui_node.py` 에서 아래 부분을 수정하여 변경할 수 있습니다:

```python
self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self._on_camera, 1)
```

### 2. Piper ctrl_node 실행

```bash
ros2 launch piper ctrl_node.launch.py
```

### 3. Web UI 노드 실행

```bash
ros2 run piper_web_ui web_ui_node
```

### 4. 브라우저 접속

스마트폰 또는 PC 브라우저에서 접속:

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

## 웨이포인트 사용

1. 원하는 위치로 로봇 이동
2. **현재 위치 저장** 버튼으로 웨이포인트 추가
3. 대기 시간 설정 후 **순서 실행** 버튼으로 자동 이동

웨이포인트는 `~/piper_ui_ws/waypoints.json`에 저장됩니다.
