# Piper Web UI 패키지 구조 가이드

---

## 워크스페이스 구조

```
piper_ui_ws/
├── src/
│   ├── piper/                        # Piper 하드웨어 제어 패키지
│   │   └── scripts/
│   │       └── piper_single_ctrl_node.py   # 메인 제어 노드
│   ├── piper_msgs/                   # 커스텀 메시지 타입
│   └── piper_web_ui/                 # 웹 UI 패키지
│       └── piper_web_ui/
│           └── web_ui_node.py        # 메인 UI 노드
├── waypoints.json                    # 웨이포인트 저장 파일 (런타임 생성)
├── README.md
└── SETUP_GUIDE.md
```

---

## 패키지 구성

### piper_web_ui

NiceGUI 기반 모바일 웹 UI 노드.

- **포트 8080**: 메인 Web UI (NiceGUI + FastAPI)
- **포트 8081**: 카메라 MJPEG 스트림 (Python `http.server`, 별도 스레드)

> 카메라 스트림을 포트 8081로 분리한 이유: NiceGUI 내부의 `BaseHTTPMiddleware`가 MJPEG 무한 스트리밍과 구조적으로 호환되지 않아 포트를 분리하여 미들웨어를 우회함.

### piper (ctrl_node)

Piper SDK를 통해 실제 하드웨어와 통신하는 노드. CAN 버스로 로봇팔 제어.

---

## ROS2 토픽 구조

### web_ui_node → ctrl_node (Publish)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `pos_cmd` | PosCmd | EE 위치/자세 제어 |
| `enable_flag` | Bool | Enable / Disable |
| `init_pos` | Bool | 초기 위치 이동 |
| `estop` | Bool | E-STOP 트리거 |
| `restore` | Bool | 복구 시퀀스 트리거 |
| `joint_states` | JointState | 관절 직접 제어 |

### ctrl_node → web_ui_node (Subscribe)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `end_pose` | Pose | EE 현재 위치 (m, quaternion) |
| `joint_states_single` | JointState | 관절 현재 상태 |
| `/camera/camera/color/image_raw/compressed` | CompressedImage | 카메라 영상 |

---

## 단위 및 좌표계

| 항목 | 단위 | 비고 |
|------|------|------|
| x / y / z | mm | ctrl_node 내부에서 ×1000 처리 |
| roll / pitch / yaw | degrees | ctrl_node 내부에서 ×1000 처리 |
| joint 각도 | radians | web_ui_node에서 degrees → radians 변환 후 퍼블리시 |

---

## 관절 매핑 (TCP 자세 제어)

Rx / Ry / Rz는 EE 좌표 제어가 아닌 joint 직접 제어 방식.

| UI 축 | 관절 | index |
|-------|------|-------|
| Rx | joint4 | 3 |
| Ry | joint5 | 4 |
| Rz | joint6 | 5 |

---

## 주요 구현 사항

### 버튼 홀드 동작

버튼을 누르는 동안 계속 이동, 떼면 즉시 정지. `all_tasks: set`으로 실행 중인 모든 asyncio 태스크를 추적하여 `stop()` 시 전체 취소.

- 짧게 누르면: 1mm/° 이동 후 0.3초 대기
- 길게 누르면: `speed_slider × 0.5` mm/° 씩 50ms 간격으로 반복 이동

### E-STOP

`MotionCtrl_1(0x01, 0, 0x00)` 만 사용. `DisableArm` 사용 금지.

> DisableArm은 토크를 즉시 제거하여 로봇팔이 자유낙하함. MotionCtrl_1은 잔류 토크를 유지하여 천천히 내려앉음.

### 복구 시퀀스

`piper_single_ctrl_node.py`의 `_restore_sequence()` 내부 구현. 총 소요 시간 약 6초.

```
MotionCtrl 초기화 → GripperCtrl (1초 대기) → EnableArm × 5회 (1초 간격)
```

---

## 카메라 토픽 변경 방법

`src/piper_web_ui/piper_web_ui/web_ui_node.py`에서 아래 부분 수정:

```python
self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self._on_camera, 1)
```
