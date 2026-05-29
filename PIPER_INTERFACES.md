# PiPER ROS2 신규 인터페이스 정의

> 작성일: 2026-05-20  
> 대상 패키지: `piper_msgs`, `piper`

---

## 1. Message Types

### `piper_msgs/msg/PiperState`

PiPER 로봇팔의 현재 FSM 상태를 나타내는 메시지.

| 필드 | 타입 | 설명 |
|------|------|------|
| `state` | `uint8` | 현재 상태 (아래 enum 참조) |
| `error_detail` | `string` | ERROR 상태일 때 오류 상세 내용 |
| `joint_temperatures_celsius` | `float32[]` | 각 관절 온도 (°C) |
| `thermal_warning` | `bool` | 온도 경고 여부 (기준: 70°C 초과) |

**State Enum**

| 값 | 이름 | 설명 |
|----|------|------|
| 0 | `INIT` | 비활성 / 미연결 |
| 1 | `MANUAL` | 활성화됨, 명령 대기 중 |
| 2 | `MOVING` | goto_pose 이동 중 |
| 3 | `STOWED` | 안전하게 접힌 상태 |
| 4 | `ERROR` | 오류 발생 |
| 5 | `DEPLOYING` | deploy 동작 중 |
| 6 | `MEASURING` | 측정 중 |
| 7 | `RETRACTING` | stow 동작 중 |

---

## 2. Topics

### `/piper/state`

| 항목 | 내용 |
|------|------|
| 메시지 타입 | `piper_msgs/msg/PiperState` |
| 발행 주기 | 1 Hz |
| QoS | Latched (transient_local, RELIABLE, depth=1) |
| 발행자 | `piper_single_ctrl_node` |
| 설명 | PiPER의 현재 FSM 상태를 주기적으로 발행. 하드웨어 에러 감지 시 자동으로 ERROR 상태로 전환. |

### `/piper/is_deployed`

| 항목 | 내용 |
|------|------|
| 메시지 타입 | `std_msgs/Bool` |
| 발행 주기 | 1 Hz (state와 동기) |
| QoS | Latched (transient_local, RELIABLE, depth=1) |
| 발행자 | `piper_single_ctrl_node` |
| 설명 | B2 로봇이 이동 가능한지 판단하는 인터록 신호. Nav2의 cmd_vel 차단 용도. |

**값 기준**

| 값 | 해당 상태 | 의미 |
|----|-----------|------|
| `true` | MANUAL, MOVING, DEPLOYING, MEASURING, RETRACTING, ERROR | 팔이 공간을 차지 → B2 이동 금지 |
| `false` | INIT, STOWED | 팔이 안전하게 접힘 → B2 이동 가능 |

### `/safety/estop` (구독)

| 항목 | 내용 |
|------|------|
| 메시지 타입 | `std_msgs/Bool` |
| 설명 | `true` 수신 시 즉시 `MotionCtrl_1(0x01, 0, 0x00)` 호출하여 모든 동작 정지. 상태를 ERROR로 전환. |

### `/b2/is_moving` (구독)

| 항목 | 내용 |
|------|------|
| 메시지 타입 | `std_msgs/Bool` |
| 설명 | `true`일 때 모든 arm 명령(서비스/액션) 거부. B2 이동 중 충돌 방지. |

---

## 3. Services

### `/piper/save_pose` — `piper_msgs/srv/SavePose`

현재 관절 각도를 이름을 붙여 YAML 파일에 저장.

**Request**

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | 저장할 자세 이름 |
| `category` | `string` | 카테고리 (예: `default`, `measurement`, `stow`) |
| `overwrite` | `bool` | 동일 이름 존재 시 덮어쓰기 여부 |

**Response**

| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | `bool` | 저장 성공 여부 |
| `message` | `string` | 결과 메시지 |

**저장 위치**: 환경변수 `PIPER_{CATEGORY}_POSES_FILE` > `$PIPER_POSE_DIR/{category}_poses.yaml` > `~/piper_poses/{category}_poses.yaml`

---

### `/piper/list_poses` — `piper_msgs/srv/ListPoses`

특정 카테고리에 저장된 자세 이름 목록 조회.

**Request**

| 필드 | 타입 | 설명 |
|------|------|------|
| `category` | `string` | 조회할 카테고리 |

**Response**

| 필드 | 타입 | 설명 |
|------|------|------|
| `names` | `string[]` | 저장된 자세 이름 배열 |

---

### `/piper/delete_pose` — `piper_msgs/srv/DeletePose`

특정 자세를 YAML 파일에서 삭제.

**Request**

| 필드 | 타입 | 설명 |
|------|------|------|
| `name` | `string` | 삭제할 자세 이름 |
| `category` | `string` | 카테고리 |

**Response**

| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | `bool` | 삭제 성공 여부 |
| `message` | `string` | 결과 메시지 |

---

### `/piper/jog_joint` — `piper_msgs/srv/JogJoint`

특정 관절을 지정한 각도만큼 즉시 이동.

**Request**

| 필드 | 타입 | 설명 |
|------|------|------|
| `joint_index` | `int32` | 관절 인덱스 (0~5, joint1~joint6) |
| `delta` | `float64` | 이동량 (radians) |
| `step` | `float64` | 예약 필드 (현재 미사용) |

**Response**

| 필드 | 타입 | 설명 |
|------|------|------|
| `success` | `bool` | 성공 여부 |
| `message` | `string` | 결과 메시지 |

---

## 4. Actions

### `/piper/stow` — `piper_msgs/action/PiperMacro`

팔을 안전하게 접힌 자세(STOW)로 이동.

- 상태 전환: `RETRACTING` → 완료 시 `STOWED`
- 완료 후 `is_deployed = false`
- B2 이동 중(`/b2/is_moving=true`)이면 즉시 거부

| 필드 | 타입 | 설명 |
|------|------|------|
| Goal: `pose_name` | `string` | 현재 미사용 (예약) |
| Feedback: `progress` | `float32` | 진행률 0.0~1.0 |
| Result: `success` | `bool` | 성공 여부 |
| Result: `message` | `string` | 결과 메시지 |

---

### `/piper/deploy` — `piper_msgs/action/PiperMacro`

팔을 운용 준비 자세(DEPLOY)로 이동.

- 상태 전환: `DEPLOYING` → 완료 시 `MANUAL`
- 완료 후 `is_deployed = true`
- B2 이동 중이면 즉시 거부

(필드 구성은 `/piper/stow`와 동일)

---

### `/piper/goto_pose` — `piper_msgs/action/GotoPose`

저장된 자세로 이동.

- 상태 전환: `MOVING` → 완료 시 `MANUAL`
- arm이 비활성(INIT)이거나 B2 이동 중이면 거부

| 필드 | 타입 | 설명 |
|------|------|------|
| Goal: `name` | `string` | 이동할 자세 이름 |
| Goal: `category` | `string` | 카테고리 |
| Goal: `speed` | `float64` | 이동 속도 1~100 (기본값: 30) |
| Feedback: `current_joints` | `float64[]` | 현재 관절 각도 (radians) |
| Result: `success` | `bool` | 성공 여부 |
| Result: `message` | `string` | 결과 메시지 |

---

## 5. YAML 자세 파일 스키마

```yaml
schema_version: 1
entries:
  - name: pose_name
    saved_at: '2026-05-20T11:05:33'
    joints:
      - 0.0   # joint1 (radians)
      - 0.0   # joint2
      - 0.0   # joint3
      - 0.0   # joint4
      - 0.0   # joint5
      - 0.0   # joint6
```
