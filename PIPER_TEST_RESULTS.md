# PiPER ROS2 인터페이스 테스트 결과

> 테스트 날짜: 2026-05-20  
> 환경: Ubuntu 22.04, ROS2 Humble, USB-CAN 직접 연결

---

## 테스트 환경

```
노드 실행: ros2 launch piper start_single_piper.launch.py
CAN 포트: can0
```

---

## 1단계 — 상태 토픽

### `/piper/state`

```bash
ros2 topic echo /piper/state --once
```

**결과**
```
state: 1
error_detail: ''
joint_temperatures_celsius: []
thermal_warning: false
```

- `state: 1` = MANUAL (arm enabled, 명령 대기 중) ✅
- Web UI에서 E-STOP / Disable 시 `state: 0` (INIT)으로 변경 확인 ✅

---

### `/piper/is_deployed`

```bash
ros2 topic echo /piper/is_deployed
```

**결과**
```
data: true
```

- MANUAL 상태에서 `true` 확인 ✅
- Web UI E-STOP / Disable 시 `false`로 변경 확인 ✅

---

## 2단계 — 자세 저장/조회/삭제 서비스

### `/piper/save_pose`

```bash
ros2 service call /piper/save_pose piper_msgs/srv/SavePose "{name: 'test_pose', category: 'default', overwrite: false}"
```

**결과**
```
success: True
message: "Saved 'test_pose' to /home/wego/piper_poses/default_poses.yaml"
```

✅ 성공

**생성된 YAML 파일 (`~/piper_poses/default_poses.yaml`)**
```yaml
entries:
- joints:
  - 0.0
  - 0.791277284
  - -0.625489508
  - 0.0
  - -0.126189896
  - 0.0
  name: test_pose
  saved_at: '2026-05-20T11:05:33'
schema_version: 1
```

---

### `/piper/list_poses`

```bash
ros2 service call /piper/list_poses piper_msgs/srv/ListPoses "{category: 'default'}"
```

**결과**
```
names: ['test_pose']
```

✅ 성공

---

### `/piper/delete_pose`

```bash
ros2 service call /piper/delete_pose piper_msgs/srv/DeletePose "{name: 'test_pose', category: 'default'}"
```

**결과**
```
success: True
message: "Deleted 'test_pose'"
```

삭제 후 YAML:
```yaml
entries: []
schema_version: 1
```

✅ 성공

---

## 3단계 — JogJoint 서비스

### `/piper/jog_joint`

```bash
ros2 service call /piper/jog_joint piper_msgs/srv/JogJoint "{joint_index: 0, delta: 0.05, step: 0.0}"
```

**joint1 변화**

| | joint1 |
|---|---|
| 이전 | `0.0` |
| 이후 | `0.050273608` |

- delta `0.05 rad` 요청 → 실제 `0.050273608 rad` 이동 ✅

---

## 4단계 — Action: stow / deploy

### `/piper/stow`

```bash
ros2 action send_goal /piper/stow piper_msgs/action/PiperMacro "{pose_name: ''}" --feedback
```

**결과**
```
success: true
message: Stowed
```

| 확인 항목 | 결과 |
|-----------|------|
| `/piper/state` → `3` (STOWED) | ✅ |
| `/piper/is_deployed` → `false` | ✅ |

---

### `/piper/deploy`

```bash
ros2 action send_goal /piper/deploy piper_msgs/action/PiperMacro "{pose_name: ''}" --feedback
```

| 확인 항목 | 결과 |
|-----------|------|
| `/piper/state` → `3` → `5` → `1` 순서 전환 | ✅ |
| `/piper/is_deployed` → `true` | ✅ |
| 액션 실행 중 토픽 끊김 없음 (MultiThreadedExecutor 적용) | ✅ |

---

## 5단계 — Action: goto_pose

### `/piper/goto_pose`

```bash
ros2 action send_goal /piper/goto_pose piper_msgs/action/GotoPose "{name: 'test_pose', category: 'default', speed: 30.0}" --feedback
```

**결과**
```
success: true
message: Reached 'test_pose'
```

**Feedback (joint1 수렴 과정)**

| 시점 | joint1 |
|------|--------|
| 1 | 0.200536 |
| 2 | 0.184435 |
| 3 | 0.137842 |
| 4 | 0.025799 |

| 확인 항목 | 결과 |
|-----------|------|
| 저장된 자세로 복귀 | ✅ |
| Feedback으로 현재 joint 실시간 수신 | ✅ |
| `success: true` / `SUCCEEDED` | ✅ |

※ `/piper/state`의 MOVING(2) 상태는 이동 거리가 짧아 1Hz 타이머 주기 내에 완료되어 캡처 안 됨 (정상 동작)

---

## 전체 진행 현황

| 단계 | 항목 | 결과 |
|------|------|------|
| 1단계 | `/piper/state` | ✅ |
| 1단계 | `/piper/is_deployed` | ✅ |
| 2단계 | `/piper/save_pose` | ✅ |
| 2단계 | `/piper/list_poses` | ✅ |
| 2단계 | `/piper/delete_pose` | ✅ |
| 3단계 | `/piper/jog_joint` | ✅ |
| 4단계 | `/piper/stow` | ✅ |
| 4단계 | `/piper/deploy` | ✅ |
| 5단계 | `/piper/goto_pose` | ✅ |
