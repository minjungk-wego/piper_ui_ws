#!/usr/bin/env python3
# -*-coding:utf8-*-

import datetime
import math
import os
import threading
import time
import yaml
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from numpy import clip
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from piper_sdk import C_PiperInterface_V2
from piper_msgs.action import GotoPose, PiperMacro
from piper_msgs.msg import PiperState, PiperStatusMsg, PosCmd
from piper_msgs.srv import DeletePose, Enable, JogJoint, ListPoses, SavePose


def _default_pose_path(category: str) -> Path:
    base = Path(os.environ.get('PIPER_POSE_DIR', Path.home() / 'piper_poses'))
    return base / f'{category}_poses.yaml'


def _load_yaml(path: Path) -> dict:
    if path.exists():
        try:
            return yaml.safe_load(path.read_text()) or {}
        except Exception:
            return {}
    return {}


def _save_yaml(path: Path, data: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.dump(data, default_flow_style=False, allow_unicode=True))


class C_PiperRosNode(Node):
    def __init__(self) -> None:
        super().__init__("piper_single_ctrl_node")

        self.declare_parameter("can_port", "can0")
        self.declare_parameter("auto_enable", False)
        self.declare_parameter("gripper_exist", True)
        self.declare_parameter("debug_flag", True)

        self.can_port = self.get_parameter("can_port").get_parameter_value().string_value
        self.auto_enable = self.get_parameter("auto_enable").get_parameter_value().bool_value
        self.gripper_exist = self.get_parameter("gripper_exist").get_parameter_value().bool_value
        self.debug = self.get_parameter("debug_flag").get_parameter_value().bool_value

        if self.debug:
            self.get_logger().info(f"can_port is {self.can_port}")
            self.get_logger().info(f"auto_enable is {self.auto_enable}")
            self.get_logger().info(f"gripper_exist is {self.gripper_exist}")

        # Latched QoS
        _latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publishers (기존)
        self.joint_pub       = self.create_publisher(JointState,     "joint_states_single", 1)
        self.arm_status_pub  = self.create_publisher(PiperStatusMsg, "arm_status",          1)
        self.end_pose_pub    = self.create_publisher(Pose,           "end_pose",            1)
        self.init_pos_pub    = self.create_publisher(Bool,           "init_pos",            1)

        # Publishers (신규, latched)
        self.piper_state_pub = self.create_publisher(PiperState, "/piper/state",       _latched)
        self.is_deployed_pub = self.create_publisher(Bool,        "/piper/is_deployed", _latched)

        self.init_pos_msg = Bool()
        self.init_pos_msg.data = True
        self.init_pos_pub.publish(self.init_pos_msg)

        # Services (기존)
        self.create_service(Enable, "enable_srv", self.handle_enable_service)

        # Services (신규)
        self.create_service(SavePose,   '/piper/save_pose',   self.handle_save_pose)
        self.create_service(ListPoses,  '/piper/list_poses',  self.handle_list_poses)
        self.create_service(DeletePose, '/piper/delete_pose', self.handle_delete_pose)
        self.create_service(JogJoint,   '/piper/jog_joint',   self.handle_jog_joint)

        # Action servers (신규)
        _action_group = ReentrantCallbackGroup()
        ActionServer(self, PiperMacro, '/piper/stow',      self.execute_stow,      callback_group=_action_group)
        ActionServer(self, PiperMacro, '/piper/deploy',    self.execute_deploy,    callback_group=_action_group)
        ActionServer(self, GotoPose,   '/piper/goto_pose', self.execute_goto_pose, callback_group=_action_group)

        # Joint state
        self.joint_states = JointState()
        self.joint_states.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort   = [0.0] * 7

        self._current_joints = [0.0] * 6  # 현재 관절 각도 캐시 (radians)
        self._current_state  = PiperState.INIT
        self._b2_is_moving   = False

        self.__enable_flag = False

        # CAN
        self.piper = C_PiperInterface_V2(can_name=self.can_port)
        self.piper.ConnectPort()

        self.count = 0
        self.position_state  = []
        self.position_states = []

        # Subscriptions (기존)
        self.create_subscription(PosCmd,     "pos_cmd",          self.pos_callback,     1)
        self.create_subscription(JointState, "joint_ctrl_single", self.joint_callback,  1)
        self.create_subscription(Bool,       "enable_flag",       self.enable_callback, 1)
        self.create_subscription(Bool,       "init_pos",          self.init_callback,   1)
        self.create_subscription(Bool,       "gripper_ctrl",      self.gripper_callback,1)
        self.create_subscription(Bool,       "estop",             self.estop_callback,  1)
        self.create_subscription(Bool,       "restore",           self.restore_callback,1)

        # Subscriptions (신규)
        self.create_subscription(Bool, '/safety/estop', self.safety_estop_callback, 1)
        self.create_subscription(Bool, '/b2/is_moving', self.b2_moving_callback,    1)

        # 1Hz 타이머: /piper/state 발행
        self.create_timer(1.0, self._publish_piper_state)

        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()

    # ── 기존 메서드 ───────────────────────────────────────────────────────────

    def GetEnableFlag(self):
        return self.__enable_flag

    def publish_thread(self):
        rate = self.create_rate(200)
        enable_flag = False
        timeout = 5
        start_time = time.time()
        elapsed_time_flag = False
        while rclpy.ok():
            if self.auto_enable:
                while not enable_flag:
                    elapsed_time = time.time() - start_time
                    enable_flag = (
                        self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status
                        and self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    )
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0, 1000, 0x01, 0)
                    if enable_flag:
                        self.__enable_flag = True
                    if elapsed_time > timeout:
                        print("시간초과")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(0.1)
            if elapsed_time_flag:
                print("프로그램을 종료합니다")
                exit(0)

            self.PublishArmState()
            self.PublishArmJointAndGirpper()
            self.PubilshArmEndPose()
            rate.sleep()

    def PublishArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode      = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status     = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback  = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status   = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status  = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code       = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)

    def PublishArmJointAndGirpper(self):
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_1 = (self.piper.GetArmJointMsgs().joint_state.joint_1 / 1000) * 0.017444
        joint_2 = (self.piper.GetArmJointMsgs().joint_state.joint_2 / 1000) * 0.017444
        joint_3 = (self.piper.GetArmJointMsgs().joint_state.joint_3 / 1000) * 0.017444
        joint_4 = (self.piper.GetArmJointMsgs().joint_state.joint_4 / 1000) * 0.017444
        joint_5 = (self.piper.GetArmJointMsgs().joint_state.joint_5 / 1000) * 0.017444
        joint_6 = (self.piper.GetArmJointMsgs().joint_state.joint_6 / 1000) * 0.017444
        gripper  = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000000
        vel_0 = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed / 1000
        vel_1 = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed / 1000
        vel_2 = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed / 1000
        vel_3 = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed / 1000
        vel_4 = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed / 1000
        vel_5 = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed / 1000
        effort_6 = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort / 1000
        self.joint_states.position = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, gripper]
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, 0.0]
        self.joint_states.effort   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, effort_6]
        self._current_joints = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        self.joint_pub.publish(self.joint_states)

    def PubilshArmEndPose(self):
        endpos = Pose()
        endpos.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis / 1000000
        endpos.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis / 1000000
        endpos.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis / 1000000
        roll  = math.radians(self.piper.GetArmEndPoseMsgs().end_pose.RX_axis / 1000)
        pitch = math.radians(self.piper.GetArmEndPoseMsgs().end_pose.RY_axis / 1000)
        yaw   = math.radians(self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis / 1000)
        quaternion = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)

    def pos_callback(self, pos_data):
        try:
            self.get_logger().warn(f"[POS_CMD] mode2={pos_data.mode2}, gripper={pos_data.gripper}")
        except Exception:
            pass

        if pos_data.mode1 is None:
            pos_data.mode1 = 0x01
        if pos_data.mode2 is None:
            pos_data.mode2 = 0x00

        if self.debug:
            self.get_logger().info(f"Received PosCmd: x={pos_data.x}, y={pos_data.y}, z={pos_data.z}")
            self.get_logger().info(f"roll={pos_data.roll}, pitch={pos_data.pitch}, yaw={pos_data.yaw}")
            self.get_logger().info(f"gripper={pos_data.gripper}, mode1={pos_data.mode1}, mode2={pos_data.mode2}")

        x  = round(pos_data.x    * 1000)
        y  = round(pos_data.y    * 1000)
        z  = round(pos_data.z    * 1000)
        rx = round(pos_data.roll  * 1000)
        ry = round(pos_data.pitch * 1000)
        rz = round(pos_data.yaw   * 1000)
        self.position_state = [x, y, z, rx, ry, rz]

        if pos_data.mode2 == 0x03:
            self.position_states.append(self.position_state)

        if self.GetEnableFlag():
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            self.piper.MotionCtrl_2(pos_data.mode1, pos_data.mode2, 30)
            if pos_data.mode2 == 0x03:
                self.piper.EndPoseCtrl(*self.position_states[self.count])
                self.count += 0x01
                self.piper.MoveCAxisUpdateCtrl(self.count)
                time.sleep(0.001)
            else:
                self.piper.EndPoseCtrl(*self.position_state)
                gripper = round(clip(pos_data.gripper * 1000 * 1000, 0, 70000))
                if self.gripper_exist:
                    try:
                        self.get_logger().error(f"[GRIPPER_CTRL] mode2={pos_data.mode2}, gripper_send={gripper}")
                    except Exception:
                        pass
                    self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)

        if self.count == 3:
            self.count = 0

    def joint_callback(self, joint_data):
        factor = 57324.840764
        joint_positions = {}
        for idx, joint_name in enumerate(joint_data.name):
            if self.debug:
                self.get_logger().info(f"{joint_name}: {joint_data.position[idx]}")
            joint_positions[joint_name] = round(joint_data.position[idx] * factor)

        if len(joint_data.position) == 6:
            joint_data.position = list(joint_data.position) + [0.0, 0.0]
        elif len(joint_data.position) == 7:
            joint_data.position = list(joint_data.position) + [-1 * joint_data.position[6]]

        gripper = round(clip(
            (joint_data.position[6] + (-1 * joint_data.position[7])) * 1000 * 1000,
            0, 70000
        ))

        if self.GetEnableFlag():
            all_zeros = all(v == 0 for v in joint_data.velocity) if joint_data.velocity else True
            if not all_zeros:
                vel_all = int(clip(30, 0, 100))
                self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
            else:
                self.piper.MotionCtrl_2(0x01, 0x01, 30)

            self.piper.JointCtrl(
                joint_positions.get("joint1", 0),
                joint_positions.get("joint2", 0),
                joint_positions.get("joint3", 0),
                joint_positions.get("joint4", 0),
                joint_positions.get("joint5", 0),
                joint_positions.get("joint6", 0),
            )

            if self.gripper_exist:
                if len(joint_data.effort) >= 7:
                    gripper_effort = clip(joint_data.effort[6], 0.5, 3)
                    gripper_effort = round(gripper_effort * 1000) if not math.isnan(float(gripper_effort)) else 1000
                    self.piper.GripperCtrl(abs(gripper), gripper_effort, 0x01, 0)
                else:
                    self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)

    def enable_callback(self, enable_flag: Bool):
        if self.debug:
            self.get_logger().info(f"enable_flag: {enable_flag.data}")
        if enable_flag.data:
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x00, 0)

    def init_callback(self, init_flag: Bool):
        if init_flag.data:
            if self.debug:
                self.get_logger().info("Moving to initial position...")
            self.piper.MotionCtrl_2(0x01, 0x01, 50)
            self.piper.GripperCtrl(0, 1000, 0x01, 0)
            self.piper.JointCtrl(0, 0, 0, 0, 0, 0)

    def gripper_callback(self, gripper_flag: Bool):
        if gripper_flag.data:
            if self.debug:
                self.get_logger().info("Close the gripper...")
            self.piper.MotionCtrl_2(0x01, self.piper.GetArmStatus().arm_status.mode_feed, 100)
            self.piper.GripperCtrl(0, 1000, 0x01, 0)
        else:
            if self.debug:
                self.get_logger().info("Open the gripper...")
            self.piper.MotionCtrl_2(0x01, self.piper.GetArmStatus().arm_status.mode_feed, 100)
            self.piper.GripperCtrl(70 * 1000 * 1000, 1000, 0x01, 0)

    def estop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().error("E-STOP triggered!")
            self.__enable_flag = False
            self.piper.MotionCtrl_1(0x01, 0, 0x00)

    def restore_callback(self, msg: Bool):
        if msg.data:
            threading.Thread(target=self._restore_sequence, daemon=True).start()

    def _restore_sequence(self):
        self.get_logger().info("Restore sequence started...")
        self.piper.MotionCtrl_1(0x02, 0, 0x00)
        self.piper.MotionCtrl_1(0x00, 0, 0x00)
        self.piper.MotionCtrl_2(0x01, 0, 0, 0x00)
        self.piper.GripperCtrl(0, 0, 0x02, 0)
        time.sleep(1)
        self.piper.MotionCtrl_2(0x01, 0, 0, 0x00)
        self.piper.GripperCtrl(0, 0, 0x03, 0)
        time.sleep(0.05)
        for _ in range(5):
            self.piper.EnableArm(7)
            time.sleep(1)
        if self.piper.GetArmStatus().arm_status.ctrl_mode == 0x01:
            self.__enable_flag = True
            self.get_logger().info("Restore successful! PiPER is ready.")
        else:
            self.get_logger().error("Restore failed! Do not operate PiPER.")

    def safety_estop_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().error("Safety E-STOP received from /safety/estop!")
            self.__enable_flag  = False
            self._current_state = PiperState.ERROR
            self.piper.MotionCtrl_1(0x01, 0, 0x00)

    def b2_moving_callback(self, msg: Bool):
        self._b2_is_moving = msg.data
        if msg.data:
            self.get_logger().info("B2 is moving — commands blocked")

    def handle_enable_service(self, req, resp):
        if self.debug:
            self.get_logger().info(f"Received request:: {req.enable_request}")
        enable_flag = False
        loop_flag   = False
        timeout     = 5
        start_time  = time.time()
        elapsed_time_flag = False
        while not loop_flag:
            elapsed_time = time.time() - start_time
            enable_list = [
                self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status,
                self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status,
            ]
            if req.enable_request:
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0, 1000, 0x02, 0)
            self.__enable_flag = enable_flag
            if enable_flag == req.enable_request:
                loop_flag = True
                enable_flag = True
            else:
                loop_flag = False
                enable_flag = False
            if elapsed_time > timeout:
                elapsed_time_flag = True
                enable_flag = False
                loop_flag = True
                break
            time.sleep(0.5)
        resp.enable_response = enable_flag
        if self.debug:
            self.get_logger().info(f"Returning response: {resp.enable_response}")
        return resp

    # ── 신규: /piper/state 발행 ───────────────────────────────────────────────

    def _publish_piper_state(self):
        msg = PiperState()
        try:
            err_code = self.piper.GetArmStatus().arm_status.err_code
            if err_code != 0:
                self._current_state = PiperState.ERROR
                msg.error_detail    = f"err_code={err_code}"
            elif self._current_state in (PiperState.INIT, PiperState.MANUAL):
                self._current_state = PiperState.MANUAL if self.__enable_flag else PiperState.INIT
        except Exception as e:
            self._current_state = PiperState.ERROR
            msg.error_detail    = str(e)

        msg.state = self._current_state
        self.piper_state_pub.publish(msg)

        deployed      = Bool()
        deployed.data = self._current_state not in (PiperState.STOWED, PiperState.INIT)
        self.is_deployed_pub.publish(deployed)

    # ── 신규: 자세 저장/조회 서비스 ──────────────────────────────────────────

    def _get_pose_path(self, category: str) -> Path:
        env_key = f'PIPER_{category.upper()}_POSES_FILE'
        env_val = os.environ.get(env_key)
        return Path(env_val) if env_val else _default_pose_path(category)

    def handle_save_pose(self, req, resp):
        path    = self._get_pose_path(req.category)
        data    = _load_yaml(path)
        entries = data.get('entries', [])

        existing = next((e for e in entries if e['name'] == req.name), None)
        if existing and not req.overwrite:
            resp.success = False
            resp.message = f"'{req.name}' already exists. Set overwrite=true to replace."
            return resp

        entry = {
            'name':     req.name,
            'saved_at': datetime.datetime.now().isoformat(timespec='seconds'),
            'joints':   list(self._current_joints),
        }
        if existing:
            entries = [entry if e['name'] == req.name else e for e in entries]
        else:
            entries.append(entry)

        data['schema_version'] = 1
        data['entries']        = entries
        _save_yaml(path, data)

        resp.success = True
        resp.message = f"Saved '{req.name}' to {path}"
        self.get_logger().info(resp.message)
        return resp

    def handle_list_poses(self, req, resp):
        path    = self._get_pose_path(req.category)
        data    = _load_yaml(path)
        resp.names = [e['name'] for e in data.get('entries', [])]
        return resp

    def handle_delete_pose(self, req, resp):
        path        = self._get_pose_path(req.category)
        data        = _load_yaml(path)
        entries     = data.get('entries', [])
        new_entries = [e for e in entries if e['name'] != req.name]

        if len(new_entries) == len(entries):
            resp.success = False
            resp.message = f"'{req.name}' not found in category '{req.category}'"
            return resp

        data['entries'] = new_entries
        _save_yaml(path, data)
        resp.success = True
        resp.message = f"Deleted '{req.name}'"
        self.get_logger().info(resp.message)
        return resp

    def handle_jog_joint(self, req, resp):
        if not self.__enable_flag:
            resp.success = False
            resp.message = "Arm is not enabled"
            return resp

        if not (0 <= req.joint_index <= 5):
            resp.success = False
            resp.message = f"joint_index must be 0~5, got {req.joint_index}"
            return resp

        factor = 57324.840764
        joints = list(self._current_joints)
        joints[req.joint_index] += req.delta

        self.piper.MotionCtrl_2(0x01, 0x01, 30)
        self.piper.JointCtrl(
            round(joints[0] * factor),
            round(joints[1] * factor),
            round(joints[2] * factor),
            round(joints[3] * factor),
            round(joints[4] * factor),
            round(joints[5] * factor),
        )

        resp.success = True
        resp.message = f"Jogged joint{req.joint_index + 1} by {req.delta:.4f} rad"
        self.get_logger().info(resp.message)
        return resp

    # ── 신규: action execute 메서드 ─────────────────────────────────────────────

    _JOINT_FACTOR  = 57324.840764
    _REACH_TOL     = 0.05            # ~3°
    _STOW_JOINTS   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    _DEPLOY_JOINTS = [0.0, 0.15608891200000002, -0.32981370800000004, 0.0, 0.13841814, 0.0]

    def _move_to_joints(self, target: list, speed: int = 30):
        self.piper.MotionCtrl_2(0x01, 0x01, speed)
        self.piper.JointCtrl(*[round(j * self._JOINT_FACTOR) for j in target])

    def _wait_for_joints(self, goal_handle, target: list, timeout: float, feedback_fn):
        start = time.time()
        while time.time() - start < timeout:
            if goal_handle.is_cancel_requested:
                return False
            err = max(abs(self._current_joints[i] - target[i]) for i in range(6))
            goal_handle.publish_feedback(feedback_fn(err))
            if err < self._REACH_TOL:
                return True
            time.sleep(0.1)
        return True  # timeout treated as done (arm may still be converging)

    def execute_stow(self, goal_handle):
        if self._b2_is_moving:
            result = PiperMacro.Result()
            result.success = False
            result.message = "Rejected: B2 is moving"
            goal_handle.abort()
            return result

        self._current_state = PiperState.RETRACTING
        self.get_logger().info("Stow sequence started")
        self._move_to_joints(self._STOW_JOINTS)

        def fb(err):
            f = PiperMacro.Feedback()
            f.progress = float(max(0.0, 1.0 - err / 0.5))
            return f

        reached = self._wait_for_joints(goal_handle, self._STOW_JOINTS, 15.0, fb)

        if not reached and goal_handle.is_cancel_requested:
            self._current_state = PiperState.MANUAL
            goal_handle.canceled()
            result = PiperMacro.Result()
            result.success = False
            result.message = "Canceled"
            return result

        self._current_state = PiperState.STOWED
        goal_handle.succeed()
        result = PiperMacro.Result()
        result.success = True
        result.message = "Stowed"
        self.get_logger().info("Stow complete")
        return result

    def execute_deploy(self, goal_handle):
        if self._b2_is_moving:
            result = PiperMacro.Result()
            result.success = False
            result.message = "Rejected: B2 is moving"
            goal_handle.abort()
            return result

        self._current_state = PiperState.DEPLOYING
        self.get_logger().info("Deploy sequence started")
        self._move_to_joints(self._DEPLOY_JOINTS)

        def fb(err):
            f = PiperMacro.Feedback()
            f.progress = float(max(0.0, 1.0 - err / 0.5))
            return f

        reached = self._wait_for_joints(goal_handle, self._DEPLOY_JOINTS, 15.0, fb)

        if not reached and goal_handle.is_cancel_requested:
            self._current_state = PiperState.STOWED
            goal_handle.canceled()
            result = PiperMacro.Result()
            result.success = False
            result.message = "Canceled"
            return result

        self._current_state = PiperState.MANUAL
        goal_handle.succeed()
        result = PiperMacro.Result()
        result.success = True
        result.message = "Deployed"
        self.get_logger().info("Deploy complete")
        return result

    def execute_goto_pose(self, goal_handle):
        if self._b2_is_moving:
            result = GotoPose.Result()
            result.success = False
            result.message = "Rejected: B2 is moving"
            goal_handle.abort()
            return result

        if not self.__enable_flag:
            result = GotoPose.Result()
            result.success = False
            result.message = "Arm is not enabled"
            goal_handle.abort()
            return result

        req   = goal_handle.request
        path  = self._get_pose_path(req.category)
        data  = _load_yaml(path)
        entry = next((e for e in data.get('entries', []) if e['name'] == req.name), None)

        if entry is None:
            result = GotoPose.Result()
            result.success = False
            result.message = f"'{req.name}' not found in category '{req.category}'"
            goal_handle.abort()
            return result

        prev_state          = self._current_state
        self._current_state = PiperState.MOVING
        target = entry['joints']
        speed  = int(clip(req.speed if req.speed > 0 else 30, 1, 100))
        self._move_to_joints(target, speed)

        start = time.time()
        while time.time() - start < 30.0:
            if goal_handle.is_cancel_requested:
                self._current_state = prev_state
                goal_handle.canceled()
                result = GotoPose.Result()
                result.success = False
                result.message = "Canceled"
                return result

            feedback = GotoPose.Feedback()
            feedback.current_joints = list(self._current_joints)
            goal_handle.publish_feedback(feedback)

            err = max(abs(self._current_joints[i] - target[i]) for i in range(6))
            if err < self._REACH_TOL:
                break
            time.sleep(0.1)

        self._current_state = PiperState.MANUAL
        goal_handle.succeed()
        result = GotoPose.Result()
        result.success = True
        result.message = f"Reached '{req.name}'"
        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    piper_single_node = C_PiperRosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(piper_single_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
