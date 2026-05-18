#!/usr/bin/env python3
import asyncio
import threading

import math

import rclpy
from geometry_msgs.msg import Pose
from piper_msgs.msg import PosCmd
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from nicegui import ui


class RobotState:
    def __init__(self):
        self._lock = threading.Lock()
        self.x = self.y = self.z = 0.0     # mm
        self.rx = self.ry = self.rz = 0.0  # degrees
        self.gripper = 0.0                  # mm
        self.joints = [0.0] * 6            # radians, joint1~6

    def update_from_pose(self, pose: Pose):
        with self._lock:
            self.x = pose.position.x * 1000.0
            self.y = pose.position.y * 1000.0
            self.z = pose.position.z * 1000.0
            q = [pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w]
            e = R.from_quat(q).as_euler('xyz', degrees=True)
            self.rx, self.ry, self.rz = float(e[0]), float(e[1]), float(e[2])

    def apply_delta_and_snapshot(self, axis: str, val: float):
        with self._lock:
            if axis == 'x':    self.x  += val
            elif axis == 'y':  self.y  += val
            elif axis == 'z':  self.z  += val
            elif axis == 'rx': self.rx += val
            elif axis == 'ry': self.ry += val
            elif axis == 'rz': self.rz += val
            return (self.x, self.y, self.z, self.rx, self.ry, self.rz, self.gripper)

    def update_joints(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name.startswith('joint') and i < len(msg.position):
                    try:
                        idx = int(name.replace('joint', '')) - 1
                        if 0 <= idx < 6:
                            self.joints[idx] = float(msg.position[i])
                    except ValueError:
                        pass

    def apply_joint_delta_and_snapshot(self, joint_idx: int, delta_rad: float):
        with self._lock:
            self.joints[joint_idx] += delta_rad
            return list(self.joints)

    def set_gripper_and_snapshot(self, mm: float):
        with self._lock:
            self.gripper = mm
            return (self.x, self.y, self.z, self.rx, self.ry, self.rz, self.gripper)

    def snapshot(self):
        with self._lock:
            return (self.x, self.y, self.z,
                    self.rx, self.ry, self.rz,
                    self.gripper)


state = RobotState()
_node: 'WebUINode | None' = None


class WebUINode(Node):
    def __init__(self):
        super().__init__('piper_web_ui_node')
        self.pos_pub    = self.create_publisher(PosCmd, 'pos_cmd',    1)
        self.enable_pub = self.create_publisher(Bool,   'enable_flag', 1)
        self.init_pub   = self.create_publisher(Bool,   'init_pos',   1)
        self.estop_pub   = self.create_publisher(Bool,       'estop',            1)
        self.restore_pub = self.create_publisher(Bool,       'restore',          1)
        self.joint_pub   = self.create_publisher(JointState, 'joint_states', 1)
        self.create_subscription(Pose,       'end_pose',          self._on_end_pose,   1)
        self.create_subscription(JointState, 'joint_states_single', self._on_joints,   1)

    def _on_end_pose(self, msg: Pose):
        state.update_from_pose(msg)

    def _on_joints(self, msg: JointState):
        state.update_joints(msg)

    def move(self, axis: str, delta: float):
        vals = state.apply_delta_and_snapshot(axis, delta)
        self._publish(*vals)

    def set_gripper(self, mm: float):
        vals = state.set_gripper_and_snapshot(mm)
        self._publish(*vals)

    def _publish(self, x, y, z, rx, ry, rz, g):
        msg = PosCmd()
        msg.x, msg.y, msg.z = x, y, z
        msg.roll, msg.pitch, msg.yaw = rx, ry, rz
        msg.gripper = g
        msg.mode1 = 0x01
        msg.mode2 = 0x00
        self.pos_pub.publish(msg)

    def enable(self, val: bool):
        m = Bool(); m.data = val
        self.enable_pub.publish(m)

    def go_init(self):
        m = Bool(); m.data = True
        self.init_pub.publish(m)

    def estop(self):
        m = Bool(); m.data = True
        self.estop_pub.publish(m)

    def restore(self):
        m = Bool(); m.data = True
        self.restore_pub.publish(m)

    def move_joint(self, joint_idx: int, delta_deg: float):
        delta_rad = delta_deg * math.pi / 180.0
        joints = state.apply_joint_delta_and_snapshot(joint_idx, delta_rad)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = joints
        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    global _node
    _node = WebUINode()
    threading.Thread(target=rclpy.spin, args=(_node,), daemon=True).start()

    @ui.page('/')
    def index():
        ui.add_head_html(
            '<meta name="viewport" content="width=device-width, initial-scale=1.0,'
            ' maximum-scale=1.0, user-scalable=no">'
            '<style>'
            '* { touch-action: manipulation; -webkit-user-select: none; user-select: none; }'
            '.arrow-btn { transition: filter 0.1s; }'
            '.arrow-btn:active { filter: brightness(0.75); }'
            '</style>'
        )
        ui.query('body').style('background-color: #111827')

        CARD = 'background:#1f2937; border:none'

        # CSS clip-path arrow shapes
        ARROWS = {
            'up':    'polygon(50% 0%, 100% 45%, 75% 45%, 75% 100%, 25% 100%, 25% 45%, 0% 45%)',
            'down':  'polygon(25% 0%, 75% 0%, 75% 55%, 100% 55%, 50% 100%, 0% 55%, 25% 55%)',
            'left':  'polygon(0% 50%, 45% 0%, 45% 25%, 100% 25%, 100% 75%, 45% 75%, 45% 100%)',
            'right': 'polygon(55% 0%, 100% 50%, 55% 100%, 55% 75%, 0% 75%, 0% 25%, 55% 25%)',
        }

        def hold_btn(label, axis, direction, is_rot, color, arrow_dir):
            task = {'ref': None}

            async def keep_moving():
                try:
                    if is_rot:
                        step = float(deg_radio.value)
                        _node.move_joint(axis, direction * step)
                        await asyncio.sleep(0.3)
                        while True:
                            _node.move_joint(axis, direction * 1.0)
                            await asyncio.sleep(0.05)
                    else:
                        step = float(mm_radio.value)
                        _node.move(axis, direction * step)
                        await asyncio.sleep(0.3)
                        while True:
                            _node.move(axis, direction * 1.0)
                            await asyncio.sleep(0.05)
                except asyncio.CancelledError:
                    pass

            async def start():
                task['ref'] = asyncio.create_task(keep_moving())

            async def stop():
                if task['ref'] and not task['ref'].done():
                    task['ref'].cancel()
                task['ref'] = None

            clip = ARROWS[arrow_dir]
            btn = (ui.button(label).classes('arrow-btn')
                   .style(
                       f'background:{color}; color:white; clip-path:{clip};'
                       ' border-radius:0; border:none; width:76px; height:76px;'
                       ' font-size:11px; font-weight:700; padding:0; min-width:0; box-shadow:none'
                   ))
            btn.on('pointerdown', start)
            btn.on('pointerup',   stop)
            btn.on('pointercancel', stop)
            return btn

        def dpad(u_lbl, u_ax, u_dir, u_col,
                 d_lbl, d_ax, d_dir, d_col,
                 l_lbl, l_ax, l_dir, l_col,
                 r_lbl, r_ax, r_dir, r_col, is_rot):
            SPC = 'width:76px; height:76px; display:block; flex-shrink:0'
            with ui.column().classes('items-center w-full gap-2'):
                with ui.row().classes('gap-2 justify-center items-center'):
                    ui.element('div').style(SPC)
                    hold_btn(u_lbl, u_ax, u_dir, is_rot, u_col, 'up')
                    ui.element('div').style(SPC)
                with ui.row().classes('gap-2 justify-center items-center'):
                    hold_btn(l_lbl, l_ax, l_dir, is_rot, l_col, 'left')
                    ui.element('div').style(
                        f'{SPC}; border-radius:50%; background:#374151')
                    hold_btn(r_lbl, r_ax, r_dir, is_rot, r_col, 'right')
                with ui.row().classes('gap-2 justify-center items-center'):
                    ui.element('div').style(SPC)
                    hold_btn(d_lbl, d_ax, d_dir, is_rot, d_col, 'down')
                    ui.element('div').style(SPC)

        with ui.column().classes('w-full max-w-md mx-auto p-4 gap-3'):
            ui.label('Piper Web UI').classes('text-2xl text-white font-bold text-center w-full')

            # E-STOP + 복구
            with ui.row().classes('w-full gap-2'):
                ui.button('E-STOP', on_click=lambda: _node.estop()) \
                    .classes('flex-1 h-14 text-xl font-bold') \
                    .style('background:#dc2626; color:white; border-radius:8px; letter-spacing:2px')
                ui.button('복구', on_click=lambda: _node.restore()) \
                    .classes('flex-1 h-14 text-sm font-bold') \
                    .style('background:#0891b2; color:white; border-radius:8px')

            # 현재 EE 위치
            with ui.card().classes('w-full').style(CARD):
                ui.label('현재 EE 위치').classes('text-gray-400 text-xs mb-2')
                with ui.grid(columns=3).classes('w-full gap-1 text-center'):
                    lbl = {
                        'x':  ui.label('X: — mm').classes('text-green-400 text-sm'),
                        'y':  ui.label('Y: — mm').classes('text-green-400 text-sm'),
                        'z':  ui.label('Z: — mm').classes('text-green-400 text-sm'),
                        'rx': ui.label('Rx: — °').classes('text-yellow-400 text-sm'),
                        'ry': ui.label('Ry: — °').classes('text-yellow-400 text-sm'),
                        'rz': ui.label('Rz: — °').classes('text-yellow-400 text-sm'),
                    }

            # 스텝 크기
            with ui.card().classes('w-full').style(CARD):
                ui.label('스텝 크기').classes('text-gray-400 text-xs mb-2')
                with ui.row().classes('items-center gap-3 w-full'):
                    ui.label('이동 (mm)').classes('text-white text-sm w-20')
                    mm_radio = ui.radio([1, 5, 10], value=5).props('inline')
                with ui.row().classes('items-center gap-3 w-full'):
                    ui.label('회전 (°)').classes('text-white text-sm w-20')
                    deg_radio = ui.radio([1, 5, 10], value=5).props('inline')

            # TCP 위치
            with ui.card().classes('w-full').style(CARD):
                ui.label('TCP 위치').classes('text-gray-400 text-xs mb-3')
                # Z: up/down arrows as a pair
                with ui.row().classes('w-full justify-center gap-4 mb-2'):
                    hold_btn('Z+', 'z', +1, False, '#1d4ed8', 'up')
                    hold_btn('Z-', 'z', -1, False, '#1d4ed8', 'down')
                ui.label('Z').classes('text-center text-blue-400 text-xs w-full -mt-1 mb-1')
                # XY cross
                dpad('X+', 'x', +1, '#dc2626',
                     'X-', 'x', -1, '#dc2626',
                     'Y-', 'y', -1, '#16a34a',
                     'Y+', 'y', +1, '#16a34a', False)

            # TCP 자세
            with ui.card().classes('w-full').style(CARD):
                ui.label('TCP 자세').classes('text-gray-400 text-xs mb-3')
                # Rz: left/right rotation arrows
                with ui.row().classes('w-full justify-center gap-4 mb-2'):
                    hold_btn('Rz+', 5, +1, True, '#7c3aed', 'left')
                    hold_btn('Rz-', 5, -1, True, '#7c3aed', 'right')
                ui.label('Rz (joint6)').classes('text-center text-purple-400 text-xs w-full -mt-1 mb-1')
                # RxRy cross — joint4=Rx, joint5=Ry
                dpad('Ry+', 4, +1, '#b45309',
                     'Ry-', 4, -1, '#b45309',
                     'Rx-', 3, -1, '#b45309',
                     'Rx+', 3, +1, '#b45309', True)

            # 그리퍼
            with ui.card().classes('w-full').style(CARD):
                ui.label('그리퍼').classes('text-gray-400 text-xs mb-1')
                with ui.row().classes('w-full gap-2'):
                    ui.button('열기', on_click=lambda: _node.set_gripper(70.0)) \
                        .classes('flex-1 h-14 text-sm font-bold') \
                        .style('background:#d97706; color:white; border-radius:8px')
                    ui.button('닫기', on_click=lambda: _node.set_gripper(0.0)) \
                        .classes('flex-1 h-14 text-sm font-bold') \
                        .style('background:#4b5563; color:white; border-radius:8px')

            # 로봇 제어
            with ui.card().classes('w-full').style(CARD):
                ui.label('로봇 제어').classes('text-gray-400 text-xs mb-1')
                with ui.row().classes('w-full gap-2'):
                    ui.button('Enable', on_click=lambda: _node.enable(True)) \
                        .classes('flex-1 h-14 text-sm font-bold') \
                        .style('background:#16a34a; color:white; border-radius:8px')
                    ui.button('Disable', on_click=lambda: _node.enable(False)) \
                        .classes('flex-1 h-14 text-sm font-bold') \
                        .style('background:#374151; color:white; border-radius:8px')
                    ui.button('초기 위치', on_click=lambda: _node.go_init()) \
                        .classes('flex-1 h-14 text-sm font-bold') \
                        .style('background:#1d4ed8; color:white; border-radius:8px')

        def _refresh():
            x, y, z, rx, ry, rz, _ = state.snapshot()
            lbl['x'].text  = f'X: {x:+.1f} mm'
            lbl['y'].text  = f'Y: {y:+.1f} mm'
            lbl['z'].text  = f'Z: {z:+.1f} mm'
            lbl['rx'].text = f'Rx: {rx:+.1f}°'
            lbl['ry'].text = f'Ry: {ry:+.1f}°'
            lbl['rz'].text = f'Rz: {rz:+.1f}°'

        ui.timer(0.1, _refresh)

    ui.run(host='0.0.0.0', port=8080, title='Piper UI', reload=False)


if __name__ == '__main__':
    main()
