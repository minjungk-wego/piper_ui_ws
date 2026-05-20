#!/usr/bin/env python3
import asyncio
import json
import math
import threading
from pathlib import Path

import http.server
import socketserver
import time as _time

import rclpy
from fastapi import Request
from geometry_msgs.msg import Pose
from nicegui import app as nicegui_app
from nicegui import ui
from piper_msgs.msg import PosCmd
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool


WAYPOINTS_FILE = Path.home() / 'piper_ui_ws' / 'waypoints.json'

_frame_lock = threading.Lock()
_latest_jpeg = None


class _MJPEGHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        try:
            while True:
                with _frame_lock:
                    data = _latest_jpeg
                if data:
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n'
                    )
                    self.wfile.flush()
                _time.sleep(0.05)
        except Exception:
            pass

    def log_message(self, fmt, *args):
        pass


def _start_mjpeg_server():
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(('0.0.0.0', 8081), _MJPEGHandler) as srv:
        srv.serve_forever()


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

    def apply_delta_and_snapshot(self, axis: str, val: float):
        with self._lock:
            if axis == 'x':    self.x  += val
            elif axis == 'y':  self.y  += val
            elif axis == 'z':  self.z  += val
            elif axis == 'rx': self.rx += val
            elif axis == 'ry': self.ry += val
            elif axis == 'rz': self.rz += val
            return (self.x, self.y, self.z, self.rx, self.ry, self.rz, self.gripper)

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
_node = None


class WebUINode(Node):
    def __init__(self):
        super().__init__('piper_web_ui_node')
        self.pos_pub     = self.create_publisher(PosCmd,       'pos_cmd',               1)
        self.enable_pub  = self.create_publisher(Bool,         'enable_flag',            1)
        self.init_pub    = self.create_publisher(Bool,         'init_pos',               1)
        self.estop_pub   = self.create_publisher(Bool,         'estop',                  1)
        self.restore_pub = self.create_publisher(Bool,         'restore',                1)
        self.joint_pub   = self.create_publisher(JointState,   'joint_states',           1)
        self.create_subscription(Pose,            'end_pose',             self._on_end_pose, 1)
        self.create_subscription(JointState,      'joint_states_single',  self._on_joints,   1)
        self.create_subscription(CompressedImage, '/camera/camera/color/image_raw/compressed', self._on_camera, 1)

    def _on_end_pose(self, msg: Pose):
        state.update_from_pose(msg)

    def _on_joints(self, msg: JointState):
        state.update_joints(msg)

    def _on_camera(self, msg: CompressedImage):
        global _latest_jpeg
        with _frame_lock:
            _latest_jpeg = bytes(msg.data)

    def move(self, axis: str, delta: float):
        vals = state.apply_delta_and_snapshot(axis, delta)
        self._publish(*vals)

    def move_joint(self, joint_idx: int, delta_deg: float):
        delta_rad = delta_deg * math.pi / 180.0
        joints = state.apply_joint_delta_and_snapshot(joint_idx, delta_rad)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        msg.position = joints
        self.joint_pub.publish(msg)

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


def _load_waypoints():
    if WAYPOINTS_FILE.exists():
        try:
            return json.loads(WAYPOINTS_FILE.read_text())
        except Exception:
            return []
    return []


def _save_waypoints(wps):
    WAYPOINTS_FILE.parent.mkdir(parents=True, exist_ok=True)
    WAYPOINTS_FILE.write_text(json.dumps(wps, indent=2))


def main():
    rclpy.init()
    global _node
    _node = WebUINode()
    threading.Thread(target=rclpy.spin, args=(_node,), daemon=True).start()
    threading.Thread(target=_start_mjpeg_server, daemon=True).start()

    waypoints = _load_waypoints()

    @ui.page('/')
    def index(request: Request):
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
        all_tasks: set = set()

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
                        _node.move_joint(axis, direction * 1.0)
                        await asyncio.sleep(0.3)
                        while True:
                            _node.move_joint(axis, direction * speed_slider.value * 0.5)
                            await asyncio.sleep(0.05)
                    else:
                        _node.move(axis, direction * 1.0)
                        await asyncio.sleep(0.3)
                        while True:
                            _node.move(axis, direction * speed_slider.value * 0.5)
                            await asyncio.sleep(0.05)
                except asyncio.CancelledError:
                    pass

            async def start():
                t = asyncio.create_task(keep_moving())
                all_tasks.add(t)
                t.add_done_callback(all_tasks.discard)
                task['ref'] = t

            async def stop():
                for t in list(all_tasks):
                    t.cancel()
                all_tasks.clear()
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
                    ui.element('div').style(f'{SPC}; border-radius:50%; background:#374151')
                    hold_btn(r_lbl, r_ax, r_dir, is_rot, r_col, 'right')
                with ui.row().classes('gap-2 justify-center items-center'):
                    ui.element('div').style(SPC)
                    hold_btn(d_lbl, d_ax, d_dir, is_rot, d_col, 'down')
                    ui.element('div').style(SPC)

        with ui.column().classes('w-full max-w-md mx-auto p-4 gap-3'):
            ui.label('Piper Web UI').classes('text-2xl text-white font-bold text-center w-full')

            # 카메라
            with ui.card().classes('w-full p-0 overflow-hidden').style(
                    'background:#000; border:none; border-radius:8px'):
                _host = request.headers.get('host', '').split(':')[0]
                ui.html(
                    f'<img src="http://{_host}:8081/"'
                    ' style="width:100%; aspect-ratio:4/3; object-fit:contain; display:block; background:#000">'
                )

            # 속도 슬라이더
            with ui.card().classes('w-full').style(CARD):
                with ui.row().classes('items-center gap-3 w-full'):
                    ui.label('속도').classes('text-white text-sm w-8')
                    speed_slider = ui.slider(min=1, max=10, value=5).classes('flex-1')
                    ui.label().classes('text-white text-sm w-4 text-right') \
                        .bind_text_from(speed_slider, 'value', backward=lambda v: str(int(v)))

            # TCP 위치
            with ui.card().classes('w-full').style(CARD):
                ui.label('TCP 위치').classes('text-gray-400 text-xs mb-3')
                with ui.row().classes('w-full justify-center gap-4 mb-2'):
                    hold_btn('Z+', 'z', +1, False, '#1d4ed8', 'up')
                    hold_btn('Z-', 'z', -1, False, '#1d4ed8', 'down')
                dpad('X+', 'x', +1, '#dc2626',
                     'X-', 'x', -1, '#dc2626',
                     'Y+', 'y', +1, '#16a34a',
                     'Y-', 'y', -1, '#16a34a', False)

            # TCP 자세
            with ui.card().classes('w-full').style(CARD):
                ui.label('TCP 자세').classes('text-gray-400 text-xs mb-3')
                with ui.row().classes('w-full justify-center gap-4 mb-2'):
                    hold_btn('Rz+', 5, +1, True, '#7c3aed', 'left')
                    hold_btn('Rz-', 5, -1, True, '#7c3aed', 'right')
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

            # 로봇 제어 (Enable/Disable/초기위치 + E-STOP/복구)
            with ui.card().classes('w-full').style(CARD):
                ui.label('로봇 제어').classes('text-gray-400 text-xs mb-1')
                with ui.row().classes('w-full gap-2 mb-2'):
                    ui.button('Enable', on_click=lambda: _node.enable(True)) \
                        .classes('flex-1 h-12 text-sm font-bold') \
                        .style('background:#16a34a; color:white; border-radius:8px')
                    ui.button('Disable', on_click=lambda: _node.enable(False)) \
                        .classes('flex-1 h-12 text-sm font-bold') \
                        .style('background:#374151; color:white; border-radius:8px')
                    ui.button('초기 위치', on_click=lambda: _node.go_init()) \
                        .classes('flex-1 h-12 text-sm font-bold') \
                        .style('background:#1d4ed8; color:white; border-radius:8px')
                with ui.row().classes('w-full gap-2'):
                    async def do_estop():
                        for t in list(all_tasks):
                            t.cancel()
                        all_tasks.clear()
                        _node.estop()

                    ui.button('E-STOP', on_click=do_estop) \
                        .classes('flex-1 h-12 text-lg font-bold') \
                        .style('background-color:#dc2626 !important; color:white !important; border-radius:8px; letter-spacing:2px')
                    async def do_restore():
                        _node.restore()
                        ui.notify('복구 진행 중...', type='info', timeout=7000)
                        await asyncio.sleep(6.5)
                        ui.notify('복구 완료', type='positive')

                    ui.button('복구', on_click=do_restore) \
                        .classes('flex-1 h-12 text-sm font-bold') \
                        .style('background-color:#16a34a !important; color:white !important; border-radius:8px')

            # 웨이포인트
            with ui.expansion('웨이포인트').classes('w-full text-white').style(
                    'background:#1f2937; border:none; border-radius:8px'):
                wp_container = ui.column().classes('w-full gap-1 mb-2')

                def refresh_wp():
                    wp_container.clear()
                    with wp_container:
                        if not waypoints:
                            ui.label('저장된 웨이포인트가 없습니다') \
                                .classes('text-gray-500 text-xs py-1')
                            return
                        for i, wp in enumerate(waypoints):
                            with ui.row().classes('w-full items-center gap-2'):
                                ui.label(
                                    f'WP{i+1}  X:{wp[0]:+.0f}  Y:{wp[1]:+.0f}  Z:{wp[2]:+.0f}'
                                ).classes('text-green-400 text-xs flex-1')
                                ui.button('×', on_click=lambda idx=i: delete_wp(idx)) \
                                    .style(
                                        'background:#dc2626; color:white; border-radius:4px;'
                                        ' width:28px; height:28px; min-width:0;'
                                        ' padding:0; font-size:14px; box-shadow:none'
                                    )

                def save_wp():
                    waypoints.append(list(state.snapshot()))
                    _save_waypoints(waypoints)
                    refresh_wp()

                def delete_wp(idx):
                    waypoints.pop(idx)
                    _save_waypoints(waypoints)
                    refresh_wp()

                def clear_wp():
                    waypoints.clear()
                    _save_waypoints(waypoints)
                    refresh_wp()

                async def execute_wp():
                    delay = float(delay_input.value)
                    for wp in list(waypoints):
                        _node._publish(*wp)
                        await asyncio.sleep(delay)

                with ui.row().classes('w-full gap-2 mb-2'):
                    ui.button('현재 위치 저장', on_click=save_wp) \
                        .classes('flex-1 h-10 text-xs font-bold') \
                        .style('background:#16a34a; color:white; border-radius:8px')
                    ui.button('전체 삭제', on_click=clear_wp) \
                        .classes('h-10 text-xs font-bold') \
                        .style('background:#374151; color:white; border-radius:8px; padding:0 12px')

                refresh_wp()

                with ui.row().classes('w-full items-center gap-2 mt-2'):
                    ui.label('대기').classes('text-gray-400 text-xs')
                    delay_input = ui.number(value=2.0, min=0.5, max=30.0, step=0.5, format='%.1f') \
                        .classes('w-20') \
                        .style('background:#374151; color:white; border-radius:4px')
                    ui.label('초').classes('text-gray-400 text-xs')
                    ui.button('순서 실행', on_click=execute_wp) \
                        .classes('flex-1 h-10 text-xs font-bold') \
                        .style('background:#7c3aed; color:white; border-radius:8px')

    ui.run(host='0.0.0.0', port=8080, title='Piper UI', reload=False)


if __name__ == '__main__':
    main()
