#!/usr/bin/env python3
"""
m20_udp_node.py — DeepRobotics Lynx M20 Pro Inspection-Protocol bridge.

This node is the M20-specific TRANSPORT layer. It speaks the robot's UDP
"Inspection Communication Protocol" (manual §1) directly over the internal
switch to the AOS motion host, and exposes the robot to nav_core as GENERIC
ROS topics/services. Porting to another robot = replace THIS node; nav_core is
untouched.

Protocol (manual §1.1–1.3, verified against the V0.1.0-0 manual):
  Wire framing : 16-byte header  EB 91 EB 90 | len(LE,u16) | msgid(LE,u16) |
                 01 | 00*7        followed by an ASCII JSON body.
  Envelope     : {"PatrolDevice":{"Type":T,"Command":C,"Time":"...","Items":{}}}
  The robot streams status to whatever IP:port sends heartbeats (≥1 Hz).

RX (robot -> ROS):
  1002/4 @10Hz  MotionStatus (Roll/Pitch/Yaw, OmegaZ, LinearX/Y, Height,
                RemainMile) + MotorStatus (per-leg HipX/HipY/Knee angle[rad],
                Wheel speed[rad/s])   -> /joint_states  + telemetry
  1002/6 @2Hz   BasicStatus (MotionState, Gait, HES hard-estop,
                ControlUsageMode, Version)  -> telemetry + readiness
  1002/3        Abnormal status (ErrorList) -> telemetry + estop latch
  1002/5 @2Hz   Device state (battery ...)  -> telemetry (best-effort parse)

TX (ROS -> robot):
  /cmd_vel (Twist) -> 2/21 axis command @cmd_rate Hz, normalized [-1,1] via
                      per-axis scale + hard cap + stale-command watchdog.
                      GATED by enable_tx (default False) — RX-only until the
                      Cmd-21 path is bench-validated (test T4-T6).
  services (std_srvs/Trigger, all under this node's namespace):
    ~/set_regular_mode  1101/5 Mode 0   (axis commands require Regular mode)
    ~/stand             2/22  MotionParam 1
    ~/sit               2/22  MotionParam 4
    ~/estop             2/22  MotionParam 2  + latch (soft e-stop)
    ~/reset             clear the local e-stop latch

Motion state graph (manual §1.2.3): Power-on Damping --[Stand]--> Stand -->
Standard/Agile Motion; Soft E-Stop auto-enters Idle after 3 s. Axis commands
only take effect while standing/in-motion AND in Regular mode.

SAFETY: the hardware e-stop and the robot's own soft e-stop remain the
authoritative stops. This node additionally streams zero velocity on stale
/cmd_vel, on HES, on reported errors, and while the e-stop latch is set. The
robot-side behaviour when the 20 Hz stream simply stops is firmware-defined and
UNVERIFIED — do not rely on it (test T6).
"""

import json
import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

HEADER_MAGIC = b'\xeb\x91\xeb\x90'
PROTO_TAG = 0x01

# Usage modes (1101/5)
MODE_REGULAR, MODE_NAV, MODE_ASSIST = 0, 1, 2
# Motion params (2/22)
MP_IDLE, MP_STAND, MP_ESTOP, MP_DAMPING, MP_SIT, MP_STANDARD = 0, 1, 2, 3, 4, 6

# MotorStatus key -> URDF joint name, and whether the value is a wheel speed
# (rad/s, integrated to a position) rather than a joint angle (rad).
# NOTE: manual footnote [3] on the leg naming is garbled in translation; this
# literal Left/Right×Front/Back -> fl/fr/hl/hr mapping matches the URDF joint
# names and the sim. VERIFY against RViz on the robot (test T3): a mislabelled
# leg shows as a limb moving out of phase.
LEG_JOINTS = [
    ("fl_hipx_joint",  "LeftFrontHipX",  False),
    ("fl_hipy_joint",  "LeftFrontHipY",  False),
    ("fl_knee_joint",  "LeftFrontKnee",  False),
    ("fl_wheel_joint", "LeftFrontWheel", True),
    ("fr_hipx_joint",  "RightFrontHipX", False),
    ("fr_hipy_joint",  "RightFrontHipY", False),
    ("fr_knee_joint",  "RightFrontKnee", False),
    ("fr_wheel_joint", "RightFrontWheel", True),
    ("hl_hipx_joint",  "LeftBackHipX",   False),
    ("hl_hipy_joint",  "LeftBackHipY",   False),
    ("hl_knee_joint",  "LeftBackKnee",   False),
    ("hl_wheel_joint", "LeftBackWheel",  True),
    ("hr_hipx_joint",  "RightBackHipX",  False),
    ("hr_hipy_joint",  "RightBackHipY",  False),
    ("hr_knee_joint",  "RightBackKnee",  False),
    ("hr_wheel_joint", "RightBackWheel", True),
]


class M20UdpNode(Node):
    def __init__(self):
        super().__init__('m20_udp_node')

        # ── parameters ──────────────────────────────────────────────────
        p = self.declare_parameter
        self._robot_ip = p('robot_ip', '10.21.31.103').value
        self._robot_port = p('robot_port', 30000).value
        self._local_port = p('local_port', 30100).value   # 0 = ephemeral
        self._hb_rate = float(p('heartbeat_rate', 1.0).value)
        self._cmd_rate = float(p('cmd_rate', 20.0).value)
        self._watchdog = float(p('watchdog_timeout', 0.5).value)
        self._enable_tx = bool(p('enable_tx', False).value)
        # normalized-axis calibration: robot speed [m/s | rad/s] at |cmd|=1.0.
        # Placeholder values — replace with the T5 calibration sweep result.
        self._scale_x = float(p('axis_scale_x', 1.0).value)
        self._scale_y = float(p('axis_scale_y', 0.5).value)
        self._scale_yaw = float(p('axis_scale_yaw', 1.5).value)
        self._deadband = float(p('axis_deadband', 0.0).value)   # normalized
        self._max_norm = float(p('max_norm', 0.6).value)        # hard cap
        self._joint_topic = p('joint_states_topic', '/joint_states').value
        # False when m20_joints_node owns /joint_states from /JOINTS_DATA, which
        # is the normal deployment: that topic carries velocity and torque too,
        # and MotorStatus here has neither. Two publishers on one topic would
        # fight, and this one's angles are in the raw vendor convention.
        self._publish_js = bool(p('publish_joint_states', True).value)
        self._cmd_topic = p('cmd_vel_topic', '/cmd_vel').value
        self._set_regular_on_start = bool(p('set_regular_mode_on_start', True).value)

        # ── UDP socket (single, shared; sendto is atomic per datagram) ───
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
        try:
            # Binding a fixed local port makes a second instance fail loudly
            # instead of silently fighting over the robot at 20 Hz.
            self._sock.bind(('', self._local_port))
        except OSError as e:
            raise RuntimeError(
                f'could not bind local UDP port {self._local_port} — another '
                f'm20_udp_node already running? ({e})')
        self._sock.settimeout(1.0)
        self._dest = (self._robot_ip, self._robot_port)
        self._msg_id = 0
        self._tx_lock = threading.Lock()

        # ── state ────────────────────────────────────────────────────────
        self._last_cmd = Twist()
        self._last_cmd_t = 0.0
        self._estop_latched = False
        self._hes = 0
        self._motion_state = 0
        self._control_usage_mode = 0
        self._last_status_t = 0.0
        self._wheel_pos = {name: 0.0 for name, _, w in LEG_JOINTS if w}
        self._wheel_last_t = None

        # ── ROS interfaces ────────────────────────────────────────────────
        self._js_pub = (self.create_publisher(JointState, self._joint_topic, 10)
                        if self._publish_js else None)
        self._telem_pub = self.create_publisher(DiagnosticArray, '/m20/telemetry', 10)
        self._ready_pub = self.create_publisher(Bool, '/robot/ready', 1)
        self.create_subscription(Twist, self._cmd_topic, self._cmd_cb,
                                 QoSProfile(depth=10))

        self.create_service(Trigger, '~/set_regular_mode',
                            lambda req, res: self._svc(res, MODE_REGULAR, mode=True))
        self.create_service(Trigger, '~/stand',
                            lambda req, res: self._svc(res, MP_STAND))
        self.create_service(Trigger, '~/sit',
                            lambda req, res: self._svc(res, MP_SIT))
        self.create_service(Trigger, '~/estop', self._srv_estop)
        self.create_service(Trigger, '~/reset', self._srv_reset)

        self.create_timer(1.0 / self._hb_rate, self._heartbeat_cb)
        self.create_timer(1.0 / self._cmd_rate, self._cmd_timer_cb)

        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx.start()

        if self._set_regular_on_start:
            self._send(1101, 5, {"Mode": MODE_REGULAR})
        self.get_logger().info(
            f'm20_udp_node -> {self._robot_ip}:{self._robot_port} '
            f'(local :{self._local_port}), TX {"ENABLED" if self._enable_tx else "disabled"}')

    # ── wire helpers ──────────────────────────────────────────────────────
    def _send(self, mtype, cmd, items):
        body = json.dumps({"PatrolDevice": {
            "Type": mtype, "Command": cmd,
            "Time": "2025-01-01 00:00:00", "Items": items}}).encode('ascii')
        with self._tx_lock:
            hdr = HEADER_MAGIC + struct.pack('<HH', len(body), self._msg_id) + \
                bytes([PROTO_TAG]) + b'\x00' * 7
            self._msg_id = (self._msg_id + 1) & 0xFFFF
            try:
                self._sock.sendto(hdr + body, self._dest)
            except OSError as e:
                self.get_logger().warn(f'UDP send failed: {e}')

    def _heartbeat_cb(self):
        self._send(100, 100, {})

    # ── TX: /cmd_vel -> axis command (2/21) ──────────────────────────────
    def _cmd_cb(self, msg):
        self._last_cmd = msg
        self._last_cmd_t = time.monotonic()

    def _norm(self, value, scale):
        if scale <= 0.0:
            return 0.0
        n = max(-self._max_norm, min(self._max_norm, value / scale))
        # deadband inversion: lift small non-zero commands above the robot's
        # no-motion threshold so DWB micro-corrections actually move the robot.
        if self._deadband > 0.0 and 0.0 < abs(n) < 1.0:
            n = math.copysign(self._deadband + abs(n) * (1.0 - self._deadband), n)
        return n

    def _cmd_timer_cb(self):
        if not self._enable_tx:
            return
        fresh = (time.monotonic() - self._last_cmd_t) < self._watchdog
        if self._estop_latched or self._hes or not fresh:
            self._send(2, 21, {"X": 0.0, "Y": 0.0, "Z": 0.0,
                               "Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0})
            return
        c = self._last_cmd
        self._send(2, 21, {
            "X": self._norm(c.linear.x, self._scale_x),
            "Y": self._norm(c.linear.y, self._scale_y),
            "Z": 0.0, "Roll": 0.0, "Pitch": 0.0,
            "Yaw": self._norm(c.angular.z, self._scale_yaw)})

    # ── services ──────────────────────────────────────────────────────────
    def _svc(self, res, param, mode=False):
        if mode:
            self._send(1101, 5, {"Mode": param})
        else:
            self._send(2, 22, {"MotionParam": param})
        res.success = True
        res.message = f'sent {"mode" if mode else "motion_param"}={param}'
        return res

    def _srv_estop(self, req, res):
        self._estop_latched = True
        self._send(2, 22, {"MotionParam": MP_ESTOP})
        res.success = True
        res.message = 'soft e-stop sent + latch set (call ~/reset to clear)'
        self.get_logger().warn('E-STOP latched')
        return res

    def _srv_reset(self, req, res):
        self._estop_latched = False
        res.success = True
        res.message = 'e-stop latch cleared'
        self.get_logger().info('e-stop latch cleared')
        return res

    # ── RX: parse robot status ────────────────────────────────────────────
    def _rx_loop(self):
        while rclpy.ok():
            try:
                data, _ = self._sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) <= 16:
                continue
            body = data[16:]
            try:
                items = json.loads(body.decode('ascii', 'ignore'))['PatrolDevice']
            except (ValueError, KeyError):
                continue  # non-JSON (XML?) or malformed — see T3
            mtype, cmd = items.get('Type'), items.get('Command')
            it = items.get('Items', {})
            if mtype == 1002 and cmd == 4:
                self._on_motion_status(it)
            elif mtype == 1002 and cmd == 6:
                self._on_basic_status(it.get('BasicStatus', {}))
            elif mtype == 1002 and cmd == 3:
                self._on_error(it)
            elif mtype == 1002 and cmd == 5:
                self._on_device_state(it)

    def _on_motion_status(self, it):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        if self._js_pub is not None:
            motor = it.get('MotorStatus', {})
            js = JointState()
            js.header.stamp = now.to_msg()  # local-clock stamp (bridge guarantee)
            dt = (0.0 if self._wheel_last_t is None
                  else max(0.0, now_s - self._wheel_last_t))
            self._wheel_last_t = now_s
            for jname, key, is_wheel in LEG_JOINTS:
                val = float(motor.get(key, 0.0))
                js.name.append(jname)
                if is_wheel:
                    self._wheel_pos[jname] = math.fmod(
                        self._wheel_pos[jname] + val * dt, 2.0 * math.pi)
                    js.position.append(self._wheel_pos[jname])
                    js.velocity.append(val)
                else:
                    js.position.append(val)
                    js.velocity.append(0.0)
            self._js_pub.publish(js)

        ms = it.get('MotionStatus', {})
        self._last_status_t = time.monotonic()
        self._publish_telem({
            'roll': ms.get('Roll'), 'pitch': ms.get('Pitch'), 'yaw': ms.get('Yaw'),
            'omega_z': ms.get('OmegaZ'), 'linear_x': ms.get('LinearX'),
            'linear_y': ms.get('LinearY'), 'height': ms.get('Height'),
            'remain_mile_km': ms.get('RemainMile')})

    def _on_basic_status(self, bs):
        self._hes = int(bs.get('HES', 0) or 0)
        self._motion_state = int(bs.get('MotionState', 0) or 0)
        self._control_usage_mode = int(bs.get('ControlUsageMode', 0) or 0)
        self._last_status_t = time.monotonic()
        if self._hes:
            self._estop_latched = True
        self._publish_telem({
            'motion_state': self._motion_state, 'gait': bs.get('Gait'),
            'hes': self._hes, 'control_usage_mode': self._control_usage_mode,
            'version': bs.get('Version')})

    def _on_error(self, it):
        errs = it.get('ErrorList', [])
        if errs:
            self._estop_latched = True
            self.get_logger().error(f'robot error report: {errs}')
            self._publish_telem({'error_list': json.dumps(errs)})

    def _on_device_state(self, it):
        bat = it.get('BatteryStatus', it.get('Battery', {}))
        if isinstance(bat, dict):
            self._publish_telem({'battery_level': bat.get('Level'),
                                 'battery_voltage': bat.get('Voltage')})

    # ── telemetry + readiness ─────────────────────────────────────────────
    def _publish_telem(self, kv):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus(name='m20_udp_node', hardware_id=self._robot_ip)
        fresh = (time.monotonic() - self._last_status_t) < 2.0
        ready = fresh and not self._hes and not self._estop_latched and \
            self._motion_state in (MP_STAND, MP_STANDARD, 8)
        st.level = DiagnosticStatus.OK if ready else DiagnosticStatus.WARN
        st.message = 'ready' if ready else 'not ready'
        st.values = [KeyValue(key=k, value=str(v)) for k, v in kv.items()
                     if v is not None]
        st.values.append(KeyValue(key='estop_latched', value=str(self._estop_latched)))
        st.values.append(KeyValue(key='tx_enabled', value=str(self._enable_tx)))
        arr.status.append(st)
        self._telem_pub.publish(arr)
        self._ready_pub.publish(Bool(data=ready))


def main():
    rclpy.init()
    node = M20UdpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
