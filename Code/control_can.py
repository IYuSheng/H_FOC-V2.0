#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual-joint arm real-time monitor/control over CAN FD.

Protocol mapping follows current Core/Inc/fdcan.h:
- CAN ID (11-bit): (src << 8) | (dst << 4) | msg_type
- STATUS frame payload (64B): joint_status_t structure
- CONTROL frame payload (64B): joint_control_t structure
"""

import argparse
import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

try:
    # 优先使用 PyQt5；若本机未安装，则兼容 PySide6（API 基本一致）
    from PyQt5 import QtCore, QtWidgets, QtGui
except Exception:
    from PySide6 import QtCore, QtWidgets, QtGui

try:
    import can
except Exception:
    can = None


# ==================== Config ====================
class Config:
    # CAN FD transport
    CAN_INTERFACE = "slcan"
    CAN_CHANNEL = "COM21"
    SLCAN_TTY_BAUDRATE = 4_000_000
    SLCAN_TIMEOUT_SEC = 0.0
    
    # CAN FD 波特率配置（必须与你的STM32配置匹配）
    # 仲裁段: 1Mbps (与你的C代码 CAN_NOMINAL_PRESCALER等匹配)
    CAN_NOMINAL_BITRATE = 1_000_000
    # 数据段: 2Mbps (与你的C代码 CAN_DATA_PRESCALER等匹配)
    CAN_DATA_BITRATE = 2_000_000
    
    ENABLE_CONTROL_TX = True

    # Node IDs
    MASTER_ID = 0
    MOTOR1_ID = 1
    MOTOR2_ID = 2
    BROADCAST_ID = 0x0F

    # Message types (必须与你的fdcan.h一致)
    MSG_TYPE_STATUS = 0x0
    MSG_TYPE_CONTROL = 0x1

    # Control mode (必须匹配 fdcan.h)
    CONTROL_MODE_TORQUE = 0
    CONTROL_MODE_POSITION = 1
    CONTROL_MODE = CONTROL_MODE_TORQUE

    # 定点数缩放比例（必须匹配你的STM32代码）
    SCALE_32 = 1e-5  # 32位定点数缩放
    SCALE_16 = 1e-2  # 16位定点数缩放

    # Loop and timeout
    LOOP_DT = 0.001
    RX_STALE_SEC = 0.05
    RX_THREAD_RECV_TIMEOUT_SEC = 0.001
    UI_UPDATE_MS = 50
    # 位置控制发送优化：目标不变且逆解无明显改进时，不重复发送
    POS_TARGET_DEADBAND_M = 0.0005
    POS_CMD_DEADBAND_DEG = 0.05
    POS_IK_IMPROVE_DEG = 0.2
    POS_KEEPALIVE_SEC = 0.5

    # Kinematics / controller
    L1, L2 = 0.07748, 0.0625
    ZERO1, ZERO2 = -53.7, 172.5
    KPX, KDX = 10.0, 1.0
    KPY, KDY = 10.0, 1.0
    F_MAX, IQ_MAX, KT = 20.0, 8.0, 0.095
    XLIM, YLIM = (-0.15, 0.15), (-0.18, 0.05)
    # 关节1角度限制（deg）
    J1_MIN_DEG, J1_MAX_DEG = -170.0, 70.0
    # 手动关节控制参数
    MANUAL_SPEED_MIN_DEG_S = 5.0
    MANUAL_SPEED_MAX_DEG_S = 300.0
    J2_MANUAL_MIN_DEG, J2_MANUAL_MAX_DEG = -360.0, 360.0


@dataclass
class Joint:
    th1: float = 0.0
    w1: float = 0.0
    th2: float = 0.0
    w2: float = 0.0


@dataclass
class Cart:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0


@dataclass
class Ctrl:
    fx: float = 0.0
    fy: float = 0.0
    iq1: float = 0.0
    iq2: float = 0.0


@dataclass
class MotorStatus:
    th: float = 0.0
    w: float = 0.0
    cur: float = 0.0
    temp: float = 0.0
    status: int = 0
    error: int = 0
    t_rx: float = 0.0


# ==================== Math / protocol helpers ====================
def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def can_id_make(src: int, dst: int, msg_type: int) -> int:
    """生成CAN ID: (src << 8) | (dst << 4) | msg_type"""
    return ((src & 0x0F) << 8) | ((dst & 0x0F) << 4) | (msg_type & 0x0F)


def can_id_get_src(can_id: int) -> int:
    return (can_id >> 8) & 0x0F


def can_id_get_dst(can_id: int) -> int:
    return (can_id >> 4) & 0x0F


def can_id_get_type(can_id: int) -> int:
    return can_id & 0x0F


def float_to_fix32(v: float) -> int:
    """32位定点数转换"""
    raw = int(v / Config.SCALE_32)
    return max(-2147483648, min(2147483647, raw))


def float_to_fix16(v: float) -> int:
    """16位定点数转换"""
    raw = int(v / Config.SCALE_16)
    return max(-32768, min(32767, raw))


def fix32_to_float(v: int) -> float:
    return float(v) * Config.SCALE_32


def fix16_to_float(v: int) -> float:
    return float(v) * Config.SCALE_16


def control_mode_name(mode: int) -> str:
    if mode == Config.CONTROL_MODE_TORQUE:
        return "torque"
    if mode == Config.CONTROL_MODE_POSITION:
        return "position"
    return f"unknown({mode})"


# ==================== Kinematics & controller ====================
def kinematics(j: Joint):
    t1 = math.radians(j.th1 - Config.ZERO1)
    t2 = math.radians(j.th2 - Config.ZERO2)
    t12 = t1 + t2
    s1, c1 = math.sin(t1), math.cos(t1)
    s12, c12 = math.sin(t12), math.cos(t12)

    x = -(Config.L1 * s1 + Config.L2 * s12)
    y = -(Config.L1 * c1 + Config.L2 * c12)

    j11 = -(Config.L1 * c1 + Config.L2 * c12)
    j12 = -(Config.L2 * c12)
    j21 = Config.L1 * s1 + Config.L2 * s12
    j22 = Config.L2 * s12

    d1, d2 = math.radians(j.w1), math.radians(j.w2)
    vx = j11 * d1 + j12 * d2
    vy = j21 * d1 + j22 * d2

    x1, y1 = -Config.L1 * s1, -Config.L1 * c1
    return Cart(x, y, vx, vy), (0.0, 0.0, x1, y1, x, y), (j11, j12, j21, j22)


def pd_control(c: Cart, tgt: Cart, jac: Tuple[float, float, float, float]):
    fx = Config.KPX * (tgt.x - c.x) + Config.KDX * (tgt.vx - c.vx)
    fy = Config.KPY * (tgt.y - c.y) + Config.KDY * (tgt.vy - c.vy)
    fx = clamp(fx, -Config.F_MAX, Config.F_MAX)
    fy = clamp(fy, -Config.F_MAX, Config.F_MAX)

    j11, j12, j21, j22 = jac
    tau1 = j11 * fx + j21 * fy
    tau2 = j12 * fx + j22 * fy

    iq1 = clamp(tau1 / Config.KT, -Config.IQ_MAX, Config.IQ_MAX)
    iq2 = clamp(tau2 / Config.KT, -Config.IQ_MAX, Config.IQ_MAX)
    return Ctrl(fx, fy, iq1, iq2)


def _wrap_deg_err(err_deg: float) -> float:
    """把角度误差约束到[-180, 180]，用于逆解分支选择。"""
    while err_deg > 180.0:
        err_deg -= 360.0
    while err_deg < -180.0:
        err_deg += 360.0
    return err_deg


def _unwrap_deg_to_ref(angle_deg: float, ref_deg: float) -> float:
    """
    将角度映射到最接近ref_deg的等效角，避免跨±180产生大跳变。
    例如：ref=179, angle=-179 -> 输出181（而不是-179）。
    """
    return ref_deg + _wrap_deg_err(angle_deg - ref_deg)


def inverse_kinematics_xy(x: float, y: float, ref_joint: Joint) -> Optional[Joint]:
    """
    根据末端目标(x,y)求关节角逆解，返回最接近当前关节(ref_joint)的分支。
    坐标系与 kinematics() 保持一致。
    """
    l1, l2 = Config.L1, Config.L2

    # 将本工程坐标映射到标准2R平面形式：
    # x_std = -y, y_std = -x
    x_std = -y
    y_std = -x
    r2 = x_std * x_std + y_std * y_std
    r = math.sqrt(r2)

    # 可达性判断
    if r > (l1 + l2) or r < abs(l1 - l2):
        return None

    c2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    c2 = clamp(c2, -1.0, 1.0)
    s2_abs = math.sqrt(max(0.0, 1.0 - c2 * c2))

    candidates = []
    for s2 in (s2_abs, -s2_abs):
        t2 = math.atan2(s2, c2)
        t1 = math.atan2(y_std, x_std) - math.atan2(l2 * s2, l1 + l2 * c2)

        th1_deg = math.degrees(t1) + Config.ZERO1
        th2_deg = math.degrees(t2) + Config.ZERO2
        # 逆解角度做"最接近当前反馈"的等效映射，减少不必要大角度反转。
        th1_deg = _unwrap_deg_to_ref(th1_deg, ref_joint.th1)
        th2_deg = _unwrap_deg_to_ref(th2_deg, ref_joint.th2)
        # 关节1限位过滤：只保留满足[-170,70]deg的逆解分支
        if Config.J1_MIN_DEG <= th1_deg <= Config.J1_MAX_DEG:
            candidates.append(Joint(th1=th1_deg, w1=0.0, th2=th2_deg, w2=0.0))

    if not candidates:
        return None

    # 选与当前关节角最接近的分支，避免肘形态来回跳变。
    best = min(
        candidates,
        key=lambda q: abs(_wrap_deg_err(q.th1 - ref_joint.th1))
        + abs(_wrap_deg_err(q.th2 - ref_joint.th2)),
    )
    return best


# ==================== CAN FD communication ====================
class CANComm:
    """
    CAN FD通信类
    支持64字节payload，仲裁段1Mbps，数据段2Mbps
    """

    def __init__(self):
        self.bus = None
        self.lock = threading.Lock()
        self.state: Dict[int, MotorStatus] = {}
        self.rx_frames = 0
        self.tx_frames = 0
        self.err_frames = 0
        self._warned_no_fb = False
        self._rx_running = False
        self._rx_thread: Optional[threading.Thread] = None

    def _build_status_filters(self):
        # Match STATUS from motor1/motor2, ignore dst nibble.
        mask_src_and_type = 0xF0F
        return [
            {
                "can_id": can_id_make(Config.MOTOR1_ID, 0x0, Config.MSG_TYPE_STATUS),
                "can_mask": mask_src_and_type,
                "extended": False,
            },
            {
                "can_id": can_id_make(Config.MOTOR2_ID, 0x0, Config.MSG_TYPE_STATUS),
                "can_mask": mask_src_and_type,
                "extended": False,
            },
        ]

    def _start_rx_thread(self):
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True, name="can-rx")
        self._rx_thread.start()

    def _rx_worker(self):
        while self._rx_running:
            bus = self.bus
            if bus is None:
                break
            try:
                msg = bus.recv(timeout=Config.RX_THREAD_RECV_TIMEOUT_SEC)
            except Exception:
                if self._rx_running:
                    time.sleep(0.001)
                continue

            if msg is None:
                continue

            self._handle_msg(msg)

            # Drain until empty: receive fully, but state keeps only latest sample per motor.
            while True:
                try:
                    msg = bus.recv(timeout=0.0)
                except Exception:
                    break
                if msg is None:
                    break
                self._handle_msg(msg)

    def open(self) -> bool:
        if can is None:
            print("[ERROR] python-can not installed. Install with: pip install python-can")
            return False

        kwargs = {
            "interface": "slcan",
            "channel": Config.CAN_CHANNEL,
            "bitrate": Config.CAN_NOMINAL_BITRATE,
            "tty_baudrate": Config.SLCAN_TTY_BAUDRATE,
            "timeout": Config.SLCAN_TIMEOUT_SEC,
        }

        try:
            self.bus = can.Bus(**kwargs)
            if not hasattr(self.bus, "set_bitrate"):
                raise RuntimeError("slcan backend has no set_bitrate() method.")
            self.bus.set_bitrate(Config.CAN_NOMINAL_BITRATE, Config.CAN_DATA_BITRATE)
            try:
                self.bus.set_filters(self._build_status_filters())
            except Exception:
                # Some slcan firmware/backends ignore filters; fallback to software parse.
                pass
            self._start_rx_thread()
            print(f"[CAN FD] open ok: {kwargs}")
            print(
                f"[PROTO] SLCAN FD, arbitration={Config.CAN_NOMINAL_BITRATE/1e6:.1f}Mbps, "
                f"data={Config.CAN_DATA_BITRATE/1e6:.1f}Mbps, tty={Config.SLCAN_TTY_BAUDRATE/1e6:.1f}Mbps"
            )
            if not Config.ENABLE_CONTROL_TX:
                print("[NOTE] TX control disabled, running in monitor-only mode.")
            return True
        except Exception as exc:
            if self.bus is not None:
                try:
                    self.bus.shutdown()
                except Exception:
                    pass
                self.bus = None
            print(f"[ERROR] CAN FD open failed: {exc}")
            print(
                "[HINT] Current build is slcan-only. Ensure firmware supports FD set_bitrate(1M,2M)."
            )
            return False

    def close(self):
        self._rx_running = False
        rx_thread = self._rx_thread
        self._rx_thread = None
        if rx_thread is not None and rx_thread.is_alive():
            rx_thread.join(timeout=0.1)

        bus = self.bus
        self.bus = None
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass

    def _handle_msg(self, msg):
        can_id = msg.arbitration_id
        msg_type = can_id_get_type(can_id)
        src = can_id_get_src(can_id)
        _dst = can_id_get_dst(can_id)

        if msg_type != Config.MSG_TYPE_STATUS:
            return
        if src not in (Config.MOTOR1_ID, Config.MOTOR2_ID):
            return
        
        # CAN FD支持最大64字节，但你的status结构体可能更小
        # 这里假设至少8字节有效数据（position + velocity）
        if len(msg.data) < 8:
            return

        try:
            # 解析前8字节为position和velocity（32位定点数）
            p_fix, v_fix = struct.unpack("<ii", bytes(msg.data[:8]))
        except struct.error:
            return

        now = time.perf_counter()
        with self.lock:
            self.state[src] = MotorStatus(
                th=fix32_to_float(p_fix),
                w=fix32_to_float(v_fix),
                cur=0.0,
                temp=0.0,
                status=0,
                error=0,
                t_rx=now,
            )
            self.rx_frames += 1

    def get_motor_status(self, motor_id: int) -> Optional[MotorStatus]:
        with self.lock:
            s = self.state.get(motor_id)
            if s is None:
                return None
            return MotorStatus(
                th=s.th,
                w=s.w,
                cur=s.cur,
                temp=s.temp,
                status=s.status,
                error=s.error,
                t_rx=s.t_rx,
            )

    def read_joint(self) -> Optional[Joint]:
        with self.lock:
            s1 = self.state.get(Config.MOTOR1_ID)
            s2 = self.state.get(Config.MOTOR2_ID)
        if s1 is None or s2 is None:
            return None

        now = time.perf_counter()
        if (now - s1.t_rx) > Config.RX_STALE_SEC or (now - s2.t_rx) > Config.RX_STALE_SEC:
            return None

        return Joint(th1=s1.th, w1=s1.w, th2=s2.th, w2=s2.w)

    def _send_control(self, dst_id: int, target_pos_deg: float, target_cur_a: float, control_mode: int) -> bool:
        if self.bus is None or can is None:
            return False

        target_cur_a = clamp(target_cur_a, -Config.IQ_MAX, Config.IQ_MAX)
        
        # 构建控制帧payload（CAN FD支持64字节，但控制指令可能只需要部分）
        # 根据你的joint_control_t结构体调整
        payload = struct.pack(
            "<ihh",
            float_to_fix32(target_pos_deg),       # target_pos (32位定点)
            float_to_fix16(target_cur_a),         # target_cur (16位定点)
            int(control_mode),                    # control_mode (16位)
        )
        
        can_id = can_id_make(Config.MASTER_ID, dst_id, Config.MSG_TYPE_CONTROL)
        
        try:
            # CAN FD消息：启用is_fd和bitrate_switch
            msg = can.Message(
                arbitration_id=can_id,
                is_extended_id=False,
                data=payload,
                is_fd=True,  # 标记为CAN FD帧
                bitrate_switch=True,  # 启用比特率切换（BRS）
            )
            self.bus.send(msg, timeout=0.0)
            self.tx_frames += 1
            return True
        except Exception as e:
            self.err_frames += 1
            return False

    def write(self, j: Joint, iq1: float, iq2: float) -> bool:
        if not Config.ENABLE_CONTROL_TX:
            return True

        ok1 = self._send_control(Config.MOTOR1_ID, j.th1, iq1, Config.CONTROL_MODE)
        ok2 = self._send_control(Config.MOTOR2_ID, j.th2, iq2, Config.CONTROL_MODE)
        return ok1 and ok2


# ==================== Visualization ====================
class ArmCanvas(QtWidgets.QWidget):
    """低开销2D工作区绘制控件（纯QPainter，橙白示教风格）。"""

    def __init__(self, click_callback):
        super().__init__()
        self._click_callback = click_callback
        self._coords = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._target = (0.0, 0.0)
        self.setMinimumSize(560, 560)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def set_scene(self, coords, target_xy):
        self._coords = coords
        self._target = target_xy
        self.update()

    def _plot_rect(self) -> QtCore.QRectF:
        margin = 26.0
        side = max(20.0, min(self.width() - 2 * margin, self.height() - 2 * margin))
        return QtCore.QRectF((self.width() - side) * 0.5, (self.height() - side) * 0.5, side, side)

    def _transform(self):
        rect = self._plot_rect()
        x_min, x_max = Config.XLIM
        y_min, y_max = Config.YLIM
        sx = rect.width() / (x_max - x_min)
        sy = rect.height() / (y_max - y_min)
        scale = min(sx, sy)
        draw_w = (x_max - x_min) * scale
        draw_h = (y_max - y_min) * scale
        left = rect.left() + (rect.width() - draw_w) * 0.5
        top = rect.top() + (rect.height() - draw_h) * 0.5
        area = QtCore.QRectF(left, top, draw_w, draw_h)
        ox = left - x_min * scale
        oy = top + y_max * scale
        return area, ox, oy, scale

    def _world_to_screen(self, x: float, y: float):
        _area, ox, oy, scale = self._transform()
        return QtCore.QPointF(ox + x * scale, oy - y * scale)

    def _screen_to_world(self, px: float, py: float):
        _area, ox, oy, scale = self._transform()
        return (px - ox) / scale, (oy - py) / scale

    def mousePressEvent(self, event):
        left_button = getattr(QtCore.Qt, "LeftButton", None)
        if left_button is None:
            left_button = QtCore.Qt.MouseButton.LeftButton
        if event.button() != left_button:
            return
        if hasattr(event, "position"):
            pos = event.position()
            px, py = pos.x(), pos.y()
        else:
            pos = event.pos()
            px, py = float(pos.x()), float(pos.y())
        x, y = self._screen_to_world(px, py)
        if x < Config.XLIM[0] or x > Config.XLIM[1] or y < Config.YLIM[0] or y > Config.YLIM[1]:
            return
        self._click_callback(x, y)

    def paintEvent(self, _event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        p.fillRect(self.rect(), QtGui.QColor("#faf8f5"))

        area, _ox, _oy, scale = self._transform()
        x_min, x_max = Config.XLIM
        y_min, y_max = Config.YLIM
        
        # 绘制工作区背景
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor("#ffffff"))
        p.drawRoundedRect(area, 16, 16)
        
        # 外边框
        p.setPen(QtGui.QPen(QtGui.QColor("#e8ddd4"), 1.5))
        p.setBrush(QtCore.Qt.NoBrush)
        p.drawRoundedRect(area, 16, 16)

        # 网格线 - 更柔和的色调
        p.setPen(QtGui.QPen(QtGui.QColor("#f0e6dd"), 1))
        grid_count = 6
        for i in range(grid_count + 1):
            t = i / grid_count
            xr = x_min + (x_max - x_min) * t
            yr = y_min + (y_max - y_min) * t
            pa = self._world_to_screen(xr, y_min)
            pb = self._world_to_screen(xr, y_max)
            p.drawLine(pa, pb)
            pc = self._world_to_screen(x_min, yr)
            pd = self._world_to_screen(x_max, yr)
            p.drawLine(pc, pd)

        # 坐标零轴 - 深橙色
        p.setPen(QtGui.QPen(QtGui.QColor("#c45c26"), 2))
        p.drawLine(self._world_to_screen(0.0, y_min), self._world_to_screen(0.0, y_max))
        p.drawLine(self._world_to_screen(x_min, 0.0), self._world_to_screen(x_max, 0.0))

        # 工作空间示意（外/内可达圆）
        center = self._world_to_screen(0.0, 0.0)
        outer_r = (Config.L1 + Config.L2) * scale
        inner_r = abs(Config.L1 - Config.L2) * scale
        p.setPen(QtGui.QPen(QtGui.QColor("#e8a87c"), 1.5, QtCore.Qt.DashLine))
        p.setBrush(QtCore.Qt.NoBrush)
        p.drawEllipse(center, outer_r, outer_r)
        p.drawEllipse(center, inner_r, inner_r)

        # 机械臂连杆与关节 - 现代配色
        x0, y0, x1, y1, x2, y2 = self._coords
        p0 = self._world_to_screen(x0, y0)
        p1 = self._world_to_screen(x1, y1)
        p2 = self._world_to_screen(x2, y2)

        # 连杆1 - 深橙
        p.setPen(QtGui.QPen(QtGui.QColor("#d4652a"), 10, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        p.drawLine(p0, p1)
        # 连杆2 - 亮橙
        p.setPen(QtGui.QPen(QtGui.QColor("#e67e45"), 10, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        p.drawLine(p1, p2)

        # 关节点
        p.setPen(QtCore.Qt.NoPen)
        # 基座 - 深灰
        p.setBrush(QtGui.QColor("#5a4a42"))
        p.drawEllipse(p0, 8, 8)
        # 中间关节 - 中橙
        p.setBrush(QtGui.QColor("#c45c26"))
        p.drawEllipse(p1, 8, 8)
        # 末端 - 亮橙带白边
        p.setBrush(QtGui.QColor("#e67e45"))
        p.drawEllipse(p2, 9, 9)
        p.setBrush(QtGui.QColor("#ffffff"))
        p.drawEllipse(p2, 4, 4)

        # 目标点 - 带十字准星的圆环
        tx, ty = self._target
        tp = self._world_to_screen(tx, ty)
        
        # 外圈
        p.setPen(QtGui.QPen(QtGui.QColor("#c45c26"), 2.5))
        p.setBrush(QtCore.Qt.NoBrush)
        p.drawEllipse(tp, 10, 10)
        # 十字
        p.drawLine(QtCore.QPointF(tp.x() - 12, tp.y()), QtCore.QPointF(tp.x() + 12, tp.y()))
        p.drawLine(QtCore.QPointF(tp.x(), tp.y() - 12), QtCore.QPointF(tp.x(), tp.y() + 12))
        # 中心点
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor("#c45c26"))
        p.drawEllipse(tp, 3, 3)


class CompactSlider(QtWidgets.QWidget):
    """紧凑型滑块组件，带数值显示"""
    def __init__(self, label: str, min_val: float, max_val: float, default: float, 
                 decimals: int = 1, suffix: str = "", parent=None):
        super().__init__(parent)
        
        self.min_val = min_val
        self.max_val = max_val
        self.decimals = decimals
        self.suffix = suffix
        
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        
        # 标签
        self.label = QtWidgets.QLabel(label)
        self.label.setStyleSheet("color: #5a4a42; font-size: 12px; font-weight: 500;")
        self.label.setFixedWidth(60)
        
        # 滑块
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 6px;
                background: #e8ddd4;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #e67e45;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                width: 16px;
                height: 16px;
                margin: -5px 0;
                background: #ffffff;
                border: 2px solid #c45c26;
                border-radius: 8px;
            }
            QSlider::handle:horizontal:hover {
                background: #fff5f0;
                border: 2px solid #d4652a;
            }
        """)
        
        # 数值显示
        self.value_label = QtWidgets.QLabel()
        self.value_label.setStyleSheet("color: #c45c26; font-size: 12px; font-weight: 600; min-width: 55px;")
        self.value_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        
        layout.addWidget(self.label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.value_label)
        
        # 初始化
        self.set_value(default)
        self.slider.valueChanged.connect(self._on_slider_changed)
        
    def _on_slider_changed(self, val):
        real_val = self.min_val + (val / 1000.0) * (self.max_val - self.min_val)
        self.value_label.setText(f"{real_val:.{self.decimals}f}{self.suffix}")
        self.valueChanged.emit(real_val)
        
    def set_value(self, val):
        val = clamp(val, self.min_val, self.max_val)
        slider_val = int((val - self.min_val) / (self.max_val - self.min_val) * 1000)
        self.slider.setValue(slider_val)
        self.value_label.setText(f"{val:.{self.decimals}f}{self.suffix}")
        
    def get_value(self):
        val = self.min_val + (self.slider.value() / 1000.0) * (self.max_val - self.min_val)
        return val
        
    # 自定义信号
    valueChanged = QtCore.pyqtSignal(float) if hasattr(QtCore, 'pyqtSignal') else QtCore.Signal(float)


class Visualizer(QtWidgets.QMainWindow):
    """示教器风格界面：工作区 + 角落浮层状态 + 底部关节控制。"""

    def __init__(self, ctrl: "Controller"):
        super().__init__()
        self.ctrl = ctrl
        self._manual_ui_sync = False
        self._init_window()
        self._init_timer()

    def _init_window(self):
        self.setWindowTitle("Arm Teach Pendant")
        self.resize(1100, 780)
        
        # 现代橙白配色样式表
        self.setStyleSheet("""
            QMainWindow { 
                background: #faf8f5; 
            }
            QFrame#Panel {
                background: #ffffff;
                border: 1px solid #e8ddd4;
                border-radius: 16px;
            }
            QFrame#Overlay {
                background: rgba(255, 255, 255, 245);
                border: 1px solid #e8ddd4;
                border-radius: 12px;
            }
            QLabel#Header {
                color: #5a4a42;
                font-size: 15px;
                font-weight: 600;
            }
            QLabel#OverlayTitle {
                color: #8b7355;
                font-size: 11px;
                font-weight: 500;
            }
            QLabel#OverlayValue {
                color: #5a4a42;
                font-size: 12px;
                font-weight: 600;
                font-family: 'SF Mono', 'Consolas', monospace;
            }
            QLabel#Hint {
                color: #8b7355;
                font-size: 11px;
                padding: 4px 0;
            }
            QCheckBox {
                color: #5a4a42;
                font-size: 12px;
                font-weight: 500;
                spacing: 6px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border-radius: 4px;
                border: 2px solid #d4c4b8;
            }
            QCheckBox::indicator:checked {
                background: #e67e45;
                border: 2px solid #c45c26;
            }
            QCheckBox::indicator:checked::after {
                content: "✓";
                color: white;
                font-size: 12px;
                font-weight: bold;
            }
            QLabel#StatusBadge {
                background: #f5ebe3;
                color: #c45c26;
                border-radius: 6px;
                padding: 2px 8px;
                font-size: 11px;
                font-weight: 600;
            }
            QPushButton {
                background: #e67e45;
                color: white;
                border: none;
                border-radius: 8px;
                padding: 8px 16px;
                font-size: 12px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #d4652a;
            }
            QPushButton:pressed {
                background: #c45c26;
            }
        """)

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        # 主工作区面板
        work_panel = QtWidgets.QFrame()
        work_panel.setObjectName("Panel")
        left_layout = QtWidgets.QVBoxLayout(work_panel)
        left_layout.setContentsMargins(12, 12, 12, 12)
        left_layout.setSpacing(8)
        
        # 标题栏
        header_layout = QtWidgets.QHBoxLayout()
        left_title = QtWidgets.QLabel("工作空间")
        left_title.setObjectName("Header")
        header_layout.addWidget(left_title)
        
        # 状态标签
        self.status_badge = QtWidgets.QLabel("CAN FD")
        self.status_badge.setObjectName("StatusBadge")
        header_layout.addStretch()
        header_layout.addWidget(self.status_badge)
        left_layout.addLayout(header_layout)

        self.arm_canvas = ArmCanvas(self._on_canvas_target_click)
        left_layout.addWidget(self.arm_canvas, 1)

        # 状态浮层
        self.overlay = QtWidgets.QFrame(self.arm_canvas)
        self.overlay.setObjectName("Overlay")
        ov_layout = QtWidgets.QGridLayout(self.overlay)
        ov_layout.setContentsMargins(12, 10, 12, 10)
        ov_layout.setHorizontalSpacing(16)
        ov_layout.setVerticalSpacing(6)

        # 创建状态标签对
        def create_status_pair(title_text):
            title = QtWidgets.QLabel(title_text)
            title.setObjectName("OverlayTitle")
            value = QtWidgets.QLabel("--")
            value.setObjectName("OverlayValue")
            return title, value

        self.ov_target_t, self.ov_target_v = create_status_pair("目标位置")
        self.ov_cart_t, self.ov_cart_v = create_status_pair("当前位置")
        self.ov_cmd_t, self.ov_cmd_v = create_status_pair("指令关节")
        self.ov_jnt_t, self.ov_jnt_v = create_status_pair("关节状态")
        self.ov_runtime_t, self.ov_runtime_v = create_status_pair("运行状态")

        ov_layout.addWidget(self.ov_target_t, 0, 0)
        ov_layout.addWidget(self.ov_target_v, 0, 1)
        ov_layout.addWidget(self.ov_cart_t, 1, 0)
        ov_layout.addWidget(self.ov_cart_v, 1, 1)
        ov_layout.addWidget(self.ov_cmd_t, 2, 0)
        ov_layout.addWidget(self.ov_cmd_v, 2, 1)
        ov_layout.addWidget(self.ov_jnt_t, 3, 0)
        ov_layout.addWidget(self.ov_jnt_v, 3, 1)
        ov_layout.addWidget(self.ov_runtime_t, 4, 0)
        ov_layout.addWidget(self.ov_runtime_v, 4, 1)
        
        self.overlay.setFixedSize(280, 150)
        self._place_overlay()

        root.addWidget(work_panel, 1)

        # 底部控制面板 - 水平紧凑布局
        ctrl_panel = QtWidgets.QFrame()
        ctrl_panel.setObjectName("Panel")
        ctrl_layout = QtWidgets.QHBoxLayout(ctrl_panel)
        ctrl_layout.setContentsMargins(16, 12, 16, 12)
        ctrl_layout.setSpacing(20)

        # 左侧：关节控制组
        joint_group = QtWidgets.QWidget()
        joint_layout = QtWidgets.QVBoxLayout(joint_group)
        joint_layout.setContentsMargins(0, 0, 0, 0)
        joint_layout.setSpacing(8)

        # 关节1行
        j1_row = QtWidgets.QHBoxLayout()
        self.cb_j1 = QtWidgets.QCheckBox("J1 手动")
        self.cb_j1.setStyleSheet("QCheckBox { min-width: 70px; }")
        self.sl_j1 = CompactSlider("", Config.J1_MIN_DEG, Config.J1_MAX_DEG, 0.0, decimals=1, suffix="°")
        self.sl_j1.setFixedWidth(220)
        j1_row.addWidget(self.cb_j1)
        j1_row.addWidget(self.sl_j1)
        j1_row.addStretch()
        joint_layout.addLayout(j1_row)

        # 关节2行
        j2_row = QtWidgets.QHBoxLayout()
        self.cb_j2 = QtWidgets.QCheckBox("J2 手动")
        self.cb_j2.setStyleSheet("QCheckBox { min-width: 70px; }")
        self.sl_j2 = CompactSlider("", Config.J2_MANUAL_MIN_DEG, Config.J2_MANUAL_MAX_DEG, 0.0, decimals=1, suffix="°")
        self.sl_j2.setFixedWidth(220)
        j2_row.addWidget(self.cb_j2)
        j2_row.addWidget(self.sl_j2)
        j2_row.addStretch()
        joint_layout.addLayout(j2_row)

        ctrl_layout.addWidget(joint_group)

        # 分隔线
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.VLine)
        line.setStyleSheet("color: #e8ddd4;")
        line.setFixedHeight(60)
        ctrl_layout.addWidget(line)

        # 右侧：速度控制
        speed_group = QtWidgets.QWidget()
        speed_layout = QtWidgets.QVBoxLayout(speed_group)
        speed_layout.setContentsMargins(0, 0, 0, 0)
        speed_layout.setSpacing(8)

        speed_row = QtWidgets.QHBoxLayout()
        speed_label = QtWidgets.QLabel("变化速度")
        speed_label.setStyleSheet("color: #5a4a42; font-size: 12px; font-weight: 500; min-width: 60px;")
        self.sl_speed = CompactSlider("", Config.MANUAL_SPEED_MIN_DEG_S, Config.MANUAL_SPEED_MAX_DEG_S, 
                                      90.0, decimals=0, suffix="°/s")
        self.sl_speed.setFixedWidth(180)
        speed_row.addWidget(speed_label)
        speed_row.addWidget(self.sl_speed)
        speed_row.addStretch()
        speed_layout.addLayout(speed_row)

        hint = QtWidgets.QLabel("勾选手动模式后可拖动滑块控制单关节")
        hint.setObjectName("Hint")
        speed_layout.addWidget(hint)

        ctrl_layout.addWidget(speed_group)
        ctrl_layout.addStretch()

        root.addWidget(ctrl_panel, 0)

        # 信号连接
        self.cb_j1.toggled.connect(lambda checked: self.ctrl.set_manual_joint_enable(1, checked))
        self.cb_j2.toggled.connect(lambda checked: self.ctrl.set_manual_joint_enable(2, checked))
        self.sl_j1.valueChanged.connect(lambda v: self.ctrl.set_manual_joint_target(1, v))
        self.sl_j2.valueChanged.connect(lambda v: self.ctrl.set_manual_joint_target(2, v))
        self.sl_speed.valueChanged.connect(lambda v: self.ctrl.set_manual_speed_deg_s(v))

    def _place_overlay(self):
        if not hasattr(self, "overlay"):
            return
        # 放置在右上角
        self.overlay.move(self.arm_canvas.width() - self.overlay.width() - 14, 14)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        QtCore.QTimer.singleShot(0, self._place_overlay)

    def _init_timer(self):
        # UI固定周期刷新，不影响后台1ms控制线程。
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(Config.UI_UPDATE_MS)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start()
        self.update_ui()

    def _on_canvas_target_click(self, x: float, y: float):
        if inverse_kinematics_xy(x, y, self.ctrl.joint) is not None:
            self.ctrl.set_target(x, y)
        else:
            print(
                f"[TARGET] reject X={x*1000:.1f}mm Y={y*1000:.1f}mm "
                f"(unreachable or J1 out of [{Config.J1_MIN_DEG:.0f},{Config.J1_MAX_DEG:.0f}]deg)"
            )

    def update_ui(self):
        st = self.ctrl.get_status()
        j, c, t, cmd = st["joint"], st["cart"], st["target"], st["cmd_joint"]
        man = st["manual"]
        _cart, coords, _jac = kinematics(Joint(j["th1"], j["w1"], j["th2"], j["w2"]))
        self.arm_canvas.set_scene(coords, (t["x"], t["y"]))

        # 更新状态显示
        self.ov_target_v.setText(f"X{t['x']*1000:+.1f} Y{t['y']*1000:+.1f} mm")
        self.ov_cart_v.setText(f"X{c['x']*1000:+.1f} Y{c['y']*1000:+.1f} mm")
        self.ov_cmd_v.setText(f"J1={cmd['th1']:+6.1f}° J2={cmd['th2']:+6.1f}°")
        self.ov_jnt_v.setText(f"J1:{j['th1']:+6.1f}°/{j['w1']:+5.0f}°s J2:{j['th2']:+6.1f}°/{j['w2']:+5.0f}°s")
        self.ov_runtime_v.setText(f"{st['mode']} | {st['control_mode_name']} | {st['hz']:.0f}Hz")
        
        # 更新状态标签
        mode_text = "MANUAL" if (man['enable1'] or man['enable2']) else st['mode']
        self.status_badge.setText(mode_text)

        # 同步手动控制UI（避免循环触发）
        self._manual_ui_sync = True
        try:
            self.cb_j1.setChecked(bool(man["enable1"]))
            self.cb_j2.setChecked(bool(man["enable2"]))
            self.sl_j1.set_value(man["target1"])
            self.sl_j2.set_value(man["target2"])
            self.sl_speed.set_value(man["speed_deg_s"])
        finally:
            self._manual_ui_sync = False

        self._place_overlay()

    def closeEvent(self, event):
        if hasattr(self, "timer"):
            self.timer.stop()
        super().closeEvent(event)


# ==================== Controller ====================
class Controller:
    def __init__(self, use_can: bool = True):
        self.comm = CANComm() if use_can else None
        self.use_can = use_can

        self.joint = Joint()
        # 发送给下位机的目标关节角（逆解结果）
        self.cmd_joint = Joint()
        self.cart = Cart()
        self.ctrl = Ctrl()
        self.target = Cart(x=-0.05, y=-0.12)

        self.running = False
        self.has_feedback = False
        self.lock = threading.Lock()
        self.rx_frames = 0
        self.tx_frames = 0
        self.err_tx_frames = 0
        self.fb1: Optional[MotorStatus] = None
        self.fb2: Optional[MotorStatus] = None
        self.hz = 0.0
        self.t_last = time.perf_counter()
        self.t_last_fb = 0.0
        self.warn_no_fb = False
        self.mode = "CAN FD" if use_can else "SIM"
        # 位置控制发送状态：减少重复发送
        self._target_dirty = True
        self._last_target_xy = (self.target.x, self.target.y)
        self._last_sent_cmd: Optional[Joint] = None
        self._last_sent_cost = float("inf")
        self._last_pos_send_t = 0.0
        # 手动关节控制状态（0->J1, 1->J2）
        self.manual_joint_enable = [False, False]
        self.manual_joint_target = [self.cmd_joint.th1, self.cmd_joint.th2]
        self.manual_joint_cmd = [self.cmd_joint.th1, self.cmd_joint.th2]
        self.manual_speed_deg_s = 90.0

    def set_target(self, x: float, y: float):
        with self.lock:
            self.target.x = x
            self.target.y = y
            # 目标有更新，触发下一次位置指令发送
            self._target_dirty = True
        print(f"[TARGET] X={x*1000:.1f}mm, Y={y*1000:.1f}mm")

    def set_manual_joint_enable(self, joint_idx: int, enabled: bool):
        """启用/关闭单关节手动控制。joint_idx: 1或2"""
        if joint_idx not in (1, 2):
            return
        i = joint_idx - 1
        with self.lock:
            self.manual_joint_enable[i] = bool(enabled)
            if self.manual_joint_enable[i]:
                seed = self.cmd_joint.th1 if i == 0 else self.cmd_joint.th2
                self.manual_joint_target[i] = seed
                self.manual_joint_cmd[i] = seed
                self._target_dirty = True

    def set_manual_joint_target(self, joint_idx: int, target_deg: float):
        """设置单关节手动目标角。joint_idx: 1或2"""
        if joint_idx not in (1, 2):
            return
        if joint_idx == 1:
            target_deg = clamp(target_deg, Config.J1_MIN_DEG, Config.J1_MAX_DEG)
        else:
            target_deg = clamp(target_deg, Config.J2_MANUAL_MIN_DEG, Config.J2_MANUAL_MAX_DEG)
        with self.lock:
            self.manual_joint_target[joint_idx - 1] = target_deg
            self._target_dirty = True

    def set_manual_speed_deg_s(self, speed_deg_s: float):
        with self.lock:
            self.manual_speed_deg_s = clamp(
                float(speed_deg_s),
                Config.MANUAL_SPEED_MIN_DEG_S,
                Config.MANUAL_SPEED_MAX_DEG_S,
            )

    def _update_manual_joint_cmd(self, dt: float):
        """按速度上限对手动目标做斜坡，避免目标突跳。"""
        with self.lock:
            enable = list(self.manual_joint_enable)
            target = list(self.manual_joint_target)
            cmd = list(self.manual_joint_cmd)
            speed = self.manual_speed_deg_s

        max_step = max(0.0, speed * max(0.0, dt))
        for i in range(2):
            err = target[i] - cmd[i]
            if abs(err) <= max_step or max_step <= 0.0:
                cmd[i] = target[i]
            else:
                cmd[i] += max_step if err > 0.0 else -max_step

        cmd[0] = clamp(cmd[0], Config.J1_MIN_DEG, Config.J1_MAX_DEG)
        cmd[1] = clamp(cmd[1], Config.J2_MANUAL_MIN_DEG, Config.J2_MANUAL_MAX_DEG)

        with self.lock:
            self.manual_joint_cmd = cmd

        return enable, cmd

    def control_loop(self):
        print("[CTRL] start loop @ 1kHz")
        t_next = time.perf_counter()

        while self.running:
            t_now = time.perf_counter()
            if t_now >= t_next:
                # Drop overdue ticks and only run current cycle with the latest data.
                if (t_now - t_next) > Config.LOOP_DT:
                    t_next = t_now

                dt = t_now - self.t_last
                self.t_last = t_now
                if dt > 0.0:
                    self.hz = 0.9 * self.hz + 0.1 / dt

                if self.use_can and self.comm:
                    self.mode = "CAN FD"
                    j = self.comm.read_joint()
                    if j is not None:
                        self.joint = j
                        self.has_feedback = True
                        self.t_last_fb = t_now

                    if self.has_feedback:
                        self.cart, _, jac = kinematics(self.joint)
                        with self.lock:
                            tgt = Cart(
                                x=self.target.x,
                                y=self.target.y,
                                vx=self.target.vx,
                                vy=self.target.vy,
                            )
                        self.ctrl = pd_control(self.cart, tgt, jac)
                        # 1ms周期：基于最新反馈求逆解，生成位置目标。
                        ik_joint = inverse_kinematics_xy(tgt.x, tgt.y, self.joint)
                        if ik_joint is not None:
                            self.cmd_joint = ik_joint
                        else:
                            # 目标不可达时保持当前角度，避免发送无效位置跳变。
                            self.cmd_joint = Joint(
                                th1=self.joint.th1,
                                w1=0.0,
                                th2=self.joint.th2,
                                w2=0.0,
                            )
                    else:
                        self.ctrl = Ctrl()

                    # If feedback is stale, fail-safe to zero current.
                    if self.has_feedback and (t_now - self.t_last_fb > Config.RX_STALE_SEC):
                        self.has_feedback = False
                        self.ctrl = Ctrl()
                        self.cmd_joint = Joint(
                            th1=self.joint.th1,
                            w1=0.0,
                            th2=self.joint.th2,
                            w2=0.0,
                        )
                        if not self.warn_no_fb:
                            print("[WARN] CAN FD feedback stale, output forced to zero.")
                            self.warn_no_fb = True
                    else:
                        self.warn_no_fb = False

                    # 手动关节覆盖：对已启用的关节用滑块目标替换逆解结果，速度由speed控制。
                    manual_enable, manual_cmd = self._update_manual_joint_cmd(dt)
                    if manual_enable[0]:
                        self.cmd_joint.th1 = manual_cmd[0]
                    if manual_enable[1]:
                        self.cmd_joint.th2 = manual_cmd[1]
                    manual_active = manual_enable[0] or manual_enable[1]

                    # 发送策略：
                    # - 位置模式：发逆解目标角 + 零电流
                    # - 力矩模式：发最新逆解目标角（占位）+ PD电流
                    if Config.CONTROL_MODE == Config.CONTROL_MODE_POSITION or manual_active:
                        tx_iq1, tx_iq2 = 0.0, 0.0

                        # 仅在需要时发送一次位置目标，避免目标不变时1kHz重复发送。
                        target_changed = False
                        with self.lock:
                            dx = self.target.x - self._last_target_xy[0]
                            dy = self.target.y - self._last_target_xy[1]
                            if math.hypot(dx, dy) > Config.POS_TARGET_DEADBAND_M:
                                target_changed = True
                                self._last_target_xy = (self.target.x, self.target.y)

                        cmd_changed = True
                        if self._last_sent_cmd is not None:
                            cmd_changed = (
                                abs(_wrap_deg_err(self.cmd_joint.th1 - self._last_sent_cmd.th1))
                                > Config.POS_CMD_DEADBAND_DEG
                                or abs(_wrap_deg_err(self.cmd_joint.th2 - self._last_sent_cmd.th2))
                                > Config.POS_CMD_DEADBAND_DEG
                            )

                        # "更优解"用逆解与当前反馈的角度差总和衡量，明显更优再更新。
                        ik_cost = abs(_wrap_deg_err(self.cmd_joint.th1 - self.joint.th1)) + abs(
                            _wrap_deg_err(self.cmd_joint.th2 - self.joint.th2)
                        )
                        better_solution = (self._last_sent_cost - ik_cost) > Config.POS_IK_IMPROVE_DEG
                        keepalive_due = (t_now - self._last_pos_send_t) >= Config.POS_KEEPALIVE_SEC

                        should_send_pos = (
                            self._last_sent_cmd is None
                            or self._target_dirty
                            or target_changed
                            or cmd_changed
                            or better_solution
                            or keepalive_due
                        )

                        if should_send_pos:
                            self.comm.write(self.cmd_joint, tx_iq1, tx_iq2)
                            self._last_sent_cmd = Joint(
                                th1=self.cmd_joint.th1,
                                w1=0.0,
                                th2=self.cmd_joint.th2,
                                w2=0.0,
                            )
                            self._last_sent_cost = ik_cost
                            self._last_pos_send_t = t_now
                            self._target_dirty = False
                    else:
                        tx_iq1, tx_iq2 = self.ctrl.iq1, self.ctrl.iq2
                        self.comm.write(self.cmd_joint, tx_iq1, tx_iq2)
                    self.rx_frames = self.comm.rx_frames
                    self.tx_frames = self.comm.tx_frames
                    self.err_tx_frames = self.comm.err_frames
                    self.fb1 = self.comm.get_motor_status(Config.MOTOR1_ID)
                    self.fb2 = self.comm.get_motor_status(Config.MOTOR2_ID)
                else:
                    # Simulation mode
                    self.mode = "SIM"
                    t = time.time()
                    self.joint = Joint(
                        20 * math.sin(2 * math.pi * 0.5 * t) - 53.7,
                        40 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * t),
                        15 * math.sin(2 * math.pi * 0.3 * t) + 172.5,
                        30 * math.pi * 0.3 * math.cos(2 * math.pi * 0.3 * t),
                    )
                    self.cart, _, _ = kinematics(self.joint)
                    with self.lock:
                        tgt = Cart(
                            x=self.target.x,
                            y=self.target.y,
                            vx=self.target.vx,
                            vy=self.target.vy,
                        )
                    self.ctrl = pd_control(self.cart, tgt, (0.0, 0.0, 0.0, 0.0))
                    self.rx_frames += 1

                t_next += Config.LOOP_DT
            else:
                time.sleep(max(0.0, t_next - time.perf_counter() - 0.00005))

    def start(self):
        if self.use_can and self.comm:
            if not self.comm.open():
                print("[ERROR] CAN FD open failed, startup aborted.")
                return False
        self.running = True
        threading.Thread(target=self.control_loop, daemon=True).start()
        return True

    def stop(self):
        self.running = False
        if self.comm:
            for _ in range(3):
                self.comm.write(self.joint, 0.0, 0.0)
                time.sleep(0.005)
            self.comm.close()

    def get_status(self):
        with self.lock:
            return {
                "joint": {
                    "th1": self.joint.th1,
                    "w1": self.joint.w1,
                    "th2": self.joint.th2,
                    "w2": self.joint.w2,
                },
                "cmd_joint": {
                    "th1": self.cmd_joint.th1,
                    "th2": self.cmd_joint.th2,
                },
                "cart": {"x": self.cart.x, "y": self.cart.y, "vx": self.cart.vx, "vy": self.cart.vy},
                "target": {"x": self.target.x, "y": self.target.y},
                "ctrl": {"iq1": self.ctrl.iq1, "iq2": self.ctrl.iq2},
                "manual": {
                    "enable1": self.manual_joint_enable[0],
                    "enable2": self.manual_joint_enable[1],
                    "target1": self.manual_joint_target[0],
                    "target2": self.manual_joint_target[1],
                    "cmd1": self.manual_joint_cmd[0],
                    "cmd2": self.manual_joint_cmd[1],
                    "speed_deg_s": self.manual_speed_deg_s,
                },
                "hz": self.hz,
                "rx_frames": self.rx_frames,
                "tx_frames": self.tx_frames,
                "err_tx_frames": self.err_tx_frames,
                "mode": self.mode,
                "control_mode_name": control_mode_name(Config.CONTROL_MODE),
                "fb1": None
                if self.fb1 is None
                else {
                    "th": self.fb1.th,
                    "w": self.fb1.w,
                    "cur": self.fb1.cur,
                    "temp": self.fb1.temp,
                    "status": self.fb1.status,
                    "error": self.fb1.error,
                    "t_rx": self.fb1.t_rx,
                },
                "fb2": None
                if self.fb2 is None
                else {
                    "th": self.fb2.th,
                    "w": self.fb2.w,
                    "cur": self.fb2.cur,
                    "temp": self.fb2.temp,
                    "status": self.fb2.status,
                    "error": self.fb2.error,
                    "t_rx": self.fb2.t_rx,
                },
                "now_perf": time.perf_counter(),
            }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="run in simulation mode")
    parser.add_argument("--interface", choices=["slcan"], default="slcan", help="python-can interface (slcan only)")
    parser.add_argument("--channel", default=Config.CAN_CHANNEL, help="python-can channel")
    parser.add_argument("--nominal-bitrate", type=int, default=Config.CAN_NOMINAL_BITRATE, help="CAN FD nominal bitrate (arbitration)")
    parser.add_argument("--data-bitrate", type=int, default=Config.CAN_DATA_BITRATE, help="CAN FD data bitrate")
    parser.add_argument("--tty-baud", type=int, default=Config.SLCAN_TTY_BAUDRATE, help="slcan serial baudrate")
    parser.add_argument("--disable-tx", action="store_true", help="receive only, do not send control frame")
    parser.add_argument(
        "--control-mode",
        choices=["torque", "position"],
        default="torque",
        help="control mode for control frame: torque->target_cur, position->target_pos",
    )
    parser.add_argument("--motor1-id", type=int, default=Config.MOTOR1_ID, help="motor1 CAN node ID")
    parser.add_argument("--motor2-id", type=int, default=Config.MOTOR2_ID, help="motor2 CAN node ID")
    args = parser.parse_args()
    Config.CAN_INTERFACE = "slcan"
    Config.CAN_CHANNEL = args.channel
    Config.CAN_NOMINAL_BITRATE = args.nominal_bitrate
    Config.CAN_DATA_BITRATE = args.data_bitrate
    Config.SLCAN_TTY_BAUDRATE = args.tty_baud
    Config.ENABLE_CONTROL_TX = not bool(args.disable_tx)
    Config.CONTROL_MODE = (
        Config.CONTROL_MODE_TORQUE
        if args.control_mode == "torque"
        else Config.CONTROL_MODE_POSITION
    )
    Config.MOTOR1_ID = args.motor1_id & 0x0F
    Config.MOTOR2_ID = args.motor2_id & 0x0F

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication([])

    ctrl = Controller(use_can=not args.sim)
    vis = Visualizer(ctrl)

    try:
        if not ctrl.start():
            raise RuntimeError("CAN FD start failed. Check interface/channel/bitrate.")
        print("[SYS] running... click plot to set target")
        vis.show()
        app.exec()
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop()


if __name__ == "__main__":
    main()