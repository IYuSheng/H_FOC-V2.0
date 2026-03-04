#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual-joint arm real-time control over CAN FD.

Protocol mapping follows Core/Inc/fdcan.h:
- CAN ID (11-bit): (src << 8) | (dst << 4) | msg_type
- STATUS frame payload (16B): <i i h h H H>
- CONTROL frame payload (12B): <i i h h H>
"""

import argparse
import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

try:
    import can
except Exception:
    can = None


# ==================== Config ====================
class Config:
    # CAN transport
    CAN_INTERFACE = "pcan"
    CAN_CHANNEL = "PCAN_USBBUS1"
    CAN_NOMINAL_BITRATE = 1_000_000
    CAN_DATA_BITRATE = 5_000_000
    CAN_FD = True
    CAN_BRS = True

    # Node IDs
    MASTER_ID = 0
    MOTOR1_ID = 1
    MOTOR2_ID = 2
    BROADCAST_ID = 0x0F

    # Message types
    MSG_TYPE_STATUS = 0x0
    MSG_TYPE_CONTROL = 0x1

    # Fixed-point scale (must match fdcan.h)
    SCALE_32 = 1e-5
    SCALE_16 = 1e-2

    # Control mode
    CONTROL_MODE_TORQUE = 2

    # Loop and timeout
    LOOP_DT = 0.001
    RX_STALE_SEC = 0.05

    # Kinematics / controller (copied from your serial script)
    L1, L2 = 0.07748, 0.0625
    ZERO1, ZERO2 = -53.7, 172.5
    KPX, KDX = 20.0, 1.0
    KPY, KDY = 20.0, 1.0
    F_MAX, IQ_MAX, KT = 20.0, 8.0, 0.095
    XLIM, YLIM = (-0.15, 0.15), (-0.18, 0.05)


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
    return ((src & 0x0F) << 8) | ((dst & 0x0F) << 4) | (msg_type & 0x0F)


def can_id_get_src(can_id: int) -> int:
    return (can_id >> 8) & 0x0F


def can_id_get_dst(can_id: int) -> int:
    return (can_id >> 4) & 0x0F


def can_id_get_type(can_id: int) -> int:
    return can_id & 0x0F


def float_to_fix32(v: float) -> int:
    raw = int(v / Config.SCALE_32)
    return max(-2147483648, min(2147483647, raw))


def float_to_fix16(v: float) -> int:
    raw = int(v / Config.SCALE_16)
    return max(-32768, min(32767, raw))


def fix32_to_float(v: int) -> float:
    return float(v) * Config.SCALE_32


def fix16_to_float(v: int) -> float:
    return float(v) * Config.SCALE_16


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


# ==================== CAN communication ====================
class CANComm:
    """
    RX STATUS payload: <i i h h H H>
      position, velocity, current, temperature, status, error_code
    TX CONTROL payload: <i i h h H>
      target_pos, target_vel, target_cur, control_mode, reserve
    """

    def __init__(self):
        self.bus = None
        self.lock = threading.Lock()
        self.state: Dict[int, MotorStatus] = {}
        self.rx_frames = 0
        self.tx_frames = 0
        self.err_frames = 0
        self._warned_no_fb = False

    def open(self) -> bool:
        if can is None:
            print("[ERROR] python-can not installed. Install with: pip install python-can")
            return False

        common_if = {"interface": Config.CAN_INTERFACE, "channel": Config.CAN_CHANNEL}
        common_bt = {"bustype": Config.CAN_INTERFACE, "channel": Config.CAN_CHANNEL}

        candidates = []
        for common in (common_if, common_bt):
            if Config.CAN_FD:
                candidates.append(
                    {
                        **common,
                        "bitrate": Config.CAN_NOMINAL_BITRATE,
                        "fd": True,
                        "data_bitrate": Config.CAN_DATA_BITRATE,
                    }
                )
                candidates.append(
                    {
                        **common,
                        "bitrate": Config.CAN_NOMINAL_BITRATE,
                        "fd": True,
                    }
                )
            candidates.append({**common, "bitrate": Config.CAN_NOMINAL_BITRATE})
            candidates.append({**common})

        last_err = None
        for kwargs in candidates:
            try:
                self.bus = can.Bus(**kwargs)
                print(f"[CAN] open ok: {kwargs}")
                print(
                    "[PROTO] id=(src<<8)|(dst<<4)|type, src=0, dst=1/2, "
                    "control_mode=2(current)"
                )
                return True
            except Exception as exc:
                last_err = exc

        print(f"[ERROR] CAN open failed: {last_err}")
        return False

    def close(self):
        with self.lock:
            if self.bus is not None:
                try:
                    self.bus.shutdown()
                except Exception:
                    pass
                self.bus = None

    def _poll_rx(self, max_frames: int = 64):
        if self.bus is None:
            return
        for _ in range(max_frames):
            try:
                msg = self.bus.recv(timeout=0.0)
            except Exception:
                break
            if msg is None:
                break
            self._handle_msg(msg)

    def _handle_msg(self, msg):
        can_id = msg.arbitration_id
        msg_type = can_id_get_type(can_id)
        src = can_id_get_src(can_id)
        _dst = can_id_get_dst(can_id)

        if msg_type != Config.MSG_TYPE_STATUS:
            return
        if src not in (Config.MOTOR1_ID, Config.MOTOR2_ID):
            return
        if len(msg.data) < 16:
            return

        try:
            p_fix, v_fix, c_fix, t_fix, status, error = struct.unpack(
                "<iihhHH", bytes(msg.data[:16])
            )
        except struct.error:
            return

        now = time.perf_counter()
        with self.lock:
            self.state[src] = MotorStatus(
                th=fix32_to_float(p_fix),
                w=fix32_to_float(v_fix),
                cur=fix16_to_float(c_fix),
                temp=fix16_to_float(t_fix),
                status=status,
                error=error,
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
        self._poll_rx()
        with self.lock:
            s1 = self.state.get(Config.MOTOR1_ID)
            s2 = self.state.get(Config.MOTOR2_ID)
        if s1 is None or s2 is None:
            return None

        now = time.perf_counter()
        if (now - s1.t_rx) > Config.RX_STALE_SEC or (now - s2.t_rx) > Config.RX_STALE_SEC:
            return None

        return Joint(th1=s1.th, w1=s1.w, th2=s2.th, w2=s2.w)

    def _send_current(self, dst_id: int, iq: float) -> bool:
        if self.bus is None or can is None:
            return False

        iq = clamp(iq, -Config.IQ_MAX, Config.IQ_MAX)
        payload = struct.pack(
            "<iihhH",
            float_to_fix32(0.0),              # target_pos
            float_to_fix32(0.0),              # target_vel
            float_to_fix16(iq),               # target_cur
            Config.CONTROL_MODE_TORQUE,       # control_mode = current/torque
            0,                                # reserve
        )
        can_id = can_id_make(Config.MASTER_ID, dst_id, Config.MSG_TYPE_CONTROL)
        try:
            msg = can.Message(
                arbitration_id=can_id,
                is_extended_id=False,
                data=payload,
                is_fd=Config.CAN_FD,
                bitrate_switch=Config.CAN_BRS,
            )
            self.bus.send(msg, timeout=0.0)
            self.tx_frames += 1
            return True
        except Exception:
            self.err_frames += 1
            return False

    def write(self, iq1: float, iq2: float) -> bool:
        ok1 = self._send_current(Config.MOTOR1_ID, iq1)
        ok2 = self._send_current(Config.MOTOR2_ID, iq2)
        return ok1 and ok2


# ==================== Visualization ====================
class Visualizer:
    def __init__(self, ctrl: "Controller"):
        self.ctrl = ctrl
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("Dual-joint Arm Control (CAN)")

        self.ax.set_xlim(Config.XLIM)
        self.ax.set_ylim(Config.YLIM)
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")

        self.link1, = self.ax.plot([], [], "b-", lw=6, solid_capstyle="round")
        self.link2, = self.ax.plot([], [], "r-", lw=6, solid_capstyle="round")
        self.joints, = self.ax.plot([], [], "ko", ms=8)
        self.end, = self.ax.plot([], [], "g*", ms=15)
        self.tgt, = self.ax.plot([], [], "m+", ms=12, mew=2)
        self.tgt_circ = plt.Circle((0, 0), 0.003, fill=False, color="m", ls="--")
        self.ax.add_patch(self.tgt_circ)

        self.traj_x, self.traj_y = [], []
        self.traj, = self.ax.plot([], [], "c-", alpha=0.4, lw=1)

        self.text = self.ax.text(
            0.02,
            0.98,
            "",
            transform=self.ax.transAxes,
            fontsize=9,
            va="top",
            family="monospace",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
        )

        self.fig.canvas.mpl_connect("button_press_event", self.on_click)

    def on_click(self, event):
        if event.inaxes == self.ax:
            x, y = event.xdata, event.ydata
            if math.hypot(x, y) < Config.L1 + Config.L2 - 0.005:
                self.ctrl.set_target(x, y)

    def update(self, _frame):
        st = self.ctrl.get_status()
        j, c, t, ct = st["joint"], st["cart"], st["target"], st["ctrl"]
        fb1, fb2 = st["fb1"], st["fb2"]

        _, coords, _ = kinematics(Joint(j["th1"], j["w1"], j["th2"], j["w2"]))
        x0, y0, x1, y1, x2, y2 = coords

        self.link1.set_data([x0, x1], [y0, y1])
        self.link2.set_data([x1, x2], [y1, y2])
        self.joints.set_data([x0, x1], [y0, y1])
        self.end.set_data([x2], [y2])
        self.tgt.set_data([t["x"]], [t["y"]])
        self.tgt_circ.center = (t["x"], t["y"])

        self.traj_x.append(x2)
        self.traj_y.append(y2)
        if len(self.traj_x) > 100:
            self.traj_x.pop(0)
            self.traj_y.pop(0)
        self.traj.set_data(self.traj_x, self.traj_y)

        err = math.hypot(t["x"] - c["x"], t["y"] - c["y"])

        def fb_line(name: str, fb: Optional[Dict], now_perf: float) -> str:
            if fb is None:
                return f"{name}: no status frame"
            age_ms = (now_perf - fb["t_rx"]) * 1000.0
            stale = " STALE" if age_ms > (Config.RX_STALE_SEC * 1000.0) else ""
            return (
                f"{name}: th={fb['th']:+7.2f}deg w={fb['w']:+8.2f}deg/s "
                f"i={fb['cur']:+6.2f}A T={fb['temp']:+5.1f}C "
                f"st=0x{fb['status']:04X} err=0x{fb['error']:04X} "
                f"age={age_ms:5.1f}ms{stale}"
            )

        status = (
            f"Target:({t['x']*1000:+.1f},{t['y']*1000:+.1f})  "
            f"Actual:({c['x']*1000:+.1f},{c['y']*1000:+.1f})  Err:{err*1000:.2f}mm\n"
            f"th1:{j['th1']:6.1f}deg th2:{j['th2']:6.1f}deg  "
            f"Iq1:{ct['iq1']:+.2f}A Iq2:{ct['iq2']:+.2f}A  "
            f"Hz:{st['hz']:.0f} RX:{st['rx_frames']} TX:{st['tx_frames']}\n"
            f"{fb_line('M1', fb1, st['now_perf'])}\n"
            f"{fb_line('M2', fb2, st['now_perf'])}"
        )
        self.text.set_text(status)
        return [self.link1, self.link2, self.joints, self.end, self.tgt, self.traj, self.text]


# ==================== Controller ====================
class Controller:
    def __init__(self, use_can: bool = True):
        self.comm = CANComm() if use_can else None
        self.use_can = use_can

        self.joint = Joint()
        self.cart = Cart()
        self.ctrl = Ctrl()
        self.target = Cart(x=0.03, y=-0.12)

        self.running = False
        self.has_feedback = False
        self.lock = threading.Lock()
        self.rx_frames = 0
        self.tx_frames = 0
        self.fb1: Optional[MotorStatus] = None
        self.fb2: Optional[MotorStatus] = None
        self.hz = 0.0
        self.t_last = time.perf_counter()
        self.t_last_fb = 0.0
        self.warn_no_fb = False

    def set_target(self, x: float, y: float):
        with self.lock:
            self.target.x = x
            self.target.y = y
        print(f"[TARGET] X={x*1000:.1f}mm, Y={y*1000:.1f}mm")

    def control_loop(self):
        print("[CTRL] start loop @ 1kHz")
        t_next = time.perf_counter()

        while self.running:
            t_now = time.perf_counter()
            if t_now >= t_next:
                dt = t_now - self.t_last
                self.t_last = t_now
                if dt > 0.0:
                    self.hz = 0.9 * self.hz + 0.1 / dt

                if self.use_can and self.comm:
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
                    else:
                        self.ctrl = Ctrl()

                    # If feedback is stale, fail-safe to zero current.
                    if self.has_feedback and (t_now - self.t_last_fb > Config.RX_STALE_SEC):
                        self.has_feedback = False
                        self.ctrl = Ctrl()
                        if not self.warn_no_fb:
                            print("[WARN] CAN feedback stale, output forced to zero.")
                            self.warn_no_fb = True
                    else:
                        self.warn_no_fb = False

                    self.comm.write(self.ctrl.iq1, self.ctrl.iq2)
                    self.rx_frames = self.comm.rx_frames
                    self.tx_frames = self.comm.tx_frames
                    self.fb1 = self.comm.get_motor_status(Config.MOTOR1_ID)
                    self.fb2 = self.comm.get_motor_status(Config.MOTOR2_ID)
                else:
                    # Simulation mode
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
                print("[WARN] switch to simulation mode")
                self.use_can = False
        self.running = True
        threading.Thread(target=self.control_loop, daemon=True).start()
        return True

    def stop(self):
        self.running = False
        if self.comm:
            for _ in range(3):
                self.comm.write(0.0, 0.0)
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
                "cart": {"x": self.cart.x, "y": self.cart.y},
                "target": {"x": self.target.x, "y": self.target.y},
                "ctrl": {"iq1": self.ctrl.iq1, "iq2": self.ctrl.iq2},
                "hz": self.hz,
                "rx_frames": self.rx_frames,
                "tx_frames": self.tx_frames,
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
    parser.add_argument("--interface", default=Config.CAN_INTERFACE, help="python-can interface")
    parser.add_argument("--channel", default=Config.CAN_CHANNEL, help="python-can channel")
    parser.add_argument("--bitrate", type=int, default=Config.CAN_NOMINAL_BITRATE, help="nominal bitrate")
    parser.add_argument("--dbitrate", type=int, default=Config.CAN_DATA_BITRATE, help="data bitrate (CAN FD)")
    parser.add_argument("--motor1-id", type=int, default=Config.MOTOR1_ID, help="motor1 CAN node ID")
    parser.add_argument("--motor2-id", type=int, default=Config.MOTOR2_ID, help="motor2 CAN node ID")
    args = parser.parse_args()

    Config.CAN_INTERFACE = args.interface
    Config.CAN_CHANNEL = args.channel
    Config.CAN_NOMINAL_BITRATE = args.bitrate
    Config.CAN_DATA_BITRATE = args.dbitrate
    Config.MOTOR1_ID = args.motor1_id & 0x0F
    Config.MOTOR2_ID = args.motor2_id & 0x0F

    ctrl = Controller(use_can=not args.sim)
    vis = Visualizer(ctrl)

    try:
        ctrl.start()
        _anim = FuncAnimation(vis.fig, vis.update, interval=20, blit=False)
        print("[SYS] running... click plot to set target")
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop()


if __name__ == "__main__":
    main()
