#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双关节机械臂实时控制 - 文本协议版
格式兼容: 接收 "th1, w1, th2, w2\n"  发送 "q1:x.xxxx, q2:x.xxxx\n"
"""

import serial
import time
import threading
import math
from dataclasses import dataclass
from typing import Tuple, Optional

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ==================== 配置 ====================
class Config:
    PORT = "COM23"
    BAUDRATE = 500000
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


# ==================== 运动学与控制 ====================
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


def pd_control(c: Cart, tgt: Cart, jac: Tuple):
    fx = Config.KPX * (tgt.x - c.x) + Config.KDX * (tgt.vx - c.vx)
    fy = Config.KPY * (tgt.y - c.y) + Config.KDY * (tgt.vy - c.vy)
    fx = max(-Config.F_MAX, min(Config.F_MAX, fx))
    fy = max(-Config.F_MAX, min(Config.F_MAX, fy))
    
    j11, j12, j21, j22 = jac
    tau1 = j11 * fx + j21 * fy
    tau2 = j12 * fx + j22 * fy
    
    iq1 = max(-Config.IQ_MAX, min(Config.IQ_MAX, tau1 / Config.KT))
    iq2 = max(-Config.IQ_MAX, min(Config.IQ_MAX, tau2 / Config.KT))
    
    return Ctrl(fx, fy, iq1, iq2)


# ==================== 串口通信 ====================
class SerialComm:
    """
    接收格式: "-53.9101, 4.3143, 163.0831, -0.2693\n"
    发送格式: "q1:1.2345, q2:-0.5678\n"
    """
    
    def __init__(self):
        self.ser = None
        self.rx_buf = ""
        self.lock = threading.Lock()
        self.last_send_time = 0
        
    def open(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=Config.PORT,
                baudrate=Config.BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001
            )
            self.ser.reset_input_buffer()
            print(f"[串口] {Config.PORT} @ {Config.BAUDRATE}bps")
            print("[协议] 接收: th1,w1,th2,w2 | 发送: q1:x, q2:x")
            return True
        except Exception as e:
            print(f"[错误] {e}")
            return False
    
    def close(self):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
    
    def read(self) -> Optional[Joint]:
        """解析STM32发送的CSV格式"""
        if not self.ser:
            return None
        
        with self.lock:
            try:
                n = self.ser.in_waiting
                if n > 0:
                    self.rx_buf += self.ser.read(n).decode('ascii', errors='ignore')
            except:
                return None
        
        # 处理多行，取最新完整帧
        if '\n' in self.rx_buf:
            lines = self.rx_buf.split('\n')
            # 保留最后不完整部分
            self.rx_buf = lines[-1]
            
            # 遍历所有完整行，取最后一个成功解析的
            valid_joint = None
            for line in lines[:-1]:
                line = line.strip()
                if not line:
                    continue
                    
                # 尝试解析CSV: th1, w1, th2, w2
                try:
                    parts = [p.strip() for p in line.split(',')]
                    if len(parts) >= 4:
                        # 检查是否全是数字（排除调试信息）
                        values = [float(p) for p in parts[:4]]
                        # 合理性检查（角度范围）
                        if abs(values[0]) < 360 and abs(values[2]) < 360:
                            valid_joint = Joint(*values)
                except:
                    continue
            
            return valid_joint
        
        # 缓冲区过长保护
        if len(self.rx_buf) > 1024:
            self.rx_buf = self.rx_buf[-256:]
            
        return None
    
    def write(self, iq1: float, iq2: float) -> bool:
        """
        发送格式: q1:1.2345, q2:-0.5678\n
        与STM32的 target_q: 解析兼容
        """
        if not self.ser:
            return False
        
        # 限制发送频率（避免STM32解析过载）
        t_now = time.perf_counter()
        if t_now - self.last_send_time < 0.001:  # 最多1kHz
            return True  # 跳过但不报错
        
        try:
            with self.lock:
                # 格式: q1:xxxxx, q2:xxxxx\n
                msg = f"q1:{iq1:.4f}, q2:{iq2:.4f}\n"
                self.ser.write(msg.encode('ascii'))
                self.last_send_time = t_now
            return True
        except:
            return False


# ==================== 可视化 ====================
class Visualizer:
    def __init__(self, ctrl: 'Controller'):
        self.ctrl = ctrl
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title('双关节机械臂控制')
        
        self.ax.set_xlim(Config.XLIM)
        self.ax.set_ylim(Config.YLIM)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        
        self.link1, = self.ax.plot([], [], 'b-', lw=6, solid_capstyle='round')
        self.link2, = self.ax.plot([], [], 'r-', lw=6, solid_capstyle='round')
        self.joints, = self.ax.plot([], [], 'ko', ms=8)
        self.end, = self.ax.plot([], [], 'g*', ms=15)
        self.tgt, = self.ax.plot([], [], 'm+', ms=12, mew=2)
        self.tgt_circ = plt.Circle((0, 0), 0.003, fill=False, color='m', ls='--')
        self.ax.add_patch(self.tgt_circ)
        
        self.traj_x, self.traj_y = [], []
        self.traj, = self.ax.plot([], [], 'c-', alpha=0.4, lw=1)
        
        self.text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                fontsize=9, va='top', family='monospace',
                                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
    def on_click(self, event):
        if event.inaxes == self.ax:
            x, y = event.xdata, event.ydata
            if math.hypot(x, y) < Config.L1 + Config.L2 - 0.005:
                self.ctrl.set_target(x, y)
    
    def update(self, frame):
        st = self.ctrl.get_status()
        j, c, t, ct = st['joint'], st['cart'], st['target'], st['ctrl']
        
        _, coords, _ = kinematics(Joint(j['th1'], j['w1'], j['th2'], j['w2']))
        x0, y0, x1, y1, x2, y2 = coords
        
        self.link1.set_data([x0, x1], [y0, y1])
        self.link2.set_data([x1, x2], [y1, y2])
        self.joints.set_data([x0, x1], [y0, y1])
        self.end.set_data([x2], [y2])
        self.tgt.set_data([t['x']], [t['y']])
        self.tgt_circ.center = (t['x'], t['y'])
        
        self.traj_x.append(x2)
        self.traj_y.append(y2)
        if len(self.traj_x) > 100:
            self.traj_x.pop(0)
            self.traj_y.pop(0)
        self.traj.set_data(self.traj_x, self.traj_y)
        
        err = math.hypot(t['x']-c['x'], t['y']-c['y'])
        status = (f"目标:({t['x']*1000:+.1f},{t['y']*1000:+.1f})  "
                 f"实际:({c['x']*1000:+.1f},{c['y']*1000:+.1f})  误差:{err*1000:.2f}mm\n"
                 f"θ₁:{j['th1']:6.1f}° θ₂:{j['th2']:6.1f}°  "
                 f"Iq₁:{ct['iq1']:+.2f}A Iq₂:{ct['iq2']:+.2f}A  "
                 f"Hz:{st['hz']:.0f} RX:{st['rx_cnt']}")
        self.text.set_text(status)
        
        return [self.link1, self.link2, self.joints, self.end, self.tgt, self.traj, self.text]


# ==================== 控制器 ====================
class Controller:
    def __init__(self, use_serial: bool = True):
        self.serial = SerialComm() if use_serial else None
        self.use_serial = use_serial
        
        self.joint = Joint()
        self.cart = Cart()
        self.ctrl = Ctrl()
        self.target = Cart(x=0.03, y=-0.12)
        
        self.running = False
        self.lock = threading.Lock()
        self.rx_cnt = 0
        self.hz = 0
        self.t_last = time.perf_counter()
        
    def set_target(self, x, y):
        with self.lock:
            self.target.x = x
            self.target.y = y
        print(f"[目标] X={x*1000:.1f}mm, Y={y*1000:.1f}mm")
    
    def control_loop(self):
        print("[控制] 启动 1kHz循环")
        T = 0.001
        t_next = time.perf_counter()
        
        while self.running:
            t_now = time.perf_counter()
            if t_now >= t_next:
                dt = t_now - self.t_last
                self.t_last = t_now
                if dt > 0:
                    self.hz = 0.9 * self.hz + 0.1 / dt
                
                if self.use_serial and self.serial:
                    j = self.serial.read()
                    if j:
                        self.joint = j
                        self.rx_cnt += 1
                        
                        c, _, jac = kinematics(j)
                        self.cart = c
                        
                        with self.lock:
                            tgt = self.target
                        self.ctrl = pd_control(c, tgt, jac)
                        
                        self.serial.write(self.ctrl.iq1, self.ctrl.iq2)
                else:
                    # 模拟模式
                    t = time.time()
                    self.joint = Joint(
                        20*math.sin(2*math.pi*0.5*t) - 53.7,
                        40*math.pi*0.5*math.cos(2*math.pi*0.5*t),
                        15*math.sin(2*math.pi*0.3*t) + 172.5,
                        30*math.pi*0.3*math.cos(2*math.pi*0.3*t)
                    )
                    self.cart, _, _ = kinematics(self.joint)
                    with self.lock:
                        tgt = self.target
                    self.ctrl = pd_control(self.cart, tgt, (0,0,0,0))
                    self.rx_cnt += 1
                
                t_next += T
            else:
                time.sleep(max(0, t_next - time.perf_counter() - 0.00005))
    
    def start(self):
        if self.use_serial and self.serial:
            if not self.serial.open():
                print("[警告] 切换到模拟模式")
                self.use_serial = False
        self.running = True
        threading.Thread(target=self.control_loop, daemon=True).start()
        return True
    
    def stop(self):
        self.running = False
        if self.serial:
            # 发送零电流停止
            for _ in range(3):
                self.serial.write(0.0, 0.0)
                time.sleep(0.005)
            self.serial.close()
    
    def get_status(self):
        with self.lock:
            return {
                'joint': {'th1': self.joint.th1, 'w1': self.joint.w1,
                         'th2': self.joint.th2, 'w2': self.joint.w2},
                'cart': {'x': self.cart.x, 'y': self.cart.y},
                'target': {'x': self.target.x, 'y': self.target.y},
                'ctrl': {'iq1': self.ctrl.iq1, 'iq2': self.ctrl.iq2},
                'hz': self.hz, 'rx_cnt': self.rx_cnt
            }


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true')
    args = parser.parse_args()
    
    ctrl = Controller(use_serial=not args.sim)
    vis = Visualizer(ctrl)
    
    try:
        ctrl.start()
        anim = FuncAnimation(vis.fig, vis.update, interval=20, blit=False)
        print("[系统] 运行中... 点击图表设置目标")
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.stop()


if __name__ == "__main__":
    main()