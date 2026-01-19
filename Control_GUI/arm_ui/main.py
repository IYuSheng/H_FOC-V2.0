# ==========================
# RC + Serial + CAN(SLCAN)
# Windows / PySide6
# ==========================

import sys
import time
import threading

import pygame
import serial
import serial.tools.list_ports
import can  # python-can

from PySide6.QtCore import QTimer, Signal, QObject
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QComboBox, QTextEdit, QLineEdit, QGroupBox
)


# ---------------- 串口工作线程类 ----------------
class SerialWorker(QObject):
    rx_text = Signal(str)
    status = Signal(str)

    def __init__(self):
        super().__init__()
        self.ser = None
        self._stop = False
        self._thread = None

    def open(self, port: str, baud: int):
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)
            self._stop = False
            self._thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._thread.start()
            self.status.emit(f"Serial connected: {port} @ {baud}")
            return True
        except Exception as e:
            self.ser = None
            self.status.emit(f"Serial connect failed: {e}")
            return False

    def close(self):
        self._stop = True
        time.sleep(0.05)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.status.emit("Serial disconnected.")

    def send_line(self, line: str):
        if not self.ser:
            self.status.emit("Serial not connected.")
            return
        try:
            data = (line + "\r\n").encode("utf-8", errors="ignore")
            self.ser.write(data)
        except Exception as e:
            self.status.emit(f"Serial send failed: {e}")

    def send_bytes(self, b: bytes):
        if not self.ser:
            self.status.emit("Serial not connected.")
            return
        try:
            self.ser.write(b)
        except Exception as e:
            self.status.emit(f"Serial send failed: {e}")

    def _rx_loop(self):
        buf = b""
        while not self._stop:
            if not self.ser:
                break
            try:
                chunk = self.ser.read(256)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            s = line.decode("utf-8", errors="ignore").strip()
                        except Exception:
                            s = str(line)
                        if s:
                            self.rx_text.emit(s)
            except Exception as e:
                self.status.emit(f"Serial rx error: {e}")
                break


# ---------------- CAN Worker (SLCAN / CANable) ----------------
class CanWorker(QObject):
    rx_frame = Signal(str)
    status = Signal(str)

    def __init__(self):
        super().__init__()
        self.bus = None
        self._stop = False
        self._thread = None

        # 你现在 MCU: float*10000 -> int16
        self.scale = 10000.0

        # 发送相关
        self.tx_id = 0x002          # 默认：发给 MCU 的命令 ID（和你 MCU 过滤器一致）
        self.tx_lock = threading.Lock()

    def open_slcan(self, com_port: str, bitrate: int = 1_000_000):
        try:
            self.bus = can.Bus(
                interface="slcan",
                channel=com_port,
                bitrate=bitrate
            )

            self._stop = False
            self._thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._thread.start()
            self.status.emit(f"CAN connected: {com_port} (SLCAN), bitrate={bitrate}")
            return True
        except Exception as e:
            self.bus = None
            self.status.emit(f"CAN connect failed: {e}")
            return False

    def close(self):
        self._stop = True
        time.sleep(0.05)
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        self.bus = None
        self.status.emit("CAN disconnected.")

    def _rx_loop(self):
        while not self._stop:
            if not self.bus:
                break
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg is None:
                    continue

                can_id = msg.arbitration_id
                dlc = msg.dlc
                data = bytes(msg.data)
                hex_str = " ".join(f"{b:02X}" for b in data)

                parsed = ""
                if dlc >= 8:
                    def i16(lo, hi):
                        v = (hi << 8) | lo
                        if v & 0x8000:
                            v -= 0x10000
                        return v

                    v0 = i16(data[0], data[1]) / self.scale
                    v1 = i16(data[2], data[3]) / self.scale
                    v2 = i16(data[4], data[5]) / self.scale
                    v3 = i16(data[6], data[7]) / self.scale
                    parsed = f" | f0={v0:+.4f}, f1={v1:+.4f}, f2={v2:+.4f}, f3={v3:+.4f}"

                line = f"ID=0x{can_id:03X} DLC={dlc} DATA=[{hex_str}]{parsed}"
                self.rx_frame.emit(line)

            except Exception as e:
                self.status.emit(f"CAN rx error: {e}")
                break

    # ---------------- 发送：像串口一样发字符串 ----------------
    def send_bytes(self, b: bytes, arb_id: int | None = None):
        """
        发送 raw bytes（SLCAN 只能发经典 CAN，单帧最多 8 bytes）
        """
        if not self.bus:
            self.status.emit("CAN not connected.")
            return False

        if arb_id is None:
            arb_id = self.tx_id

        # SLCAN / Classic CAN：单帧最多 8 字节 -> 需要分片
        try:
            with self.tx_lock:
                for i in range(0, len(b), 8):
                    chunk = b[i:i+8]
                    msg = can.Message(
                        arbitration_id=arb_id,
                        is_extended_id=False,
                        data=chunk
                    )
                    self.bus.send(msg)
            return True
        except Exception as e:
            self.status.emit(f"CAN send failed: {e}")
            return False

    def send_line(self, line: str, arb_id: int | None = None):
        """
        发送一行 ASCII（自动加 \\n）
        """
        if not line.endswith("\n"):
            line = line + "\n"
        return self.send_bytes(line.encode("ascii", errors="ignore"), arb_id=arb_id)


# ---------------- 主界面类 ----------------
class RcReader(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RC + Serial + CAN(SLCAN) (PySide6)")
        self.resize(920, 780)

        root = QVBoxLayout(self)

        # ===================== RC 组 =====================
        rc_group = QGroupBox("RC (USB HID)")
        rc_layout = QVBoxLayout(rc_group)

        self.combo_rc = QComboBox()
        self.btn_scan_rc = QPushButton("Scan Controllers")
        self.btn_connect_rc = QPushButton("Connect RC")
        self.btn_disconnect_rc = QPushButton("Disconnect RC")
        self.info_rc = QLabel("RC Status: idle")
        self.out_rc = QTextEdit()
        self.out_rc.setReadOnly(True)

        rc_layout.addWidget(self.combo_rc)
        rc_layout.addWidget(self.btn_scan_rc)

        rc_btns = QHBoxLayout()
        rc_btns.addWidget(self.btn_connect_rc)
        rc_btns.addWidget(self.btn_disconnect_rc)
        rc_layout.addLayout(rc_btns)

        rc_layout.addWidget(self.info_rc)
        rc_layout.addWidget(self.out_rc)

        self.btn_disconnect_rc.setEnabled(False)

        # ===================== Serial 组 =====================
        ser_group = QGroupBox("Serial (Arm UART Test)")
        ser_layout = QVBoxLayout(ser_group)

        row1 = QHBoxLayout()
        self.combo_ser = QComboBox()
        self.btn_refresh_ser = QPushButton("Refresh Ports")
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["115200", "921600", "500000", "1000000", "2000000"])
        self.btn_connect_ser = QPushButton("Connect Serial")
        self.btn_disconnect_ser = QPushButton("Disconnect Serial")
        self.btn_disconnect_ser.setEnabled(False)

        row1.addWidget(QLabel("Port:"))
        row1.addWidget(self.combo_ser, 1)
        row1.addWidget(self.btn_refresh_ser)
        row1.addWidget(QLabel("Baud:"))
        row1.addWidget(self.combo_baud)
        row1.addWidget(self.btn_connect_ser)
        row1.addWidget(self.btn_disconnect_ser)

        row2 = QHBoxLayout()
        self.edit_tx = QLineEdit()
        self.edit_tx.setPlaceholderText("Type text and press Send. Example: hello")
        self.btn_send = QPushButton("Send")
        row2.addWidget(self.edit_tx, 1)
        row2.addWidget(self.btn_send)

        self.info_ser = QLabel("Serial Status: idle")
        self.out_ser = QTextEdit()
        self.out_ser.setReadOnly(True)

        ser_layout.addLayout(row1)
        ser_layout.addLayout(row2)
        ser_layout.addWidget(self.info_ser)
        ser_layout.addWidget(self.out_ser)

        # ===================== CAN 组 =====================
        can_group = QGroupBox("CAN (CANable SLCAN / CAN FD NBRS)")
        can_layout = QVBoxLayout(can_group)

        can_row1 = QHBoxLayout()
        self.combo_can = QComboBox()
        self.btn_refresh_can = QPushButton("Refresh CAN Ports")
        self.combo_can_bitrate = QComboBox()
        self.combo_can_bitrate.addItems(["1000000", "500000"])
        self.btn_connect_can = QPushButton("Connect CAN")
        self.btn_disconnect_can = QPushButton("Disconnect CAN")
        self.btn_disconnect_can.setEnabled(False)

        can_row1.addWidget(QLabel("Port:"))
        can_row1.addWidget(self.combo_can, 1)
        can_row1.addWidget(self.btn_refresh_can)
        can_row1.addWidget(QLabel("Bitrate:"))
        can_row1.addWidget(self.combo_can_bitrate)
        can_row1.addWidget(self.btn_connect_can)
        can_row1.addWidget(self.btn_disconnect_can)

        self.info_can = QLabel("CAN Status: idle")
        self.out_can = QTextEdit()
        self.out_can.setReadOnly(True)

        can_layout.addLayout(can_row1)
        can_layout.addWidget(self.info_can)
        can_layout.addWidget(self.out_can)

        # ===================== 添加到主布局 =====================
        root.addWidget(rc_group, 2)
        root.addWidget(ser_group, 3)
        root.addWidget(can_group, 3)

        # ===================== RC (pygame) =====================
        pygame.init()
        pygame.joystick.init()
        self.joy = None

        self.timer_rc = QTimer(self)
        self.timer_rc.setInterval(20)  # 50Hz
        self.timer_rc.timeout.connect(self.poll_rc)

        self.btn_scan_rc.clicked.connect(self.scan_rc)
        self.btn_connect_rc.clicked.connect(self.connect_rc)
        self.btn_disconnect_rc.clicked.connect(self.disconnect_rc)

        # ===================== Serial Worker =====================
        self.ser_worker = SerialWorker()
        self.ser_worker.rx_text.connect(self.on_ser_rx)
        self.ser_worker.status.connect(self.on_ser_status)

        self.btn_refresh_ser.clicked.connect(self.refresh_ports)
        self.btn_connect_ser.clicked.connect(self.connect_serial)
        self.btn_disconnect_ser.clicked.connect(self.disconnect_serial)
        self.btn_send.clicked.connect(self.send_serial)

        # ===================== CAN Worker =====================
        self.can_worker = CanWorker()
        self.can_worker.rx_frame.connect(self.on_can_rx)
        self.can_worker.status.connect(self.on_can_status)

        self.btn_refresh_can.clicked.connect(self.refresh_can_ports)
        self.btn_connect_can.clicked.connect(self.connect_can)
        self.btn_disconnect_can.clicked.connect(self.disconnect_can)

        # ===================== 初始化扫描 =====================
        self.scan_rc()
        self.refresh_ports()
        self.refresh_can_ports()

    # ---------------- RC ----------------
    def scan_rc(self):
        self.combo_rc.clear()
        pygame.joystick.quit()
        pygame.joystick.init()

        n = pygame.joystick.get_count()
        if n == 0:
            self.info_rc.setText("RC Status: No HID controller found. (Check joy.cpl)")
            return

        for i in range(n):
            j = pygame.joystick.Joystick(i)
            j.init()
            self.combo_rc.addItem(f"[{i}] {j.get_name()} (GUID={j.get_guid()})", i)

        self.info_rc.setText(f"RC Status: Found {n} controller(s).")

    def connect_rc(self):
        if self.combo_rc.count() == 0:
            self.info_rc.setText("RC Status: No controller to connect.")
            return
        idx = self.combo_rc.currentData()
        try:
            self.joy = pygame.joystick.Joystick(idx)
            self.joy.init()
            axes = self.joy.get_numaxes()
            buttons = self.joy.get_numbuttons()
            hats = self.joy.get_numhats()
            self.info_rc.setText(
                f"RC Status: Connected [{idx}] {self.joy.get_name()} | axes={axes}, buttons={buttons}, hats={hats}"
            )
            self.btn_connect_rc.setEnabled(False)
            self.btn_disconnect_rc.setEnabled(True)
            self.timer_rc.start()
        except Exception as e:
            self.info_rc.setText(f"RC Status: Connect failed: {e}")
            self.joy = None

    def disconnect_rc(self):
        self.timer_rc.stop()
        if self.joy is not None:
            try:
                self.joy.quit()
            except Exception:
                pass
        self.joy = None
        self.info_rc.setText("RC Status: Disconnected.")
        self.btn_connect_rc.setEnabled(True)
        self.btn_disconnect_rc.setEnabled(False)

    def poll_rc(self):
        """
        50Hz 轮询手柄
        - 串口发送 target_position
        - CAN 也发送同样字符串（会自动 8B 分片）
        """
        if self.joy is None:
            return

        pygame.event.pump()
        axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        buttons = [self.joy.get_button(i) for i in range(self.joy.get_numbuttons())]

        # 生成同一份命令字符串
        axis_1 = axes[1] if len(axes) > 1 else 0.0
        target_pos = axis_1 * 360.0
        cmd = f"target_position:{target_pos:.4f}\n"

        # 串口已连接：发送
        if self.ser_worker.ser:
            self.ser_worker.send_bytes(cmd.encode("ascii"))

        # CAN 已连接：发送（注意：Classic CAN 单帧 8字节，会分片）
        if self.can_worker.bus:
            self.can_worker.send_bytes(cmd.encode("ascii"))  # 默认 ID=0x002

        txt = []
        txt.append("Axes:   " + "  ".join([f"{v:+.3f}" for v in axes]))
        txt.append("Buttons:" + "  ".join([str(b) for b in buttons[:16]]) + (" ..." if len(buttons) > 16 else ""))
        self.out_rc.setPlainText("\n".join(txt))

    # ---------------- Serial ----------------
    def refresh_ports(self):
        self.combo_ser.clear()
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.combo_ser.addItem(f"{p.device} - {p.description}", p.device)
        if not ports:
            self.info_ser.setText("Serial Status: No COM ports found.")

    def connect_serial(self):
        if self.combo_ser.count() == 0:
            self.info_ser.setText("Serial Status: No port to connect.")
            return
        port = self.combo_ser.currentData()
        baud = int(self.combo_baud.currentText())
        ok = self.ser_worker.open(port, baud)
        if ok:
            self.btn_connect_ser.setEnabled(False)
            self.btn_disconnect_ser.setEnabled(True)

    def disconnect_serial(self):
        self.ser_worker.close()
        self.btn_connect_ser.setEnabled(True)
        self.btn_disconnect_ser.setEnabled(False)

    def send_serial(self):
        s = self.edit_tx.text().strip()
        if not s:
            return
        self.ser_worker.send_line(s)
        self.out_ser.append(f"[TX] {s}")

    def on_ser_rx(self, s: str):
        self.out_ser.append(f"[RX] {s}")

    def on_ser_status(self, s: str):
        self.info_ser.setText(f"Serial Status: {s}")

    # ---------------- CAN (SLCAN) ----------------
    def refresh_can_ports(self):
        self.combo_can.clear()
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            self.combo_can.addItem(f"{p.device} - {p.description}", p.device)
        if not ports:
            self.info_can.setText("CAN Status: No COM ports found.")

    def connect_can(self):
        if self.combo_can.count() == 0:
            self.info_can.setText("CAN Status: No port to connect.")
            return
        port = self.combo_can.currentData()
        bitrate = int(self.combo_can_bitrate.currentText())
        ok = self.can_worker.open_slcan(port, bitrate)
        if ok:
            self.btn_connect_can.setEnabled(False)
            self.btn_disconnect_can.setEnabled(True)

    def disconnect_can(self):
        self.can_worker.close()
        self.btn_connect_can.setEnabled(True)
        self.btn_disconnect_can.setEnabled(False)

    def on_can_rx(self, s: str):
        self.out_can.append(s)

    def on_can_status(self, s: str):
        self.info_can.setText(f"CAN Status: {s}")

    # ---------------- Close event ----------------
    def closeEvent(self, event):
        try:
            self.timer_rc.stop()
        except Exception:
            pass
        try:
            self.disconnect_rc()
        except Exception:
            pass
        try:
            self.disconnect_serial()
        except Exception:
            pass
        try:
            self.disconnect_can()
        except Exception:
            pass
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    w = RcReader()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()