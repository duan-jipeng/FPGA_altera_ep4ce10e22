"""
W5500 TCP 通信上位机 — 八通道独立脉冲控制
基于 PySide6 实现 TCP Client 功能，每个 PWM 通道独立显示
"""

import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                                QHBoxLayout, QLabel, QLineEdit, QPushButton,
                                QTextEdit, QSpinBox, QGroupBox, QComboBox,
                                QMessageBox, QGridLayout, QFrame)
from PySide6.QtCore import Qt, QThread, Signal, QTimer
from PySide6.QtGui import QFont, QTextCursor
import socket
import datetime


# ==================== TCP 工作线程 ====================

class TCPWorker(QThread):
    data_received = Signal(str)
    raw_data_received = Signal(bytes)
    connection_status = Signal(str)
    error_occurred = Signal(str)

    def __init__(self):
        super().__init__()
        self.host = "192.168.123.98"
        self.port = 5000
        self.connected = False
        self.client_socket = None
        self.running = True

    def run(self):
        while self.running:
            if not self.connected:
                self.msleep(100)
                continue
            try:
                data = self.client_socket.recv(1024)
                if data:
                    self.raw_data_received.emit(data)
                    message = data.decode('utf-8', errors='ignore')
                    self.data_received.emit(message)
                else:
                    self.connection_status.emit("服务器断开连接")
                    self.connected = False
                    self.client_socket.close()
            except socket.timeout:
                pass
            except Exception as e:
                if self.connected:
                    self.connected = False
                    if "非套接字" not in str(e) and "10038" not in str(e):
                        self.error_occurred.emit(f"接收错误: {str(e)}")
                    try:
                        self.client_socket.close()
                    except:
                        pass
                    self.connection_status.emit("已断开连接")

    def connect_to_server(self, host, port):
        try:
            self.host = host
            self.port = port
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(1.0)
            self.client_socket.connect((self.host, self.port))
            self.connected = True
            self.connection_status.emit(f"已连接到 {host}:{port}")
            return True
        except Exception as e:
            self.error_occurred.emit(f"连接失败: {str(e)}")
            return False

    def disconnect(self):
        self.connected = False  # 先设标志, 再关 socket, 避免 race condition
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.connection_status.emit("已断开连接")

    def send_data(self, data):
        if self.connected and self.client_socket:
            try:
                if isinstance(data, str):
                    self.client_socket.send(data.encode('utf-8'))
                else:
                    self.client_socket.send(data)
                return True
            except Exception as e:
                self.error_occurred.emit(f"发送失败: {str(e)}")
                return False
        return False

    def stop(self):
        self.running = False
        self.disconnect()
        self.wait()


# ==================== 单个通道面板 ====================

class ChannelPanel(QGroupBox):
    """独立 PWM 通道控制面板"""

    def __init__(self, title, chan_byte, pulse_pin, dir_pin, color, parent=None):
        super().__init__(title, parent)
        self.chan_byte = chan_byte
        self.pulse_pin = pulse_pin
        self.dir_pin = dir_pin
        self.color = color
        self.worker = None  # 由主窗口注入
        self.log_func = None

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout()

        # ---- 引脚信息 ----
        pin_label = QLabel(f"Pulse: {self.pulse_pin}  |  DIR: {self.dir_pin}")
        pin_label.setStyleSheet(f"color: {self.color}; font-weight: bold;")
        pin_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(pin_label)

        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        # ---- 模式选择 ----
        row_mode = QHBoxLayout()
        row_mode.addWidget(QLabel("模式:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["固定脉冲", "持续脉冲"])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        row_mode.addWidget(self.mode_combo)
        row_mode.addStretch()

        # ---- 频率 ----
        row_freq = QHBoxLayout()
        row_freq.addWidget(QLabel("频率:"))
        self.freq_spin = QSpinBox()
        self.freq_spin.setRange(100, 200000)
        self.freq_spin.setValue(10000)
        self.freq_spin.setSingleStep(100)
        self.freq_spin.setSuffix(" Hz")
        row_freq.addWidget(self.freq_spin)
        row_freq.addStretch()

        # ---- 脉冲数量 ----
        row_count = QHBoxLayout()
        row_count.addWidget(QLabel("脉冲数:"))
        self.count_spin = QSpinBox()
        self.count_spin.setRange(1, 10000000)
        self.count_spin.setValue(10)
        row_count.addWidget(self.count_spin)
        row_count.addStretch()

        # ---- 方向 ----
        row_dir = QHBoxLayout()
        row_dir.addWidget(QLabel("方向:"))
        self.dir_combo = QComboBox()
        self.dir_combo.addItems(["LOW", "HIGH"])
        self.dir_combo.setCurrentIndex(1)  # 默认 HIGH，匹配固件上电状态
        self.dir_combo.currentIndexChanged.connect(self._send_direction)
        row_dir.addWidget(self.dir_combo)
        row_dir.addStretch()

        # ---- 2×2 网格布局: 模式/频率 第一行, 脉冲数/方向 第二行 ----
        param_grid = QGridLayout()
        param_grid.setContentsMargins(0, 0, 0, 0)
        param_grid.setSpacing(4)
        param_grid.addLayout(row_mode,  0, 0)
        param_grid.addLayout(row_freq,  0, 1)
        param_grid.addLayout(row_count, 1, 0)
        param_grid.addLayout(row_dir,   1, 1)
        layout.addLayout(param_grid)

        # ---- 发送 / 停止 ----
        row_btn = QHBoxLayout()
        self.send_btn = QPushButton("发送脉冲")
        self.send_btn.setStyleSheet(f"background-color: {self.color}; color: white; font-weight: bold;")
        self.send_btn.clicked.connect(self._send_pulse)
        row_btn.addWidget(self.send_btn)

        self.stop_btn = QPushButton("停止")
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.stop_btn.clicked.connect(self._send_stop)
        row_btn.addWidget(self.stop_btn)
        layout.addLayout(row_btn)

        self.setLayout(layout)

    def set_worker(self, worker, log_func):
        self.worker = worker
        self.log_func = log_func

    def _on_mode_changed(self, index):
        is_fixed = (index == 0)
        self.count_spin.setEnabled(is_fixed)

    def _calc_freq_unit(self, hz):
        u = hz // 100
        return max(u, 1)

    def _log(self, msg):
        if self.log_func:
            self.log_func(msg)

    def _send_pulse(self):
        if not self.worker or not self.worker.connected:
            QMessageBox.warning(self, "警告", "请先连接到设备！")
            return

        mode = self.mode_combo.currentIndex()
        freq_hz = self.freq_spin.value()
        freq_u = self._calc_freq_unit(freq_hz)
        actual_hz = freq_u * 100

        if mode == 0:
            pulse_count = self.count_spin.value()
            frame = bytearray(11)
            frame[0] = 0xAA
            frame[1] = 0x55
            frame[2] = 0x01
            frame[3] = self.chan_byte
            frame[4] = (pulse_count >> 24) & 0xFF
            frame[5] = (pulse_count >> 16) & 0xFF
            frame[6] = (pulse_count >> 8) & 0xFF
            frame[7] = pulse_count & 0xFF
            frame[8] = (freq_u >> 8) & 0xFF
            frame[9] = freq_u & 0xFF
            checksum = 0
            for i in range(10):
                checksum ^= frame[i]
            frame[10] = checksum

            if self.worker.send_data(bytes(frame)):
                hx = ' '.join(f'{b:02X}' for b in frame)
                self._log(f"[{self.title()}] 固定脉冲 {pulse_count}个 @{actual_hz}Hz: {hx}")
        else:
            frame = bytearray(7)
            frame[0] = 0xAA
            frame[1] = 0x55
            frame[2] = 0x02
            frame[3] = self.chan_byte
            frame[4] = (freq_u >> 8) & 0xFF
            frame[5] = freq_u & 0xFF
            checksum = 0
            for i in range(6):
                checksum ^= frame[i]
            frame[6] = checksum

            if self.worker.send_data(bytes(frame)):
                hx = ' '.join(f'{b:02X}' for b in frame)
                self._log(f"[{self.title()}] 持续脉冲 @{actual_hz}Hz: {hx}")

    def _send_stop(self):
        if not self.worker or not self.worker.connected:
            QMessageBox.warning(self, "警告", "请先连接到设备！")
            return

        frame = bytearray(5)
        frame[0] = 0xAA
        frame[1] = 0x55
        frame[2] = 0x03
        frame[3] = self.chan_byte
        checksum = 0
        for i in range(4):
            checksum ^= frame[i]
        frame[4] = checksum

        if self.worker.send_data(bytes(frame)):
            hx = ' '.join(f'{b:02X}' for b in frame)
            self._log(f"[{self.title()}] 停止脉冲: {hx}")

    def _send_direction(self):
        if not self.worker or not self.worker.connected:
            QMessageBox.warning(self, "警告", "请先连接到设备！")
            return

        direction = self.dir_combo.currentIndex()
        frame = bytearray(6)
        frame[0] = 0xAA
        frame[1] = 0x55
        frame[2] = 0x04
        frame[3] = self.chan_byte
        frame[4] = direction
        checksum = 0
        for i in range(5):
            checksum ^= frame[i]
        frame[5] = checksum

        dir_name = "HIGH" if direction else "LOW"
        if self.worker.send_data(bytes(frame)):
            hx = ' '.join(f'{b:02X}' for b in frame)
            self._log(f"[{self.title()}] 方向 → {dir_name}: {hx}")


# ==================== 主窗口 ====================

class TCPClientWindow(QMainWindow):
    CHANNEL_COLORS = ["#2196F3", "#4CAF50", "#FF9800", "#9C27B0",
                      "#00BCD4", "#8BC34A", "#FF5722", "#E91E63"]

    CHANNELS = [
        ("电机1 (0x01)", 0x01, "PIN64", "PIN65"),
        ("电机2 (0x02)", 0x02, "PIN66", "PIN67"),
        ("电机3 (0x03)", 0x03, "PIN68", "PIN69"),
        ("电机4 (0x04)", 0x04, "PIN70", "PIN71"),
        ("电机5 (0x05)", 0x05, "PIN98", "PIN99"),
        ("电机6 (0x06)", 0x06, "PIN100", "PIN101"),
        ("电机7 (0x07)", 0x07, "PIN103", "PIN104"),
        ("电机8 (0x08)", 0x08, "PIN105", "PIN106"),
    ]

    def __init__(self):
        super().__init__()
        self.worker = TCPWorker()
        self.channels = []
        self.rx_buffer = bytearray()
        self.init_ui()
        self.setup_connections()

    def init_ui(self):
        self.setWindowTitle("八电机独立脉冲控制")
        self.setMinimumSize(1500, 850)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # ========== 连接设置 ==========
        conn_group = QGroupBox("连接设置")
        conn_layout = QHBoxLayout()

        conn_layout.addWidget(QLabel("IP地址:"))
        self.ip_edit = QLineEdit("192.168.123.98")
        self.ip_edit.setMaximumWidth(150)
        conn_layout.addWidget(self.ip_edit)

        conn_layout.addWidget(QLabel("端口:"))
        self.port_spin = QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(5000)
        self.port_spin.setMaximumWidth(80)
        conn_layout.addWidget(self.port_spin)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.setMaximumWidth(80)
        conn_layout.addWidget(self.connect_btn)

        self.status_label = QLabel("状态: 未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        conn_layout.addWidget(self.status_label)
        conn_layout.addStretch()

        conn_group.setLayout(conn_layout)
        main_layout.addWidget(conn_group)

        # ========== 八通道控制面板 (2×4 网格) ==========
        pulse_group = QGroupBox("脉冲控制 (八通道独立)")
        grid = QGridLayout()
        grid.setSpacing(10)

        for idx, (name, chan_byte, pulse_pin, dir_pin) in enumerate(self.CHANNELS):
            color = self.CHANNEL_COLORS[idx]
            panel = ChannelPanel(name, chan_byte, pulse_pin, dir_pin, color)
            panel.set_worker(self.worker, self.append_log)
            self.channels.append(panel)
            grid.addWidget(panel, idx // 4, idx % 4)  # 2行 × 4列

        pulse_group.setLayout(grid)
        main_layout.addWidget(pulse_group, 3)  # stretch=3: 脉冲控制独占更多高度（通道面板变高）

        # ========== 输入端口状态 (12路) ==========
        input_group = QGroupBox("输入端口状态 (12路)")
        input_layout = QGridLayout()
        input_layout.setHorizontalSpacing(4)
        input_layout.setVerticalSpacing(12)  # 上下两行(1↕7,2↕8...)拉开，不贴在一起

        self.INPUT_PINS = ["PIN110", "PIN111", "PIN112", "PIN113", "PIN114", "PIN115",
                           "PIN119", "PIN120", "PIN121", "PIN124", "PIN125", "PIN126"]
        self.input_indicators = []
        for i in range(12):
            lbl = QLabel(f"输入{i + 1}\n{self.INPUT_PINS[i]}\nLOW")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setMinimumSize(52, 38)
            lbl.setStyleSheet(
                "background-color: #555555; color: #AAAAAA;"
                "border-radius: 4px; font-weight: bold; font-size: 9px;"
            )
            input_layout.addWidget(lbl, i // 6, i % 6)  # 2行 × 6列
            self.input_indicators.append(lbl)

        input_group.setLayout(input_layout)
        input_group.setMaximumHeight(125)  # 压缩高度（留出两行间隙）
        main_layout.addWidget(input_group, 0)

        # ========== RS485 通信 (485-1 / 485-2) ==========
        rs485_group = QGroupBox("RS485 通信")
        rs485_layout = QHBoxLayout()
        rs485_layout.setSpacing(12)

        self.rs485_inputs = []
        for port_id, port_name in [(0x12, "485-1"), (0x13, "485-2")]:
            panel = QWidget()
            panel_layout = QVBoxLayout()
            panel_layout.setContentsMargins(4, 4, 4, 4)
            panel_layout.setSpacing(4)

            header = QLabel(port_name)
            header.setStyleSheet("font-weight: bold; font-size: 11px;")
            header.setAlignment(Qt.AlignCenter)
            panel_layout.addWidget(header)

            # 方向模式选择
            mode_row = QHBoxLayout()
            mode_row.addWidget(QLabel("模式:"))
            mode_combo = QComboBox()
            mode_combo.addItems(["RX", "TX"])
            mode_combo.setCurrentIndex(0)  # 默认 RX（接收）
            mode_combo.currentIndexChanged.connect(
                lambda idx, pid=port_id: self._send_485_mode(pid, idx))
            mode_row.addWidget(mode_combo)
            panel_layout.addLayout(mode_row)

            row = QHBoxLayout()
            edit = QLineEdit()
            edit.setPlaceholderText("输入要发送的数据...")
            row.addWidget(edit)
            btn = QPushButton("发送")
            btn.setMaximumWidth(50)
            btn.clicked.connect(lambda checked, pid=port_id, ed=edit: self._send_rs485(pid, ed))
            row.addWidget(btn)
            panel_layout.addLayout(row)

            panel.setLayout(panel_layout)
            rs485_layout.addWidget(panel)
            self.rs485_inputs.append((port_id, edit))

        rs485_group.setLayout(rs485_layout)
        rs485_group.setMaximumHeight(105)  # 压缩高度
        main_layout.addWidget(rs485_group, 0)

        # ========== 接收日志 ==========
        recv_group = QGroupBox("接收数据")
        recv_layout = QVBoxLayout()

        self.recv_text = QTextEdit()
        self.recv_text.setReadOnly(True)
        self.recv_text.setFont(QFont("Consolas", 10))
        recv_layout.addWidget(self.recv_text)

        recv_ctrl_layout = QHBoxLayout()
        self.clear_recv_btn = QPushButton("清空接收")
        self.clear_recv_btn.setMaximumWidth(100)
        recv_ctrl_layout.addWidget(self.clear_recv_btn)

        self.auto_scroll_check = QPushButton("自动滚动: 开")
        self.auto_scroll_check.setMaximumWidth(100)
        self.auto_scroll_check.setCheckable(True)
        self.auto_scroll_check.setChecked(True)
        self.auto_scroll_check.setStyleSheet("background-color: #4CAF50; color: white;")
        recv_ctrl_layout.addWidget(self.auto_scroll_check)
        recv_ctrl_layout.addStretch()

        recv_layout.addLayout(recv_ctrl_layout)
        recv_group.setLayout(recv_layout)
        recv_group.setMaximumHeight(135)  # 压缩高度
        main_layout.addWidget(recv_group, 0)

    def setup_connections(self):
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.clear_recv_btn.clicked.connect(lambda: self.recv_text.clear())
        self.worker.data_received.connect(self.on_data_received)
        self.worker.raw_data_received.connect(self._parse_raw_data)
        self.worker.connection_status.connect(self.on_connection_status)
        self.worker.error_occurred.connect(self.on_error)

    def toggle_connection(self):
        if not self.worker.connected:
            host = self.ip_edit.text()
            port = self.port_spin.value()
            if self.worker.connect_to_server(host, port):
                self.connect_btn.setText("断开")
                self.connect_btn.setStyleSheet("background-color: #f44336; color: white;")
                self.status_label.setText("状态: 已连接")
                self.status_label.setStyleSheet("color: green; font-weight: bold;")
                if not self.worker.isRunning():
                    self.worker.start()  # 仅首次启动线程
        else:
            self.worker.disconnect()
            self.connect_btn.setText("连接")
            self.connect_btn.setStyleSheet("")
            self.status_label.setText("状态: 未连接")
            self.status_label.setStyleSheet("color: red; font-weight: bold;")

    def on_data_received(self, data):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.recv_text.append(f"[{timestamp}] {data}")
        if self.auto_scroll_check.isChecked():
            self.recv_text.moveCursor(QTextCursor.End)
            self.recv_text.ensureCursorVisible()

    def on_connection_status(self, status):
        self.append_log(f"[状态] {status}")

    def on_error(self, error):
        self.append_log(f"[错误] {error}")
        QMessageBox.warning(self, "错误", error)

    def append_log(self, message):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.recv_text.append(f"[{timestamp}] {message}")

    def _parse_raw_data(self, data: bytes):
        """解析原始二进制帧，提取 0x10 输入状态帧 和 0x14 485接收帧"""
        self.rx_buffer.extend(data)
        while len(self.rx_buffer) >= 6:
            # 0x10 输入状态帧 (6字节): AA 55 10 <lo> <hi> <xor>
            idx = self.rx_buffer.find(b'\xAA\x55\x10')
            if idx >= 0 and idx + 6 <= len(self.rx_buffer):
                frame = self.rx_buffer[idx:idx + 6]
                checksum = frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4]
                if checksum == frame[5]:
                    self._update_input_status(frame[3] | (frame[4] << 8))
                self.rx_buffer = self.rx_buffer[idx + 6:]  # 完整消费6字节帧
                continue

            # 0x14 485接收帧 (6字节)
            idx14 = self.rx_buffer.find(b'\xAA\x55\x14')
            if idx14 >= 0 and idx14 + 6 <= len(self.rx_buffer):
                frame = self.rx_buffer[idx14:idx14 + 6]
                checksum = frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4]
                if checksum == frame[5]:
                    port = "485-1" if frame[3] == 0x01 else "485-2"
                    self.append_log(f"[{port} RX] 0x{frame[4]:02X} ('{chr(frame[4]) if 0x20 <= frame[4] < 0x7F else '?'}')")
                self.rx_buffer = self.rx_buffer[idx14 + 6:]  # 完整消费6字节帧
                continue

            # 未找到完整已知帧: 保留可能的帧头起始, 等待更多数据
            idxh = self.rx_buffer.rfind(b'\xAA\x55')
            if idxh > 0:
                self.rx_buffer = self.rx_buffer[idxh:]
            elif idxh < 0:
                self.rx_buffer = self.rx_buffer[-1:]
            break

    def _update_input_status(self, status: int):
        """更新 12 个输入端口指示灯 (bit0..11 = 输入1..12)"""
        for i in range(12):
            high = (status >> i) & 1
            if high:
                text = f"输入{i + 1}\n{self.INPUT_PINS[i]}\nHIGH"
                color = "#4CAF50"
                text_color = "white"
            else:
                text = f"输入{i + 1}\n{self.INPUT_PINS[i]}\nLOW"
                color = "#555555"
                text_color = "#AAAAAA"
            self.input_indicators[i].setText(text)
            self.input_indicators[i].setStyleSheet(
                f"background-color: {color}; color: {text_color};"
                "border-radius: 4px; font-weight: bold; font-size: 9px;"
            )

    def _send_rs485(self, port_id: int, edit: QLineEdit):
        """发送 RS485 数据 (0x12→485-1, 0x13→485-2)"""
        if not self.worker or not self.worker.connected:
            QMessageBox.warning(self, "警告", "请先连接到设备！")
            return
        text = edit.text()
        if not text:
            return
        raw = text.encode('utf-8', errors='ignore')[:255]
        if len(raw) == 0:
            return
        frame = bytearray(5 + len(raw))
        frame[0] = 0xAA
        frame[1] = 0x55
        frame[2] = port_id
        frame[3] = len(raw)
        for i, b in enumerate(raw):
            frame[4 + i] = b
        checksum = 0
        for i in range(4 + len(raw)):
            checksum ^= frame[i]
        frame[4 + len(raw)] = checksum

        port_name = "485-1" if port_id == 0x12 else "485-2"
        if self.worker.send_data(bytes(frame)):
            self.append_log(f"[{port_name} TX] {text}")
            edit.clear()

    def _send_485_mode(self, port_id: int, mode_idx: int):
        """设置 485 方向模式 (0x15) — port_id 区分 485-1/485-2"""
        if not self.worker or not self.worker.connected:
            return
        # port_id 是发送命令号 (0x12/0x13), 转为 485 端口号
        port = 0x01 if port_id == 0x12 else 0x03
        # combo 索引 -> 协议模式值 (原 AUTO=0 已移除, RX=1, TX=2)
        MODE_VALS  = [1, 2]
        MODE_NAMES = ["RX", "TX"]
        frame = bytearray(6)
        frame[0] = 0xAA
        frame[1] = 0x55
        frame[2] = 0x15
        frame[3] = port
        frame[4] = MODE_VALS[mode_idx]
        frame[5] = frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4]

        port_name = "485-1" if port == 0x01 else "485-2"
        if self.worker.send_data(bytes(frame)):
            self.append_log(f"[{port_name}] 模式 → {MODE_NAMES[mode_idx]}")

    def closeEvent(self, event):
        self.worker.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = TCPClientWindow()
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
