import struct
import sys
import time
import math
from collections import deque
from datetime import datetime

import serial
from serial.tools import list_ports
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg


MODE_TEXT = {
    0: "占空比模式",
    1: "电流环模式",
    2: "电流刹车模式",
    3: "转速模式",
    4: "位置模式",
    5: "设置原点模式",
    6: "位置速度环模式",
}
KT_TMOTOR = 0.73  # tau(Nm) = iq(A) * Kt


class MotorBleGui(QtWidgets.QWidget):
    FRAME_LEN = 32
    HEADER = b"\xA5\x5A\x20"

    def __init__(self):
        super().__init__()
        self.ser = None
        self.rx_buf = b""
        self.target_id = 104
        self.current_mode = 1
        self.wave_enabled = False
        self.wave_t0 = time.monotonic()
        self.last_wave_send = 0.0
        self.wave_cycles_target = -1.0
        self.last_cmd_torque = 0.0
        self.plot_t0 = time.monotonic()
        self.buf_t = deque(maxlen=6000)
        self.buf_pos = deque(maxlen=6000)
        self.buf_spd = deque(maxlen=6000)
        self.buf_cmd = deque(maxlen=6000)
        self.buf_cur = deque(maxlen=6000)

        self.setWindowTitle("T-Motor Servo 7模式调试")
        self.resize(1180, 760)
        self._build_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start()

    def _build_ui(self):
        root = QtWidgets.QHBoxLayout(self)
        left = QtWidgets.QVBoxLayout()
        right = QtWidgets.QVBoxLayout()
        root.addLayout(left, 5)
        root.addLayout(right, 2)

        row1 = QtWidgets.QHBoxLayout()
        left.addLayout(row1)
        self.cmb_port = QtWidgets.QComboBox()
        self.btn_refresh = QtWidgets.QPushButton("刷新串口")
        self.btn_conn = QtWidgets.QPushButton("连接")
        row1.addWidget(QtWidgets.QLabel("蓝牙串口"))
        row1.addWidget(self.cmb_port, 1)
        row1.addWidget(self.btn_refresh)
        row1.addWidget(self.btn_conn)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn.clicked.connect(self.toggle_connect)
        self.refresh_ports()

        ctl = QtWidgets.QHBoxLayout()
        left.addLayout(ctl)
        self.cmb_id = QtWidgets.QComboBox()
        self.cmb_id.addItems(["104", "105"])
        self.btn_set_id = QtWidgets.QPushButton("切换ID并初始化")
        self.btn_init = QtWidgets.QPushButton("初始化电机")
        self.btn_exec = QtWidgets.QPushButton("执行当前模式")
        self.btn_origin = QtWidgets.QPushButton("电机原点归零")
        self.btn_stop = QtWidgets.QPushButton("STOP(0A)")
        ctl.addWidget(QtWidgets.QLabel("目标ID"))
        ctl.addWidget(self.cmb_id)
        ctl.addWidget(self.btn_set_id)
        ctl.addWidget(self.btn_init)
        ctl.addWidget(self.btn_exec)
        ctl.addWidget(self.btn_origin)
        ctl.addWidget(self.btn_stop)
        self.btn_set_id.clicked.connect(self.send_set_id)
        self.btn_init.clicked.connect(self.send_init)
        self.btn_exec.clicked.connect(self.send_execute)
        self.btn_origin.clicked.connect(self.send_origin)
        self.btn_stop.clicked.connect(self.send_stop)

        status_box = QtWidgets.QGroupBox("状态")
        status_layout = QtWidgets.QHBoxLayout(status_box)
        left.addWidget(status_box)
        self.lbl_conn = QtWidgets.QLabel("未连接")
        self.lbl_can = QtWidgets.QLabel("CAN没数据返回")
        self.lbl_can.setStyleSheet("color:#c62828;font-weight:bold;")
        self.lbl_id = QtWidgets.QLabel("-")
        self.lbl_mode = QtWidgets.QLabel("-")
        self.lbl_last = QtWidgets.QLabel("-")
        status_layout.addWidget(QtWidgets.QLabel("连接:"))
        status_layout.addWidget(self.lbl_conn)
        status_layout.addSpacing(8)
        status_layout.addWidget(QtWidgets.QLabel("CAN:"))
        status_layout.addWidget(self.lbl_can)
        status_layout.addSpacing(8)
        status_layout.addWidget(QtWidgets.QLabel("ID:"))
        status_layout.addWidget(self.lbl_id)
        status_layout.addSpacing(8)
        status_layout.addWidget(QtWidgets.QLabel("模式:"))
        status_layout.addWidget(self.lbl_mode)
        status_layout.addSpacing(8)
        status_layout.addWidget(QtWidgets.QLabel("最近:"))
        status_layout.addWidget(self.lbl_last, 1)

        data_box = QtWidgets.QGroupBox("实时数据")
        data_layout = QtWidgets.QHBoxLayout(data_box)
        left.addWidget(data_box)
        self.data = {}
        self.data_order = [
            ("pos", "位置"),
            ("pos_abs", "绝对位置"),
            ("spd", "转速"),
            ("cur", "实际电流"),
            ("cmd", "期望扭矩"),
            ("temp", "温度"),
            ("err", "故障"),
            ("rx", "RX"),
            ("tx", "TX"),
            ("delta", "ΔRX"),
        ]
        for key, text in self.data_order:
            col = QtWidgets.QVBoxLayout()
            name = QtWidgets.QLabel(text)
            name.setStyleSheet("color:#666; font-size:11px;")
            lbl = QtWidgets.QLabel("N/A")
            lbl.setStyleSheet("font-weight:600;")
            self.data[key] = lbl
            col.addWidget(name)
            col.addWidget(lbl)
            data_layout.addLayout(col)

        self.lbl_sent = QtWidgets.QLabel("已下发: -")
        left.addWidget(self.lbl_sent)

        mode_box = QtWidgets.QGroupBox("7种控制模式")
        mode_layout = QtWidgets.QVBoxLayout(mode_box)
        right.addWidget(mode_box)

        btn_grid = QtWidgets.QGridLayout()
        self.mode_buttons = {}
        for m in range(7):
            b = QtWidgets.QPushButton(f"{m} - {MODE_TEXT[m]}")
            b.clicked.connect(lambda _, mm=m: self.select_mode(mm))
            self.mode_buttons[m] = b
            btn_grid.addWidget(b, m // 2, m % 2)
        mode_layout.addLayout(btn_grid)

        self.param_stack = QtWidgets.QStackedWidget()
        self.mode_inputs = {}

        def add_mode_page(mode, fields):
            page = QtWidgets.QWidget()
            form = QtWidgets.QFormLayout(page)
            widgets = []
            for name, mn, mx, step, default in fields:
                sb = QtWidgets.QDoubleSpinBox()
                sb.setRange(mn, mx)
                sb.setSingleStep(step)
                sb.setDecimals(4)
                sb.setValue(default)
                form.addRow(name, sb)
                widgets.append(sb)
            self.mode_inputs[mode] = widgets
            self.param_stack.addWidget(page)

        add_mode_page(0, [("占空比", -1.0, 1.0, 0.01, 0.0)])
        add_mode_page(1, [("扭矩(Nm)", -30.0, 30.0, 0.1, 0.0)])
        add_mode_page(2, [("刹车电流(A)", -60.0, 60.0, 0.1, 0.0)])
        add_mode_page(3, [("转速(rpm)", -400000.0, 400000.0, 10.0, 0.0)])
        add_mode_page(4, [("位置(协议单位)", -2000.0, 2000.0, 0.01, 0.0)])
        add_mode_page(5, [])
        add_mode_page(6, [
            ("位置(协议单位)", -2000.0, 2000.0, 0.01, 0.0),
            ("速度(rpm)", -32767.0, 32767.0, 10.0, 1000.0),
            ("加速度", -32767.0, 32767.0, 10.0, 3000.0),
        ])

        mode_layout.addWidget(self.param_stack)
        self.select_mode(1)

        wave_box = QtWidgets.QGroupBox("扭矩波形（仅模式1）")
        wave_form = QtWidgets.QFormLayout(wave_box)
        self.cmb_wave = QtWidgets.QComboBox()
        self.cmb_wave.addItems(["常值", "正弦", "三角"])
        self.sb_amp = QtWidgets.QDoubleSpinBox()
        self.sb_amp.setRange(0.0, 15.0)
        self.sb_amp.setDecimals(3)
        self.sb_amp.setSingleStep(0.1)
        self.sb_amp.setValue(1.0)
        self.sb_freq = QtWidgets.QDoubleSpinBox()
        self.sb_freq.setRange(0.01, 20.0)
        self.sb_freq.setDecimals(3)
        self.sb_freq.setSingleStep(0.1)
        self.sb_freq.setValue(0.5)
        self.sb_bias = QtWidgets.QDoubleSpinBox()
        self.sb_bias.setRange(-30.0, 30.0)
        self.sb_bias.setDecimals(3)
        self.sb_bias.setSingleStep(0.1)
        self.sb_bias.setValue(0.0)
        self.sb_cycles = QtWidgets.QDoubleSpinBox()
        self.sb_cycles.setRange(0.1, 1000.0)
        self.sb_cycles.setDecimals(2)
        self.sb_cycles.setSingleStep(0.5)
        self.sb_cycles.setValue(1.0)

        wave_btn_row = QtWidgets.QHBoxLayout()
        self.btn_wave_start = QtWidgets.QPushButton("开始波形")
        self.btn_wave_cycles = QtWidgets.QPushButton("发送N周期")
        self.btn_wave_stop = QtWidgets.QPushButton("停止波形")
        wave_btn_row.addWidget(self.btn_wave_start)
        wave_btn_row.addWidget(self.btn_wave_cycles)
        wave_btn_row.addWidget(self.btn_wave_stop)

        wave_form.addRow("类型", self.cmb_wave)
        wave_form.addRow("幅值Nm(半Vpp)", self.sb_amp)
        wave_form.addRow("频率Hz", self.sb_freq)
        wave_form.addRow("偏置Nm", self.sb_bias)
        wave_form.addRow("N周期", self.sb_cycles)
        wave_form.addRow(wave_btn_row)
        mode_layout.addWidget(wave_box)

        self.btn_wave_start.clicked.connect(self.start_wave)
        self.btn_wave_cycles.clicked.connect(self.send_wave_cycles)
        self.btn_wave_stop.clicked.connect(self.stop_wave)

        plot_box = QtWidgets.QGroupBox("实时曲线")
        plot_layout = QtWidgets.QVBoxLayout(plot_box)
        top_tools = QtWidgets.QHBoxLayout()
        self.sb_horizon = QtWidgets.QDoubleSpinBox()
        self.sb_horizon.setRange(1.0, 120.0)
        self.sb_horizon.setDecimals(1)
        self.sb_horizon.setSingleStep(1.0)
        self.sb_horizon.setValue(15.0)
        top_tools.addWidget(QtWidgets.QLabel("Horizon(s)"))
        top_tools.addWidget(self.sb_horizon)
        top_tools.addStretch(1)
        legend_txt = "蓝=位置  橙=转速  绿=期望扭矩  红=实际扭矩"
        self.lbl_legend = QtWidgets.QLabel(legend_txt)
        self.lbl_legend.setStyleSheet("font-weight:600;")
        top_tools.addWidget(self.lbl_legend)
        plot_layout.addLayout(top_tools)
        vis_row = QtWidgets.QHBoxLayout()
        self.chk_pos = QtWidgets.QCheckBox("位置")
        self.chk_spd = QtWidgets.QCheckBox("转速")
        self.chk_cmd = QtWidgets.QCheckBox("期望扭矩")
        self.chk_cur = QtWidgets.QCheckBox("实际扭矩")
        self.chk_pos.setChecked(True)
        self.chk_spd.setChecked(True)
        self.chk_cmd.setChecked(True)
        self.chk_cur.setChecked(True)
        self.chk_pos.setStyleSheet("color: rgb(33,150,243); font-weight:600;")
        self.chk_spd.setStyleSheet("color: rgb(255,152,0); font-weight:600;")
        self.chk_cmd.setStyleSheet("color: rgb(76,175,80); font-weight:600;")
        self.chk_cur.setStyleSheet("color: rgb(244,67,54); font-weight:600;")
        vis_row.addWidget(self.chk_pos)
        vis_row.addWidget(self.chk_spd)
        vis_row.addWidget(self.chk_cmd)
        vis_row.addWidget(self.chk_cur)
        vis_row.addStretch(1)
        plot_layout.addLayout(vis_row)

        self.plot_pos = pg.PlotWidget(title="位置")
        self.plot_pos.showGrid(x=True, y=True, alpha=0.25)
        self.curve_pos = self.plot_pos.plot(pen=pg.mkPen((33, 150, 243), width=2), name="位置")

        self.plot_spd = pg.PlotWidget(title="转速")
        self.plot_spd.showGrid(x=True, y=True, alpha=0.25)
        self.curve_spd = self.plot_spd.plot(pen=pg.mkPen((255, 152, 0), width=2), name="转速")

        self.plot_cur = pg.PlotWidget(title="期望扭矩/实际扭矩")
        self.plot_cur.showGrid(x=True, y=True, alpha=0.25)
        self.curve_cmd = self.plot_cur.plot(pen=pg.mkPen((76, 175, 80), width=2), name="期望")
        self.curve_act = self.plot_cur.plot(pen=pg.mkPen((244, 67, 54), width=2), name="实际")
        plot_layout.addWidget(self.plot_pos)
        plot_layout.addWidget(self.plot_spd)
        plot_layout.addWidget(self.plot_cur)
        left.addWidget(plot_box, 12)
        self.chk_pos.toggled.connect(self.curve_pos.setVisible)
        self.chk_spd.toggled.connect(self.curve_spd.setVisible)
        self.chk_cmd.toggled.connect(self.curve_cmd.setVisible)
        self.chk_cur.toggled.connect(self.curve_act.setVisible)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        right.addWidget(self.log, 1)

    def log_line(self, text):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log.appendPlainText(f"[{ts}] {text}")

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.cmb_port.clear()
        self.cmb_port.addItems(ports)

    def toggle_connect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.lbl_conn.setText("未连接")
            self.btn_conn.setText("连接")
            self.log_line("已断开")
            return

        port = self.cmb_port.currentText().strip()
        if not port:
            self.log_line("没有可用串口")
            return

        try:
            self.ser = serial.Serial(port, 115200, timeout=0)
            self.lbl_conn.setText(f"已连接 {port}")
            self.btn_conn.setText("断开")
            self.rx_buf = b""
            self.log_line(f"连接成功: {port}")
            self.send_init()
        except Exception as exc:
            self.log_line(f"连接失败: {exc}")
            self.ser = None

    def select_mode(self, mode):
        self.current_mode = mode
        self.param_stack.setCurrentIndex(mode)
        for m, b in self.mode_buttons.items():
            if m == mode:
                b.setStyleSheet("background:#1976d2;color:white;font-weight:bold;")
            else:
                b.setStyleSheet("")

    def _current_params(self):
        vals = [0.0, 0.0, 0.0]
        w = self.mode_inputs.get(self.current_mode, [])
        for i in range(min(3, len(w))):
            vals[i] = float(w[i].value())
        return vals[0], vals[1], vals[2]

    def _build_payload(self, flags, mode=0, p1=0.0, p2=0.0, p3=0.0):
        self.target_id = int(self.cmb_id.currentText())
        payload = bytearray(29)
        payload[0] = ord('C')
        payload[1] = flags & 0xFF
        payload[2] = self.target_id
        payload[3] = mode & 0x07
        payload[4:8] = struct.pack('<f', float(p1))
        payload[8:12] = struct.pack('<f', float(p2))
        payload[12:16] = struct.pack('<f', float(p3))
        return bytes(payload)

    def _send_frame(self, payload):
        if not self.ser or not self.ser.is_open:
            return
        try:
            self.ser.write(self.HEADER + payload)
        except Exception as exc:
            self.log_line(f"发送失败: {exc}")

    def send_init(self):
        self._send_frame(self._build_payload(0x04))
        self.log_line("> INIT")

    def send_set_id(self):
        self._send_frame(self._build_payload(0x01 | 0x04))
        self.log_line(f"> SET_ID+INIT id={self.target_id}")

    def send_stop(self):
        self.wave_enabled = False
        self.wave_cycles_target = -1.0
        self.last_cmd_torque = 0.0
        self._send_frame(self._build_payload(0x08))
        self.log_line("> STOP")

    def send_origin(self):
        self._send_frame(self._build_payload(0x20))
        self.log_line("> ORIGIN_HERE")

    def send_execute(self):
        self.wave_enabled = False
        p1, p2, p3 = self._current_params()
        self._send_frame(self._build_payload(0x40, self.current_mode, p1, p2, p3))
        if self.current_mode == 1:
            self.last_cmd_torque = p1
        self.lbl_sent.setText(
            f"已下发: mode={self.current_mode}({MODE_TEXT[self.current_mode]}) p1={p1:.4f} p2={p2:.4f} p3={p3:.4f}"
        )
        self.log_line(f"> EXEC mode={self.current_mode} p1={p1:.4f} p2={p2:.4f} p3={p3:.4f}")

    def _send_torque_cmd(self, tau):
        tau = max(-30.0, min(30.0, float(tau)))
        self.last_cmd_torque = tau
        self._send_frame(self._build_payload(0x40, 1, tau, 0.0, 0.0))

    def _wave_target(self):
        amp = float(self.sb_amp.value())
        freq = float(self.sb_freq.value())
        bias = float(self.sb_bias.value())
        typ = self.cmb_wave.currentText()
        t = time.monotonic() - self.wave_t0
        if typ == "正弦":
            y = math.sin(2.0 * math.pi * freq * t)
        elif typ == "三角":
            y = (2.0 / math.pi) * math.asin(math.sin(2.0 * math.pi * freq * t))
        else:
            y = 1.0
        return bias + amp * y

    def start_wave(self):
        self.current_mode = 1
        self.select_mode(1)
        self.wave_t0 = time.monotonic()
        self.wave_enabled = True
        self.wave_cycles_target = -1.0
        self.log_line("> WAVE START")

    def stop_wave(self):
        self.wave_enabled = False
        self.wave_cycles_target = -1.0
        self.log_line("> WAVE STOP")

    def send_wave_cycles(self):
        self.current_mode = 1
        self.select_mode(1)
        self.wave_t0 = time.monotonic()
        self.wave_enabled = True
        self.wave_cycles_target = float(self.sb_cycles.value())
        self.log_line(f"> WAVE {self.wave_cycles_target:.2f} cycles")

    def _update_plots(self):
        if len(self.buf_t) < 2:
            return
        x = list(self.buf_t)
        y_pos = list(self.buf_pos)
        y_spd = list(self.buf_spd)
        y_cmd = list(self.buf_cmd)
        y_cur = list(self.buf_cur)
        self.curve_pos.setData(x, y_pos)
        self.curve_spd.setData(x, y_spd)
        self.curve_cmd.setData(x, y_cmd)
        self.curve_act.setData(x, y_cur)
        xmax = x[-1]
        xmin = max(0.0, xmax - float(self.sb_horizon.value()))
        self.plot_pos.setXRange(xmin, xmax, padding=0.0)
        self.plot_spd.setXRange(xmin, xmax, padding=0.0)
        self.plot_cur.setXRange(xmin, xmax, padding=0.0)

    def _on_tick(self):
        self._poll_serial()
        now = time.monotonic()
        if self.wave_enabled and self.ser and self.ser.is_open:
            if now - self.last_wave_send >= 0.02:  # 50Hz
                self.last_wave_send = now
                target = self._wave_target()
                self._send_torque_cmd(target)
            if self.wave_cycles_target > 0:
                cycles_now = (now - self.wave_t0) * float(self.sb_freq.value())
                if cycles_now >= self.wave_cycles_target:
                    self.wave_enabled = False
                    self.wave_cycles_target = -1.0
                    self.log_line("> WAVE DONE")
        self._update_plots()

    def _fmt_i16_na(self, raw, scale):
        if raw == -32768:
            return "N/A"
        return f"{raw / scale:.2f}"

    def _handle_frame(self, frame):
        if len(frame) != 32:
            return
        payload = frame[3:]

        t_cs = struct.unpack_from('<H', payload, 0)[0]
        pos_x10 = struct.unpack_from('<h', payload, 2)[0]
        spd_div10 = struct.unpack_from('<h', payload, 4)[0]
        cur_x100 = struct.unpack_from('<h', payload, 6)[0]
        temp_raw = struct.unpack_from('<b', payload, 8)[0]
        err_u8 = payload[9]
        active_id = payload[10]
        can_ok = payload[11]
        cmd_x100 = struct.unpack_from('<h', payload, 12)[0]
        rx_total = struct.unpack_from('<I', payload, 14)[0]
        tx_total = struct.unpack_from('<I', payload, 18)[0]
        rx_delta = struct.unpack_from('<H', payload, 22)[0]
        pos_zeroed_x10 = struct.unpack_from('<i', payload, 24)[0]
        mode_rx = payload[28] & 0x07

        self.lbl_last.setText(f"t={t_cs/100.0:.2f}s")
        self.lbl_id.setText(str(active_id))
        self.lbl_mode.setText(f"{mode_rx} - {MODE_TEXT.get(mode_rx, '未知')}")
        self.lbl_can.setText("CAN正常" if can_ok else "CAN没数据返回")
        self.lbl_can.setStyleSheet(
            "color:#2e7d32;font-weight:bold;" if can_ok else "color:#c62828;font-weight:bold;"
        )

        self.data["cmd"].setText("N/A" if cmd_x100 == -32768 else f"{cmd_x100/100.0:.2f}")
        self.data["pos_abs"].setText(self._fmt_i16_na(pos_x10, 10.0))
        self.data["pos"].setText(f"{pos_zeroed_x10/10.0:.2f}")
        self.data["spd"].setText("N/A" if spd_div10 == -32768 else f"{spd_div10 * 10.0:.1f}")
        self.data["cur"].setText(self._fmt_i16_na(cur_x100, 100.0))
        self.data["temp"].setText("N/A" if temp_raw == -128 else f"{temp_raw:.1f}")
        self.data["err"].setText("N/A" if err_u8 == 255 else str(err_u8))
        self.data["rx"].setText(str(rx_total))
        self.data["tx"].setText(str(tx_total))
        self.data["delta"].setText(str(rx_delta))

        t_plot = time.monotonic() - self.plot_t0
        pos_v = float("nan") if pos_x10 == -32768 else (pos_zeroed_x10 / 10.0)
        spd_v = float("nan") if spd_div10 == -32768 else (spd_div10 * 10.0)
        cur_v = float("nan") if cur_x100 == -32768 else (cur_x100 / 100.0)
        self.buf_t.append(t_plot)
        self.buf_pos.append(pos_v)
        self.buf_spd.append(spd_v)
        self.buf_cmd.append(self.last_cmd_torque)
        tau_v = float("nan") if math.isnan(cur_v) else (cur_v * KT_TMOTOR)
        self.buf_cur.append(tau_v)

    def _poll_serial(self):
        if not self.ser or not self.ser.is_open:
            return
        try:
            chunk = self.ser.read(4096)
        except Exception as exc:
            self.log_line(f"读取失败: {exc}")
            return

        if not chunk:
            return
        self.rx_buf += chunk

        while True:
            idx = self.rx_buf.find(self.HEADER)
            if idx < 0:
                if len(self.rx_buf) > 512:
                    self.rx_buf = self.rx_buf[-128:]
                return
            if len(self.rx_buf) < idx + self.FRAME_LEN:
                if idx > 0:
                    self.rx_buf = self.rx_buf[idx:]
                return
            frame = self.rx_buf[idx: idx + self.FRAME_LEN]
            self.rx_buf = self.rx_buf[idx + self.FRAME_LEN:]
            self._handle_frame(frame)


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MotorBleGui()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
