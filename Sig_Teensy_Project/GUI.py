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
    0: "力矩控制",
    1: "速度控制",
    2: "位置控制",
}

AXIS_STATE_TEXT = {
    0: "未定义",
    1: "空闲",
    3: "校准",
    4: "电机校准",
    7: "编码器校准",
    8: "闭环",
}

ERROR_TYPE_TEXT = {
    0: "电机异常",
    1: "编码器异常",
    3: "控制异常",
    4: "系统异常",
    255: "N/A",
}


class MotorBleGui(QtWidgets.QWidget):
    FRAME_LEN = 32
    HEADER = b"\xA5\x5A\x20"

    def __init__(self):
        super().__init__()
        self.ser = None
        self.rx_buf = b""

        self.target_id = 1
        self.current_mode = 0
        self.last_cmd_value = 0.0
        self.gear_ratio = 9.67
        self.torque_constant = 0.097
        self.trans_eff = 1.0
        self.wave_enabled = False
        self.wave_t0 = time.monotonic()
        self.last_wave_send = 0.0
        self.wave_cycles_target = -1.0

        self.plot_t0 = time.monotonic()
        self.buf_t = deque(maxlen=6000)
        self.buf_pos = deque(maxlen=6000)
        self.buf_spd = deque(maxlen=6000)
        self.buf_tau_set = deque(maxlen=6000)
        self.buf_tau = deque(maxlen=6000)
        self.buf_iq_set = deque(maxlen=6000)
        self.buf_iq = deque(maxlen=6000)

        self.setWindowTitle("Sig Motor CANSimple 调试")
        self.resize(1160, 760)
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
        self.cmb_id.addItems(["1", "2"])

        self.btn_set_id = QtWidgets.QPushButton("切换ID并初始化")
        self.btn_init = QtWidgets.QPushButton("初始化电机")
        self.btn_exec = QtWidgets.QPushButton("执行当前模式")
        self.btn_clear = QtWidgets.QPushButton("清错并重初始化")
        self.btn_stop = QtWidgets.QPushButton("STOP(0命令)")
        for b in [self.btn_set_id, self.btn_init, self.btn_exec, self.btn_clear, self.btn_stop]:
            b.setAutoDefault(False)
            b.setDefault(False)

        ctl.addWidget(QtWidgets.QLabel("目标ID"))
        ctl.addWidget(self.cmb_id)
        ctl.addWidget(self.btn_set_id)
        ctl.addWidget(self.btn_init)
        ctl.addWidget(self.btn_exec)
        ctl.addWidget(self.btn_clear)
        ctl.addWidget(self.btn_stop)

        self.btn_set_id.clicked.connect(self.send_set_id)
        self.btn_init.clicked.connect(self.send_init)
        self.btn_exec.clicked.connect(self.send_execute)
        self.btn_clear.clicked.connect(self.send_clear)
        self.btn_stop.clicked.connect(self.send_stop)

        status_box = QtWidgets.QGroupBox("状态")
        status_layout = QtWidgets.QHBoxLayout(status_box)
        left.addWidget(status_box)

        self.lbl_conn = QtWidgets.QLabel("未连接")
        self.lbl_can = QtWidgets.QLabel("CAN没数据返回")
        self.lbl_can.setStyleSheet("color:#c62828;font-weight:bold;")
        self.lbl_id = QtWidgets.QLabel("-")
        self.lbl_mode = QtWidgets.QLabel("-")
        self.lbl_axis_state = QtWidgets.QLabel("-")
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
        status_layout.addWidget(QtWidgets.QLabel("轴状态:"))
        status_layout.addWidget(self.lbl_axis_state)
        status_layout.addSpacing(8)
        status_layout.addWidget(QtWidgets.QLabel("最近:"))
        status_layout.addWidget(self.lbl_last, 1)
        self.lbl_faults = QtWidgets.QLabel("Faults: -")
        self.lbl_faults.setStyleSheet("font-weight:600; color:#5d4037;")
        left.addWidget(self.lbl_faults)

        data_box = QtWidgets.QGroupBox("实时数据")
        data_layout = QtWidgets.QGridLayout(data_box)
        data_layout.setHorizontalSpacing(10)
        data_layout.setVerticalSpacing(6)
        left.addWidget(data_box)

        self.data = {}
        self.data_order = [
            ("pos", "相对位置(rev)"),
            ("pos_abs", "绝对位置(rev)"),
            ("spd", "速度(rev/s)"),
            ("iq_set", "Iq目标(A)"),
            ("iq", "Iq实际(A)"),
            ("tau", "实测力矩(Nm)"),
            ("cmd", "目标力矩(Nm)"),
            ("axis_state", "Axis_State"),
            ("req_state", "请求状态(0x07)"),
            ("axis_error", "Axis_Error(0x01)"),
            ("hb_flags", "Heartbeat Flags"),
            ("detail_type", "Detail Type(0x03)"),
            ("detail_err", "Detail Error(0x03)"),
        ]

        items_per_row = 7  # 13项 -> 两行显示（7 + 6）
        for idx, (key, text) in enumerate(self.data_order):
            row = idx // items_per_row
            col_idx = idx % items_per_row

            col = QtWidgets.QVBoxLayout()
            col.setSpacing(1)
            name = QtWidgets.QLabel(text)
            name.setAlignment(QtCore.Qt.AlignCenter)
            name.setStyleSheet("color:#666; font-size:10px;")
            lbl = QtWidgets.QLabel("N/A")
            lbl.setAlignment(QtCore.Qt.AlignCenter)
            lbl.setStyleSheet("font-weight:600; font-size:11px;")
            self.data[key] = lbl
            col.addWidget(name)
            col.addWidget(lbl)
            data_layout.addLayout(col, row, col_idx)

        self.lbl_sent = QtWidgets.QLabel("已下发: -")
        left.addWidget(self.lbl_sent)

        mode_box = QtWidgets.QGroupBox("控制模式")
        mode_layout = QtWidgets.QVBoxLayout(mode_box)
        right.addWidget(mode_box)

        btn_grid = QtWidgets.QGridLayout()
        self.mode_buttons = {}
        for m in range(3):
            b = QtWidgets.QPushButton(f"{m} - {MODE_TEXT[m]}")
            b.clicked.connect(lambda _, mm=m: self.select_mode(mm))
            self.mode_buttons[m] = b
            btn_grid.addWidget(b, m, 0)
        mode_layout.addLayout(btn_grid)

        self.param_stack = QtWidgets.QStackedWidget()
        self.mode_inputs = {}

        def add_mode_page(mode, fields):
            page = QtWidgets.QWidget()
            form = QtWidgets.QFormLayout(page)
            widgets = []
            for name, mn, mx, step, default, decimals in fields:
                sb = QtWidgets.QDoubleSpinBox()
                sb.setRange(mn, mx)
                sb.setSingleStep(step)
                sb.setDecimals(decimals)
                sb.setValue(default)
                form.addRow(name, sb)
                widgets.append(sb)
            self.mode_inputs[mode] = widgets
            self.param_stack.addWidget(page)

        add_mode_page(0, [("目标力矩(Nm)", -50.0, 50.0, 0.1, 0.0, 3)])
        add_mode_page(1, [
            ("目标速度(rev/s)", -100.0, 100.0, 0.1, 0.0, 3),
            ("力矩前馈(Nm)", -50.0, 50.0, 0.1, 0.0, 3),
        ])
        add_mode_page(2, [
            ("目标位置(rev)", -2000.0, 2000.0, 0.01, 0.0, 4),
            ("速度前馈(rev/s)", -100.0, 100.0, 0.1, 0.0, 3),
            ("力矩前馈(Nm)", -50.0, 50.0, 0.1, 0.0, 3),
        ])

        mode_layout.addWidget(self.param_stack)
        self.select_mode(0)

        conv_box = QtWidgets.QGroupBox("目标换算(输出侧 -> 转子侧)")
        conv_form = QtWidgets.QFormLayout(conv_box)
        self.sb_gear = QtWidgets.QDoubleSpinBox()
        self.sb_gear.setRange(1.0, 1000.0)
        self.sb_gear.setDecimals(4)
        self.sb_gear.setValue(self.gear_ratio)
        self.sb_kt = QtWidgets.QDoubleSpinBox()
        self.sb_kt.setRange(0.001, 10.0)
        self.sb_kt.setDecimals(6)
        self.sb_kt.setValue(self.torque_constant)
        self.sb_eff = QtWidgets.QDoubleSpinBox()
        self.sb_eff.setRange(0.1, 1.0)
        self.sb_eff.setDecimals(4)
        self.sb_eff.setValue(self.trans_eff)
        self.lbl_tau_rotor = QtWidgets.QLabel("-")
        self.lbl_iq_target = QtWidgets.QLabel("-")
        conv_form.addRow("减速比", self.sb_gear)
        conv_form.addRow("Kt(Nm/A)", self.sb_kt)
        conv_form.addRow("效率η", self.sb_eff)
        conv_form.addRow("目标转子扭矩(Nm)", self.lbl_tau_rotor)
        conv_form.addRow("目标转子电流(A)", self.lbl_iq_target)
        mode_layout.addWidget(conv_box)

        self.sb_gear.valueChanged.connect(self._update_conversion_display)
        self.sb_kt.valueChanged.connect(self._update_conversion_display)
        self.sb_eff.valueChanged.connect(self._update_conversion_display)
        for widgets in self.mode_inputs.values():
            for w in widgets:
                w.valueChanged.connect(self._update_conversion_display)
                w.valueChanged.connect(self._on_manual_param_changed)

        wave_box = QtWidgets.QGroupBox("力矩波形（仅模式0，输出侧Nm）")
        wave_form = QtWidgets.QFormLayout(wave_box)
        self.cmb_wave = QtWidgets.QComboBox()
        self.cmb_wave.addItems(["常值", "正弦", "三角"])
        self.sb_amp = QtWidgets.QDoubleSpinBox()
        self.sb_amp.setRange(0.0, 50.0)
        self.sb_amp.setDecimals(3)
        self.sb_amp.setSingleStep(0.1)
        self.sb_amp.setValue(1.0)
        self.sb_freq = QtWidgets.QDoubleSpinBox()
        self.sb_freq.setRange(0.01, 20.0)
        self.sb_freq.setDecimals(3)
        self.sb_freq.setSingleStep(0.1)
        self.sb_freq.setValue(0.5)
        self.sb_bias = QtWidgets.QDoubleSpinBox()
        self.sb_bias.setRange(-50.0, 50.0)
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
        for b in [self.btn_wave_start, self.btn_wave_cycles, self.btn_wave_stop]:
            b.setAutoDefault(False)
            b.setDefault(False)
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
        self.sb_amp.valueChanged.connect(self._on_manual_param_changed)
        self.sb_freq.valueChanged.connect(self._on_manual_param_changed)
        self.sb_bias.valueChanged.connect(self._on_manual_param_changed)
        self.sb_cycles.valueChanged.connect(self._on_manual_param_changed)
        self.cmb_wave.currentIndexChanged.connect(self._on_manual_param_changed)

        plot_box = QtWidgets.QGroupBox("实时曲线")
        plot_layout = QtWidgets.QVBoxLayout(plot_box)
        self.plot_layout = plot_layout

        top_tools = QtWidgets.QHBoxLayout()
        self.sb_horizon = QtWidgets.QDoubleSpinBox()
        self.sb_horizon.setRange(1.0, 120.0)
        self.sb_horizon.setDecimals(1)
        self.sb_horizon.setSingleStep(1.0)
        self.sb_horizon.setValue(15.0)
        top_tools.addWidget(QtWidgets.QLabel("Horizon(s)"))
        top_tools.addWidget(self.sb_horizon)
        top_tools.addStretch(1)

        legend_txt = "蓝=位置  橙=速度  绿=目标力矩  红=实测力矩  紫=Iq目标  粉=Iq实际"
        self.lbl_legend = QtWidgets.QLabel(legend_txt)
        self.lbl_legend.setStyleSheet("font-weight:600;")
        top_tools.addWidget(self.lbl_legend)
        plot_layout.addLayout(top_tools)

        vis_row = QtWidgets.QHBoxLayout()
        self.chk_pos = QtWidgets.QCheckBox("位置")
        self.chk_spd = QtWidgets.QCheckBox("速度")
        self.chk_cmd = QtWidgets.QCheckBox("目标力矩")
        self.chk_tau = QtWidgets.QCheckBox("力矩")
        self.chk_iq_set = QtWidgets.QCheckBox("Iq目标")
        self.chk_iq = QtWidgets.QCheckBox("Iq实际")
        self.chk_pos.setChecked(True)
        self.chk_spd.setChecked(True)
        self.chk_cmd.setChecked(True)
        self.chk_tau.setChecked(True)
        self.chk_iq_set.setChecked(True)
        self.chk_iq.setChecked(True)
        self.chk_pos.setStyleSheet("color: rgb(33,150,243); font-weight:600;")
        self.chk_spd.setStyleSheet("color: rgb(255,152,0); font-weight:600;")
        self.chk_cmd.setStyleSheet("color: rgb(76,175,80); font-weight:600;")
        self.chk_tau.setStyleSheet("color: rgb(244,67,54); font-weight:600;")
        self.chk_iq_set.setStyleSheet("color: rgb(156,39,176); font-weight:600;")
        self.chk_iq.setStyleSheet("color: rgb(233,30,99); font-weight:600;")
        vis_row.addWidget(self.chk_pos)
        vis_row.addWidget(self.chk_spd)
        vis_row.addWidget(self.chk_cmd)
        vis_row.addWidget(self.chk_tau)
        vis_row.addWidget(self.chk_iq_set)
        vis_row.addWidget(self.chk_iq)
        vis_row.addStretch(1)
        plot_layout.addLayout(vis_row)

        self.plot_pos = pg.PlotWidget(title="位置")
        self.plot_pos.showGrid(x=True, y=True, alpha=0.25)
        self.curve_pos = self.plot_pos.plot(pen=pg.mkPen((33, 150, 243), width=2), name="位置")

        self.plot_spd = pg.PlotWidget(title="速度")
        self.plot_spd.showGrid(x=True, y=True, alpha=0.25)
        self.curve_spd = self.plot_spd.plot(pen=pg.mkPen((255, 152, 0), width=2), name="速度")

        self.plot_cmd_tau = pg.PlotWidget(title="命令/实测力矩")
        self.plot_cmd_tau.showGrid(x=True, y=True, alpha=0.25)
        self.curve_cmd = self.plot_cmd_tau.plot(pen=pg.mkPen((76, 175, 80), width=2), name="目标力矩")
        self.curve_tau = self.plot_cmd_tau.plot(pen=pg.mkPen((244, 67, 54), width=2), name="实测力矩")
        self.plot_iq = pg.PlotWidget(title="Iq")
        self.plot_iq.showGrid(x=True, y=True, alpha=0.25)
        self.curve_iq_set = self.plot_iq.plot(pen=pg.mkPen((156, 39, 176), width=2), name="Iq目标")
        self.curve_iq = self.plot_iq.plot(pen=pg.mkPen((233, 30, 99), width=2), name="Iq实际")

        plot_layout.addWidget(self.plot_pos)
        plot_layout.addWidget(self.plot_spd)
        plot_layout.addWidget(self.plot_cmd_tau)
        plot_layout.addWidget(self.plot_iq)
        left.addWidget(plot_box, 12)

        self.chk_pos.toggled.connect(self._update_plot_visibility)
        self.chk_spd.toggled.connect(self._update_plot_visibility)
        self.chk_cmd.toggled.connect(self._update_plot_visibility)
        self.chk_tau.toggled.connect(self._update_plot_visibility)
        self.chk_iq_set.toggled.connect(self._update_plot_visibility)
        self.chk_iq.toggled.connect(self._update_plot_visibility)
        self._update_plot_visibility()
        self._update_conversion_display()

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

    def _to_rotor_torque(self, tau_out):
        gear = max(1e-6, float(self.sb_gear.value()))
        eta = max(1e-6, float(self.sb_eff.value()))
        return float(tau_out) / (gear * eta)

    def _target_outside_torque(self, p1, p2, p3):
        if self.current_mode == 0:
            return p1
        if self.current_mode == 1:
            return p2
        if self.current_mode == 2:
            return p3
        return 0.0

    def _update_conversion_display(self):
        p1, p2, p3 = self._current_params()
        tau_out = self._target_outside_torque(p1, p2, p3)
        tau_rotor = self._to_rotor_torque(tau_out)
        kt = max(1e-6, float(self.sb_kt.value()))
        iq_target = tau_rotor / kt
        self.lbl_tau_rotor.setText(f"{tau_rotor:.4f}")
        self.lbl_iq_target.setText(f"{iq_target:.4f}")

    def _on_manual_param_changed(self):
        # 修改参数不应自动下发；若波形正在运行则先停掉，避免“边改边执行”。
        if self.wave_enabled:
            self.wave_enabled = False
            self.wave_cycles_target = -1.0
            self.log_line("> WAVE STOP (参数变更)")

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
        self.last_cmd_value = 0.0
        self._send_frame(self._build_payload(0x08))
        self.log_line("> STOP")

    def send_clear(self):
        self._send_frame(self._build_payload(0x20))
        self.log_line("> CLEAR+INIT")

    def send_execute(self):
        self.wave_enabled = False
        self.wave_cycles_target = -1.0
        p1_out, p2_out, p3_out = self._current_params()
        p1, p2, p3 = p1_out, p2_out, p3_out
        if self.current_mode == 0:
            p1 = self._to_rotor_torque(p1_out)
            self.last_cmd_value = p1_out
        elif self.current_mode == 1:
            p2 = self._to_rotor_torque(p2_out)
            self.last_cmd_value = p2_out
        elif self.current_mode == 2:
            p3 = self._to_rotor_torque(p3_out)
            self.last_cmd_value = p3_out
        self._send_frame(self._build_payload(0x40, self.current_mode, p1, p2, p3))
        self.lbl_sent.setText(
            f"已下发: mode={self.current_mode}({MODE_TEXT[self.current_mode]}) "
            f"out=({p1_out:.4f},{p2_out:.4f},{p3_out:.4f}) rotor=({p1:.4f},{p2:.4f},{p3:.4f})"
        )
        self.log_line(
            f"> EXEC mode={self.current_mode} out=({p1_out:.4f},{p2_out:.4f},{p3_out:.4f}) "
            f"rotor=({p1:.4f},{p2:.4f},{p3:.4f})"
        )

    def _send_output_torque_cmd(self, tau_out):
        tau_rotor = self._to_rotor_torque(tau_out)
        self.last_cmd_value = float(tau_out)
        self._send_frame(self._build_payload(0x40, 0, tau_rotor, 0.0, 0.0))

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
        self.select_mode(0)
        self.wave_t0 = time.monotonic()
        self.wave_enabled = True
        self.wave_cycles_target = -1.0
        self.log_line("> WAVE START")

    def stop_wave(self):
        self.wave_enabled = False
        self.wave_cycles_target = -1.0
        self.log_line("> WAVE STOP")

    def send_wave_cycles(self):
        self.select_mode(0)
        self.wave_t0 = time.monotonic()
        self.wave_enabled = True
        self.wave_cycles_target = float(self.sb_cycles.value())
        self.log_line(f"> WAVE {self.wave_cycles_target:.2f} cycles")

    def _axis_state_text(self, raw):
        if raw == 255:
            return "N/A"
        return f"{raw} - {AXIS_STATE_TEXT.get(raw, '未知')}"

    def _err_type_text(self, t):
        return f"{t} - {ERROR_TYPE_TEXT.get(t, '未知')}"

    def _hb_flags_text(self, f):
        # bit0 motor, bit1 encoder, bit2 controller, bit3 system, bit7 traj_done
        return (
            f"0x{f:02X} M{(f>>0)&1} E{(f>>1)&1} C{(f>>2)&1} "
            f"S{(f>>3)&1} T{(f>>7)&1}"
        )

    def _yn(self, v):
        return "YES/是" if v else "NO/否"

    def _handle_frame(self, frame):
        if len(frame) != 32:
            return

        payload = frame[3:]
        t_cs = struct.unpack_from('<H', payload, 0)[0]
        pos_x10 = struct.unpack_from('<h', payload, 2)[0]
        spd_x100 = struct.unpack_from('<h', payload, 4)[0]
        tau_x100 = struct.unpack_from('<h', payload, 6)[0]
        tau_set_x100 = struct.unpack_from('<h', payload, 8)[0]
        iq_x100 = struct.unpack_from('<h', payload, 10)[0]
        iq_set_x100 = struct.unpack_from('<h', payload, 12)[0]
        axis_state = payload[14]
        hb_flags = payload[15]
        active_id = payload[16]
        can_ok = payload[17]
        axis_error_u32 = struct.unpack_from('<I', payload, 18)[0]
        mode_rx = payload[22] & 0x07
        req_state = payload[23]
        detail_err_u32 = struct.unpack_from('<I', payload, 24)[0]
        detail_type = payload[28]

        self.lbl_last.setText(f"t={t_cs/100.0:.2f}s")
        self.lbl_id.setText(str(active_id))
        self.lbl_mode.setText(f"{mode_rx} - {MODE_TEXT.get(mode_rx, '未知')}")
        self.lbl_axis_state.setText(self._axis_state_text(axis_state))

        self.lbl_can.setText("CAN正常" if can_ok else "CAN没数据返回")
        self.lbl_can.setStyleSheet(
            "color:#2e7d32;font-weight:bold;" if can_ok else "color:#c62828;font-weight:bold;"
        )

        self.data["cmd"].setText("N/A" if tau_set_x100 == -32768 else f"{tau_set_x100 / 100.0:.2f}")
        self.data["pos_abs"].setText("N/A" if pos_x10 == -32768 else f"{pos_x10 / 10.0:.3f}")
        self.data["pos"].setText("N/A" if pos_x10 == -32768 else f"{pos_x10 / 10.0:.3f}")
        self.data["spd"].setText("N/A" if spd_x100 == -32768 else f"{spd_x100 / 100.0:.3f}")
        self.data["iq"].setText("N/A" if iq_x100 == -32768 else f"{iq_x100 / 100.0:.3f}")
        self.data["iq_set"].setText("N/A" if iq_set_x100 == -32768 else f"{iq_set_x100 / 100.0:.3f}")
        self.data["tau"].setText("N/A" if tau_x100 == -32768 else f"{tau_x100 / 100.0:.3f}")
        self.data["axis_state"].setText(self._axis_state_text(axis_state))
        self.data["req_state"].setText(self._axis_state_text(req_state))
        self.data["axis_error"].setText(f"0x{axis_error_u32:08X}")
        self.data["hb_flags"].setText(self._hb_flags_text(hb_flags))
        self.data["detail_type"].setText(self._err_type_text(detail_type))
        self.data["detail_err"].setText(f"0x{detail_err_u32:08X}")

        # 单行故障总览（中英双语）：
        # Axis Fault 直接由 Axis_Error 是否非0判断；
        # Motor/Encoder/Controller/System 由 Heartbeat Flags 的 bit0..3 展示。
        axis_fault = axis_error_u32 != 0
        motor_fault = ((hb_flags >> 0) & 0x01) != 0
        encoder_fault = ((hb_flags >> 1) & 0x01) != 0
        controller_fault = ((hb_flags >> 2) & 0x01) != 0
        system_fault = ((hb_flags >> 3) & 0x01) != 0
        traj_done = ((hb_flags >> 7) & 0x01) != 0
        self.lbl_faults.setText(
            "Faults | "
            f"System Fault 系统异常:{self._yn(system_fault)}  "
            f"Axis Fault 驱动异常:{self._yn(axis_fault)}  "
            f"Motor Fault 电机异常:{self._yn(motor_fault)}  "
            f"Encoder Fault 编码器异常:{self._yn(encoder_fault)}  "
            f"Controller Fault 控制异常:{self._yn(controller_fault)}  "
            f"Flags={self._hb_flags_text(hb_flags)}"
            f" (T={self._yn(traj_done)})"
        )

        t_plot = time.monotonic() - self.plot_t0
        pos_v = float("nan") if pos_x10 == -32768 else (pos_x10 / 10.0)
        spd_v = float("nan") if spd_x100 == -32768 else (spd_x100 / 100.0)
        tau_v = float("nan") if tau_x100 == -32768 else (tau_x100 / 100.0)
        tau_set_v = float("nan") if tau_set_x100 == -32768 else (tau_set_x100 / 100.0)
        iq_set_v = float("nan") if iq_set_x100 == -32768 else (iq_set_x100 / 100.0)
        iq_v = float("nan") if iq_x100 == -32768 else (iq_x100 / 100.0)

        self.buf_t.append(t_plot)
        self.buf_pos.append(pos_v)
        self.buf_spd.append(spd_v)
        self.buf_tau_set.append(tau_set_v if tau_set_x100 != -32768 else self.last_cmd_value)
        self.buf_tau.append(tau_v)
        self.buf_iq_set.append(iq_set_v)
        self.buf_iq.append(iq_v)

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

    def _update_plot_visibility(self):
        show_pos = self.chk_pos.isChecked()
        show_spd = self.chk_spd.isChecked()
        show_cmd_tau = self.chk_cmd.isChecked() or self.chk_tau.isChecked()
        show_iq = self.chk_iq_set.isChecked() or self.chk_iq.isChecked()

        self.curve_pos.setVisible(show_pos)
        self.curve_spd.setVisible(show_spd)
        self.curve_cmd.setVisible(self.chk_cmd.isChecked())
        self.curve_tau.setVisible(self.chk_tau.isChecked())
        self.curve_iq_set.setVisible(self.chk_iq_set.isChecked())
        self.curve_iq.setVisible(self.chk_iq.isChecked())

        self.plot_pos.setVisible(show_pos)
        self.plot_spd.setVisible(show_spd)
        self.plot_cmd_tau.setVisible(show_cmd_tau)
        self.plot_iq.setVisible(show_iq)

        # 只保留可见图时，让其自动占满剩余空间
        self.plot_layout.setStretchFactor(self.plot_pos, 1 if show_pos else 0)
        self.plot_layout.setStretchFactor(self.plot_spd, 1 if show_spd else 0)
        self.plot_layout.setStretchFactor(self.plot_cmd_tau, 1 if show_cmd_tau else 0)
        self.plot_layout.setStretchFactor(self.plot_iq, 1 if show_iq else 0)

    def _update_plots(self):
        if len(self.buf_t) < 2:
            return

        x = list(self.buf_t)
        self.curve_pos.setData(x, list(self.buf_pos))
        self.curve_spd.setData(x, list(self.buf_spd))
        self.curve_cmd.setData(x, list(self.buf_tau_set))
        self.curve_tau.setData(x, list(self.buf_tau))
        self.curve_iq_set.setData(x, list(self.buf_iq_set))
        self.curve_iq.setData(x, list(self.buf_iq))

        xmax = x[-1]
        xmin = max(0.0, xmax - float(self.sb_horizon.value()))
        self.plot_pos.setXRange(xmin, xmax, padding=0.0)
        self.plot_spd.setXRange(xmin, xmax, padding=0.0)
        self.plot_cmd_tau.setXRange(xmin, xmax, padding=0.0)
        self.plot_iq.setXRange(xmin, xmax, padding=0.0)

    def _on_tick(self):
        self._poll_serial()
        now = time.monotonic()
        if self.wave_enabled and self.ser and self.ser.is_open:
            if now - self.last_wave_send >= 0.02:  # 50Hz
                self.last_wave_send = now
                target_out = self._wave_target()
                self._send_output_torque_cmd(target_out)
            if self.wave_cycles_target > 0:
                cycles_now = (now - self.wave_t0) * float(self.sb_freq.value())
                if cycles_now >= self.wave_cycles_target:
                    self.wave_enabled = False
                    self.wave_cycles_target = -1.0
                    self.log_line("> WAVE DONE")
        self._update_plots()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MotorBleGui()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
