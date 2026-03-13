#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
// 全局 CAN3（用于本 ino 收包）
// 在本项目里，CAN3 必须按下面结构使用，才能稳定给 TMotor 持续下发电流命令并持续接收回传。

// 怎么做

// 在 Tmotor_Teensy_Project.ino 保留全局：
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
// 在 Motor_Control_Tmotor.h 的 public 增加类内成员：
// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
// 在 Motor_Control_Tmotor.cpp 去掉 extern Can3，send_current_A() 使用类内 Can3.write(...)。
// 在 setup() 中执行两步初始化：
// 先 initial_CAN()（全局），再 motor.initial_CAN()（类内）。
// 为什么要这么做

// 你这个工程实测表明：单纯 extern 共享一个全局 Can3 时，进入持续发送后容易在 Can3.write() 路径失稳（表现为只跑一次或卡死）。
// 采用“全局 Can3 + 类内 Can3 + 二次初始化”后，CAN 外设状态能被拉回稳定，发送和回传都恢复正常。
// 结论来自你真机验证结果，不是语法层面推测。
// 不这么做会怎么样

// 可能出现：上电后只打印一次 heartbeat、只发出首帧、随后主循环停滞或无回包。
// 你这个现象可以这样理解：

// extern 共享全局 Can3 时
// 硬件上只有一个 CAN3 外设。
// 但 FlexCAN_T4 对象除了操作硬件，还维护一套软件状态（mailbox/队列/标志位/回调状态）。
// 你所有读写都走同一个对象时，如果初始化时序或某次状态进入异常（你日志里就是 write 后停），这个对象的状态会一直坏下去。
// “全局 Can3 + 类内 Can3 + 二次初始化”为什么会不一样
// 这不是多了一个硬件 CAN3，而是多了一个 FlexCAN_T4 软件实例。
// motor.initial_CAN() 再次 begin + setBaudRate，相当于把 CAN3 寄存器/mailbox 状态重新刷一遍。
// 实测上你这一步把之前容易卡死的状态清掉了，所以恢复持续发送/回传。
// 核心差异
// 差异不在 CAN ID，也不在 extern 语法对错。
// 差异在 FlexCAN_T4 对象状态 + 初始化时序。
// 你当前方案本质是“用二次初始化和对象隔离绕开库/时序问题”，所以能稳定跑。
// 所以你看到“extern 不行、双对象+二次初始化行”，本质是运行时状态机问题，不是 C++ 链接机制本身问题。

// GUI 侧会看到“有启动日志但无持续数据”，电机侧表现为“不持续响应/不转或瞬时动作后停止”。

#include "Motor_Control_Tmotor.h"

Motor_Control_Tmotor motor(0x001, 104);
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

static uint8_t active_id = 104;
static uint8_t active_mode = 1;  // 默认电流模式
static float cmd_p1 = 0.0f;
static float cmd_p2 = 0.0f;
static float cmd_p3 = 0.0f;
static constexpr float KT_TMOTOR = 0.73f;  // tau(Nm) = iq(A) * Kt

static uint32_t last_ctrl_us = 0;
static uint32_t last_ble_tx_ms = 0;
static uint32_t rx_total = 0;
static uint32_t tx_total = 0;
static uint32_t last_rx_snapshot = 0;
static uint32_t last_rx_ms = 0;
static uint32_t last_recover_ms = 0;
static bool pos_raw_valid = false;
static int16_t last_pos_raw_x10 = 0;
static int32_t pos_turns = 0;
static int32_t pos_multi_x10 = 0;
static bool pos_zero_valid = false;
static int32_t pos_zero_x10 = 0;

static uint8_t ble_payload_rx[29] = {0};
static uint8_t ble_frame_tx[32] = {0};
// 有些驱动固件在负半轴会回传电流幅值(始终为正)。
// 这里在电流模式下按指令符号做补偿，避免 GUI 上“负电流被取正”。
static bool g_force_current_sign_from_cmd = true;

static inline int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static inline void put_u16_le(uint8_t *b, int &i, uint16_t v) {
  b[i++] = (uint8_t)(v & 0xFF);
  b[i++] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void put_i16_le(uint8_t *b, int &i, int16_t v) {
  put_u16_le(b, i, (uint16_t)v);
}

static inline void put_u32_le(uint8_t *b, int &i, uint32_t v) {
  b[i++] = (uint8_t)(v & 0xFF);
  b[i++] = (uint8_t)((v >> 8) & 0xFF);
  b[i++] = (uint8_t)((v >> 16) & 0xFF);
  b[i++] = (uint8_t)((v >> 24) & 0xFF);
}

static inline int16_t get_i16_le(const uint8_t *b, int idx) {
  return (int16_t)((uint16_t)b[idx] | ((uint16_t)b[idx + 1] << 8));
}

static inline float get_f32_le(const uint8_t *b, int idx) {
  float v = 0.0f;
  memcpy(&v, b + idx, 4);
  return v;
}

void apply_active_mode_command() {
  switch (active_mode) {
    case 0:
      motor.send_duty(cmd_p1);
      break;
    case 1:
      // mode1 统一为“扭矩输入(Nm)”，内部换算成 Iq(A) 下发
      motor.send_current_A(cmd_p1 / KT_TMOTOR);
      break;
    case 2:
      motor.send_current_brake_A(cmd_p1);
      break;
    case 3:
      motor.send_rpm(cmd_p1);
      break;
    case 4:
      motor.send_pos(cmd_p1);
      break;
    case 6:
      motor.send_pos_spd(cmd_p1, cmd_p2, cmd_p3);
      break;
    default:
      break;
  }
}

void initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
}

void recover_can_if_needed() {
  const uint32_t now_ms = millis();
  const bool can_ok_now = (last_rx_ms > 0 && (now_ms - last_rx_ms) <= 1000);
  if (can_ok_now) return;
  if (now_ms - last_recover_ms < 1200) return;

  last_recover_ms = now_ms;
  Serial.println("[CAN] recover start");
  active_mode = 1;
  cmd_p1 = 0.0f;
  cmd_p2 = 0.0f;
  cmd_p3 = 0.0f;
  initial_CAN();
  motor.initial_CAN();
  motor.set_drive_id(active_id);
  bool ok = initial_motor();
  Serial.print("[CAN] recover init=");
  Serial.println(ok ? "OK" : "NO_DATA");
}

void receive_motor_feedback() {
  const uint32_t t0 = micros();
  uint16_t frames = 0;
  CAN_message_t mr;
  while (Can3.read(mr)) {
    if (motor.unpack_servo_telemetry(mr)) {
      rx_total++;
      last_rx_ms = millis();
      int16_t cur_raw = motor.servo_pos_raw_x10;
      if (!pos_raw_valid) {
        pos_raw_valid = true;
        last_pos_raw_x10 = cur_raw;
        pos_multi_x10 = (int32_t)cur_raw;
      } else {
        int32_t diff = (int32_t)cur_raw - (int32_t)last_pos_raw_x10;
        if (diff < -40000) pos_turns += 1;  // 正向跨 +3200 -> -3200
        if (diff > 40000) pos_turns -= 1;   // 反向跨 -3200 -> +3200
        last_pos_raw_x10 = cur_raw;
        pos_multi_x10 = (int32_t)cur_raw + pos_turns * 64000;
      }
    }
    frames++;
    if (frames >= 32) break;
    if (micros() - t0 > 2000) break;
  }
}

bool initial_motor() {
  motor.set_drive_id(active_id);
  motor.send_current_A(0.0f);
  delay(20);

  CAN_message_t tmp;
  while (Can3.read(tmp)) {}

  bool got = false;
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    CAN_message_t mr;
    while (Can3.read(mr)) {
      if (motor.unpack_servo_telemetry(mr)) {
        got = true;
      }
    }
    if (got) break;
  }
  return got;
}

/******************** BLE：接收（带占位、32字节帧） ********************/
void Receive_ble_Data() {
  const uint8_t FRAME_LEN = 32;
  const uint8_t PAYLOAD_LEN = FRAME_LEN - 3;
  static uint8_t state = 0;
  static uint8_t len_byte = 0;
  static uint8_t payload_idx = 0;

  while (Serial5.available()) {
    uint8_t ch = (uint8_t)Serial5.read();
    if (state == 0) {
      if (ch == 0xA5) state = 1;
      continue;
    }
    if (state == 1) {
      if (ch == 0x5A) state = 2;
      else state = (ch == 0xA5) ? 1 : 0;
      continue;
    }
    if (state == 2) {
      len_byte = ch;
      if (len_byte == FRAME_LEN) {
        payload_idx = 0;
        state = 3;
      } else {
        state = 0;
      }
      continue;
    }
    if (state == 3) {
      ble_payload_rx[payload_idx++] = ch;
      if (payload_idx < PAYLOAD_LEN) continue;
      state = 0;

      // payload[0] magic='C'
      if (ble_payload_rx[0] != 'C') {
        continue;
      }

      uint8_t flags = ble_payload_rx[1];
      uint8_t req_id = ble_payload_rx[2];
      uint8_t req_mode = ble_payload_rx[3] & 0x07;
      float p1 = get_f32_le(ble_payload_rx, 4);
      float p2 = get_f32_le(ble_payload_rx, 8);
      float p3 = get_f32_le(ble_payload_rx, 12);
      if ((flags & 0x01) && (req_id == 104 || req_id == 105) && req_id != active_id) {
        active_id = req_id;
        motor.set_drive_id(active_id);
        pos_raw_valid = false;
        pos_turns = 0;
        pos_multi_x10 = 0;
        pos_zero_valid = false;
        bool ok_id = initial_motor();
        Serial.print("[BLE] SET_ID id=");
        Serial.print((int)active_id);
        Serial.print(" init=");
        Serial.println(ok_id ? "OK" : "NO_DATA");
      } else {
        motor.set_drive_id(active_id);
      }

      // 兼容旧按钮：flags 0x02 按电流模式处理 p1
      if (flags & 0x02) {
        active_mode = 1;
        cmd_p1 = p1;
        cmd_p2 = 0.0f;
        cmd_p3 = 0.0f;
      }

      if (flags & 0x04) {
        bool ok = initial_motor();
        Serial.print("[BLE] INIT id=");
        Serial.print((int)active_id);
        Serial.print(" ok=");
        Serial.println(ok ? "1" : "0");
      }

      if (flags & 0x08) {
        active_mode = 1;
        cmd_p1 = 0.0f;
        cmd_p2 = 0.0f;
        cmd_p3 = 0.0f;
        motor.send_current_A(0.0f);
        tx_total++;
      }

      // 真实原点归零（电机内部原点）
      if (flags & 0x20) {
        motor.set_origin_here();
        delay(5);
        bool ok = initial_motor();
        Serial.print("[BLE] ORIGIN_HERE id=");
        Serial.print((int)active_id);
        Serial.print(" ok=");
        Serial.println(ok ? "1" : "0");
      }

      // 执行所选模式参数
      if (flags & 0x40) {
        active_mode = req_mode;
        cmd_p1 = p1;
        cmd_p2 = p2;
        cmd_p3 = p3;
        if (active_mode == 5) {
          motor.set_origin_here();
        } else {
          apply_active_mode_command();
          tx_total++;
        }
      }

      Serial.print("[BLE] flags=");
      Serial.print((unsigned int)flags);
      Serial.print(" id=");
      Serial.print((int)active_id);
      Serial.print(" mode=");
      Serial.print((int)active_mode);
      Serial.print(" p1=");
      Serial.print(cmd_p1, 3);
      Serial.print(" p2=");
      Serial.print(cmd_p2, 3);
      Serial.print(" p3=");
      Serial.println(cmd_p3, 3);
    }
  }
}

/******************** BLE：发送（与你一致） ********************/
void Transmit_ble_Data() {
  const uint8_t FRAME_LEN = 32;

  uint16_t t_cs = (uint16_t)((millis() / 10) & 0xFFFF);

  int16_t pos_x10 = isnan(motor.servo_pos_deg) ? (int16_t)-32768
                                                : clamp_i16((int32_t)lroundf(motor.servo_pos_deg * 10.0f));
  // speed uses /10 to fit int16, GUI recovers by *10
  int16_t spd_div10 = isnan(motor.servo_spd_rpm) ? (int16_t)-32768
                                                  : clamp_i16((int32_t)lroundf(motor.servo_spd_rpm / 10.0f));
  float cur_for_tx_A = motor.servo_cur_A;
  if (g_force_current_sign_from_cmd && active_mode == 1 && !isnan(cur_for_tx_A)) {
    if (cmd_p1 < -0.05f && cur_for_tx_A > 0.0f) cur_for_tx_A = -cur_for_tx_A;
    if (cmd_p1 > 0.05f && cur_for_tx_A < 0.0f) cur_for_tx_A = -cur_for_tx_A;
  }
  int16_t cur_x100 = isnan(cur_for_tx_A) ? (int16_t)-32768
                                          : clamp_i16((int32_t)lroundf(cur_for_tx_A * 100.0f));
  int8_t temp_i8 = isnan(motor.servo_temp_C) ? (int8_t)-128
                                              : (int8_t)clamp_i16((int32_t)lroundf(motor.servo_temp_C));
  uint8_t err_u8 = isnan(motor.servo_error) ? 255 : (uint8_t)motor.servo_error;

  uint32_t rx_delta = rx_total - last_rx_snapshot;
  last_rx_snapshot = rx_total;
  uint8_t can_ok = (last_rx_ms > 0 && (millis() - last_rx_ms) <= 1000) ? 1 : 0;
  // mode1 上发期望值按“扭矩Nm*100”；其它模式保持 p1 直传
  int16_t cmd_x100 = clamp_i16((int32_t)lroundf(cmd_p1 * 100.0f));
  int32_t pos_zeroed_x10 = pos_multi_x10;
  if (pos_zero_valid) pos_zeroed_x10 -= pos_zero_x10;

  memset(ble_frame_tx, 0, sizeof(ble_frame_tx));
  ble_frame_tx[0] = 0xA5;
  ble_frame_tx[1] = 0x5A;
  ble_frame_tx[2] = FRAME_LEN;

  int i = 3;
  put_u16_le(ble_frame_tx, i, t_cs);             // payload[0..1]
  put_i16_le(ble_frame_tx, i, pos_x10);          // payload[2..3]
  put_i16_le(ble_frame_tx, i, spd_div10);        // payload[4..5]
  put_i16_le(ble_frame_tx, i, cur_x100);         // payload[6..7]
  ble_frame_tx[i++] = (uint8_t)temp_i8;          // payload[8]
  ble_frame_tx[i++] = err_u8;                    // payload[9]
  ble_frame_tx[i++] = active_id;                 // payload[10]
  ble_frame_tx[i++] = can_ok;                    // payload[11]
  put_i16_le(ble_frame_tx, i, cmd_x100);         // payload[12..13]
  put_u32_le(ble_frame_tx, i, rx_total);         // payload[14..17]
  put_u32_le(ble_frame_tx, i, tx_total);         // payload[18..21]
  put_u16_le(ble_frame_tx, i, (uint16_t)(rx_delta & 0xFFFF)); // payload[22..23]

  put_u32_le(ble_frame_tx, i, (uint32_t)pos_zeroed_x10); // payload[24..27]
  ble_frame_tx[i++] = active_mode;                       // payload[28]

  Serial5.write((uint8_t *)ble_frame_tx, (size_t)FRAME_LEN);
}

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  Serial5.setTimeout(10);
  delay(600);

  Serial.println("=== TMOTOR BLE BINARY START ===");
  initial_CAN();
  motor.initial_CAN();
  motor.set_drive_id(active_id);
  bool ok = initial_motor();
  Serial.print("[OK] init feedback=");
  Serial.println(ok ? "YES" : "NO");
}

void loop() {
  Receive_ble_Data();
  recover_can_if_needed();

  const uint32_t now_us = micros();
  if (now_us - last_ctrl_us >= 10000) {  // 100Hz control
    last_ctrl_us = now_us;
    for (int i = 0; i < 4; ++i) {
      receive_motor_feedback();
    }
    if (active_mode != 5) {
      apply_active_mode_command();
      tx_total++;
    }
  }

  const uint32_t now_ms = millis();
  if (now_ms - last_ble_tx_ms >= 50) {  // 20Hz uplink
    last_ble_tx_ms = now_ms;
    Transmit_ble_Data();
  }
}
