#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Motor_Control_Sig.h"

Motor_Control_Sig motor_node1(1);
Motor_Control_Sig motor_node2(2);
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

enum SigControlMode : uint8_t {
  SIG_MODE_TORQUE = 0,
  SIG_MODE_VELOCITY = 1,
  SIG_MODE_POSITION = 2,
};

static uint8_t active_id = 1;
static uint8_t active_mode = SIG_MODE_TORQUE;
static uint8_t configured_mode = 0xFF;

static float cmd_p1 = 0.0f;
static float cmd_p2 = 0.0f;
static float cmd_p3 = 0.0f;

static uint32_t last_ctrl_us = 0;
static uint32_t last_ble_tx_ms = 0;
static uint32_t last_req_ms = 0;
static uint32_t last_err_req_ms = 0;
static uint8_t err_req_idx = 0;

static uint32_t rx_total = 0;
static uint32_t tx_total = 0;
static uint32_t last_rx_ms = 0;
static uint32_t last_recover_ms = 0;
static uint32_t last_mismatch_log_ms = 0;

static bool pos_zero_valid = false;
static float pos_zero_rev = 0.0f;

static uint8_t ble_payload_rx[29] = {0};
static uint8_t ble_frame_tx[32] = {0};
static uint8_t requested_axis_state = 1;

static inline Motor_Control_Sig &active_motor() {
  return (active_id == 2) ? motor_node2 : motor_node1;
}

static inline int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static inline int32_t clamp_i32(int64_t v) {
  if (v > 2147483647LL) return 2147483647;
  if (v < -2147483648LL) return -2147483648;
  return (int32_t)v;
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

static inline void put_i32_le(uint8_t *b, int &i, int32_t v) {
  put_u32_le(b, i, (uint32_t)v);
}

static inline float get_f32_le(const uint8_t *b, int idx) {
  float v = 0.0f;
  memcpy(&v, b + idx, 4);
  return v;
}

bool configure_active_mode(bool force = false) {
  Motor_Control_Sig &m = active_motor();
  if (!force && configured_mode == active_mode) {
    return true;
  }

  switch (active_mode) {
    case SIG_MODE_TORQUE:
      m.set_controller_mode(1, 1);  // torque + direct
      break;
    case SIG_MODE_VELOCITY:
      m.set_controller_mode(2, 2);  // velocity + ramp
      break;
    case SIG_MODE_POSITION:
      m.set_controller_mode(3, 3);  // position + filter
      break;
    default:
      return false;
  }

  delay(3);
  requested_axis_state = 8;
  m.set_axis_state(8);  // closed loop
  delay(3);

  configured_mode = active_mode;
  tx_total += 2;
  return true;
}

void apply_active_mode_command() {
  Motor_Control_Sig &m = active_motor();
  switch (active_mode) {
    case SIG_MODE_TORQUE:
      m.send_torque_Nm(cmd_p1);
      break;
    case SIG_MODE_VELOCITY:
      m.send_velocity_rev_s(cmd_p1, cmd_p2);
      break;
    case SIG_MODE_POSITION:
      m.send_position_rev(cmd_p1, cmd_p2, cmd_p3);
      break;
    default:
      return;
  }
  tx_total++;
}

void request_feedback_once() {
  Motor_Control_Sig &m = active_motor();
  m.request_encoder_estimates();
  m.request_iq();
  m.request_torques();
  tx_total += 3;
}

void initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
}

void receive_motor_feedback() {
  const uint32_t t0 = micros();
  uint16_t frames = 0;
  CAN_message_t mr;

  while (Can3.read(mr)) {
    bool matched = false;
    if (motor_node1.unpack_can_message(mr)) {
      matched = true;
      if (active_id == 1) {
        rx_total++;
        last_rx_ms = millis();
      }
    }
    if (motor_node2.unpack_can_message(mr)) {
      matched = true;
      if (active_id == 2) {
        rx_total++;
        last_rx_ms = millis();
      }
    }

    if (!matched && !mr.flags.extended) {
      const uint8_t rx_node = (uint8_t)((mr.id >> 5) & 0x3F);
      const uint8_t cmd_id = (uint8_t)(mr.id & 0x1F);
      if ((cmd_id == 0x01 || cmd_id == 0x09 || cmd_id == 0x14 || cmd_id == 0x1C) &&
          rx_node != active_id) {
        const uint32_t now_ms = millis();
        if (now_ms - last_mismatch_log_ms > 500) {
          last_mismatch_log_ms = now_ms;
          Serial.print("[CAN] frame node=");
          Serial.print((int)rx_node);
          Serial.print(" cmd=0x");
          Serial.print((int)cmd_id, HEX);
          Serial.print(" while active_id=");
          Serial.println((int)active_id);
        }
      }
    }
    frames++;
    if (frames >= 32) break;
    if (micros() - t0 > 2000) break;
  }
}

bool initial_motor() {
  Motor_Control_Sig &m = active_motor();
  configured_mode = 0xFF;
  m.seen_heartbeat = false;
  m.seen_encoder = false;
  m.seen_iq = false;
  m.seen_torque = false;

  active_mode = SIG_MODE_TORQUE;
  cmd_p1 = 0.0f;
  cmd_p2 = 0.0f;
  cmd_p3 = 0.0f;

  m.clear_errors();
  delay(20);
  requested_axis_state = 1;
  m.set_axis_state(1);  // idle
  delay(20);

  if (!configure_active_mode(true)) {
    return false;
  }
  apply_active_mode_command();

  CAN_message_t tmp;
  while (Can3.read(tmp)) {}

  bool got_enc = false;
  const uint32_t t0 = millis();

  while (millis() - t0 < 900) {
    request_feedback_once();
    const uint32_t wait_t = millis();
    while (millis() - wait_t < 20) {
      receive_motor_feedback();
      got_enc = m.seen_encoder;
      if (got_enc) break;
    }
    if (got_enc) break;
  }

  if (isfinite(m.pos_rev)) {
    pos_zero_rev = m.pos_rev;
    pos_zero_valid = true;
  } else {
    pos_zero_valid = false;
  }

  return got_enc;
}

void recover_can_if_needed() {
  const uint32_t now_ms = millis();
  const bool can_ok_now = (last_rx_ms > 0 && (now_ms - last_rx_ms) <= 1200);
  if (can_ok_now) return;
  if (now_ms - last_recover_ms < 1500) return;

  last_recover_ms = now_ms;
  Serial.println("[CAN] recover start");

  active_mode = SIG_MODE_TORQUE;
  configured_mode = 0xFF;
  cmd_p1 = 0.0f;
  cmd_p2 = 0.0f;
  cmd_p3 = 0.0f;

  initial_CAN();
  motor_node1.initial_CAN();
  motor_node2.initial_CAN();

  const bool ok = initial_motor();
  Serial.print("[CAN] recover init=");
  Serial.println(ok ? "OK" : "NO_DATA");
}

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

      if (ble_payload_rx[0] != 'C') {
        continue;
      }

      const uint8_t flags = ble_payload_rx[1];
      const uint8_t req_id = ble_payload_rx[2];
      const uint8_t req_mode = ble_payload_rx[3] & 0x07;
      const float p1 = get_f32_le(ble_payload_rx, 4);
      const float p2 = get_f32_le(ble_payload_rx, 8);
      const float p3 = get_f32_le(ble_payload_rx, 12);

      const bool req_id_valid = (req_id == 1 || req_id == 2);
      const bool id_changed = req_id_valid && req_id != active_id;
      if (id_changed) {
        active_id = req_id;
        configured_mode = 0xFF;
        pos_zero_valid = false;
        Serial.print("[BLE] APPLY_ID id=");
        Serial.println((int)active_id);
      }

      if (flags & 0x01) {
        const bool ok_id = initial_motor();
        Serial.print("[BLE] SET_ID+INIT id=");
        Serial.print((int)active_id);
        Serial.print(" ok=");
        Serial.println(ok_id ? "1" : "0");
      }

      if (flags & 0x04) {
        const bool ok = initial_motor();
        Serial.print("[BLE] INIT id=");
        Serial.print((int)active_id);
        Serial.print(" ok=");
        Serial.println(ok ? "1" : "0");
      }

      if (flags & 0x08) {
        active_mode = SIG_MODE_TORQUE;
        cmd_p1 = 0.0f;
        cmd_p2 = 0.0f;
        cmd_p3 = 0.0f;
        configure_active_mode();
        apply_active_mode_command();
      }

      if (flags & 0x20) {
        active_motor().clear_errors();
        tx_total++;
        delay(5);
        const bool ok = initial_motor();
        Serial.print("[BLE] CLEAR+INIT id=");
        Serial.print((int)active_id);
        Serial.print(" ok=");
        Serial.println(ok ? "1" : "0");
      }

      if (flags & 0x40) {
        if (req_mode <= SIG_MODE_POSITION) {
          active_mode = req_mode;
          cmd_p1 = p1;
          cmd_p2 = p2;
          cmd_p3 = p3;
          if (configure_active_mode()) {
            apply_active_mode_command();
          }
        } else {
          Serial.print("[BLE] unsupported mode=");
          Serial.println((int)req_mode);
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

void Transmit_ble_Data() {
  Motor_Control_Sig &m = active_motor();
  const uint8_t FRAME_LEN = 32;
  const uint16_t t_cs = (uint16_t)((millis() / 10) & 0xFFFF);

  const int16_t pos_x10 = isfinite(m.pos_rev)
                              ? clamp_i16((int32_t)lroundf(m.pos_rev * 10.0f))
                              : (int16_t)-32768;

  const int16_t spd_x100 = isfinite(m.vel_rev_s)
                               ? clamp_i16((int32_t)lroundf(m.vel_rev_s * 100.0f))
                               : (int16_t)-32768;

  const int16_t tau_x100 = isfinite(m.torque_measured_Nm)
                               ? clamp_i16((int32_t)lroundf(m.torque_measured_Nm * 100.0f))
                               : (int16_t)-32768;
  const int16_t tau_set_x100 = isfinite(m.torque_setpoint_Nm)
                                   ? clamp_i16((int32_t)lroundf(m.torque_setpoint_Nm * 100.0f))
                                   : (int16_t)-32768;
  const int16_t iq_set_x100 = isfinite(m.iq_setpoint_A)
                                  ? clamp_i16((int32_t)lroundf(m.iq_setpoint_A * 100.0f))
                                  : (int16_t)-32768;
  const int16_t iq_x100 = isfinite(m.iq_measured_A)
                              ? clamp_i16((int32_t)lroundf(m.iq_measured_A * 100.0f))
                              : (int16_t)-32768;

  const uint8_t axis_state_u8 = m.seen_heartbeat ? m.axis_state : 255;
  const uint8_t axis_flags_u8 = m.seen_heartbeat ? m.axis_flags : 0;

  const uint8_t can_ok = (last_rx_ms > 0 && (millis() - last_rx_ms) <= 1200) ? 1 : 0;

  const uint32_t axis_error_u32 = m.axis_error;
  const uint32_t detail_error_u32 = m.detail_error_u32;
  const uint8_t detail_type_u8 = m.last_error_type;

  memset(ble_frame_tx, 0, sizeof(ble_frame_tx));
  ble_frame_tx[0] = 0xA5;
  ble_frame_tx[1] = 0x5A;
  ble_frame_tx[2] = FRAME_LEN;

  int i = 3;
  put_u16_le(ble_frame_tx, i, t_cs);          // payload[0..1]
  put_i16_le(ble_frame_tx, i, pos_x10);       // payload[2..3]
  put_i16_le(ble_frame_tx, i, spd_x100);      // payload[4..5]
  put_i16_le(ble_frame_tx, i, tau_x100);      // payload[6..7]
  put_i16_le(ble_frame_tx, i, tau_set_x100);  // payload[8..9]
  put_i16_le(ble_frame_tx, i, iq_x100);       // payload[10..11]
  put_i16_le(ble_frame_tx, i, iq_set_x100);   // payload[12..13]
  ble_frame_tx[i++] = axis_state_u8;          // payload[14]
  ble_frame_tx[i++] = axis_flags_u8;          // payload[15]
  ble_frame_tx[i++] = active_id;              // payload[16]
  ble_frame_tx[i++] = can_ok;                 // payload[17]
  put_u32_le(ble_frame_tx, i, axis_error_u32);    // payload[18..21]
  ble_frame_tx[i++] = active_mode;                // payload[22]
  ble_frame_tx[i++] = requested_axis_state;       // payload[23]
  put_u32_le(ble_frame_tx, i, detail_error_u32);  // payload[24..27]
  ble_frame_tx[i++] = detail_type_u8;             // payload[28]

  Serial5.write((uint8_t *)ble_frame_tx, (size_t)FRAME_LEN);
}

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  Serial5.setTimeout(10);
  delay(600);

  Serial.println("=== SIG CAN SIMPLE BLE START ===");

  initial_CAN();
  motor_node1.initial_CAN();
  motor_node2.initial_CAN();

  const bool ok = initial_motor();
  Serial.print("[OK] init feedback=");
  Serial.println(ok ? "YES" : "NO");
}

void loop() {
  Receive_ble_Data();
  recover_can_if_needed();

  const uint32_t now_us = micros();
  if (now_us - last_ctrl_us >= 10000) {  // 100Hz
    last_ctrl_us = now_us;

    for (int i = 0; i < 4; ++i) {
      receive_motor_feedback();
    }

    if (configure_active_mode()) {
      apply_active_mode_command();
    }

    const uint32_t now_ms = millis();
    if (now_ms - last_req_ms >= 20) {  // 50Hz 请求反馈
      last_req_ms = now_ms;
      request_feedback_once();
    }
    if (now_ms - last_err_req_ms >= 120) {  // ~8Hz 轮询详细错误
      last_err_req_ms = now_ms;
      const uint8_t err_types[4] = {0, 1, 3, 4};
      active_motor().request_error_detail(err_types[err_req_idx & 0x03]);
      err_req_idx++;
      tx_total++;
    }
  }

  const uint32_t now_ms = millis();
  if (now_ms - last_ble_tx_ms >= 50) {  // 20Hz 上传
    last_ble_tx_ms = now_ms;
    Transmit_ble_Data();
  }
}
