#pragma once

#include "opendbc/safety/safety_declarations.h"

// CAN message addresses (decimal from DBC)
#define VINFAST_ADAS_EPS_LATE_CON 890U  // 0x37A - Steering control from SCAM
#define VINFAST_ADAS_IDB_APA      309U  // 0x135 - ACC control from ADAS_Chassis
#define VINFAST_ADAS_LKA          306U  // 0x132 - LKA control from SCAM
#define VINFAST_ADAS_ACC_STATUS   813U  // 0x32D - ACC status (longitudinal control) from SCAM
#define VINFAST_EPS_STEERING_TRQ  891U  // 0x37B - Steering torque feedback from EPS
#define VINFAST_IDB_STATUS        525U  // 0x20D - Vehicle status from IDB_Chassis
#define VINFAST_SAS_SENSOR        382U  // 0x17E - Steering angle sensor from EPS
#define VINFAST_BCM_CLAMP_STAT    274U  // 0x112 - Brake light status from XGW_Chassis

// CAN bus assignments
// Note: Bus numbers need to be verified based on actual vehicle wiring
// Assuming chassis bus is 0, camera/SCAM might be on bus 2
#define VINFAST_CHASSIS_BUS 0U
#define VINFAST_CAMERA_BUS  2U  // SCAM messages might be on camera bus

// Steering angle limits (VF8 uses angle-based control)
// MAX_EPS_ANGLE = 470 degrees (from /data/card values.h)
static const AngleSteeringLimits VINFAST_STEERING_LIMITS = {
  .max_angle = 47000,  // centi-degrees (470 degrees)
  .angle_deg_to_can = 1.0,  // 1:1 conversion (already in degrees)
  .angle_rate_up_lookup = {
    {0., 20., 40.},  // speed breakpoints (m/s)
    {20., 10., 5.}   // angle rate limits (degrees per step)
  },
  .angle_rate_down_lookup = {
    {0., 20., 40.},  // speed breakpoints (m/s)
    {20., 10., 5.}   // angle rate limits (degrees per step)
  },
  .max_angle_error = 1000,  // centi-degrees (10 degrees)
  .angle_error_min_speed = 0.0,  // m/s
  .frequency = 20U,  // Hz
  .angle_is_curvature = false,
  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

// Longitudinal acceleration limits (from values.py)
static const LongitudinalLimits VINFAST_LONG_LIMITS = {
  .max_accel = 200,   // 2.0 m/s² in 1/100 m/s² units
  .min_accel = -350,  // -3.5 m/s² in 1/100 m/s² units
  .inactive_accel = 0,
};

static uint8_t vinfast_get_counter(const CANPacket_t *msg) {
  // Extract counter from alive counter field
  // ALV_ADAS_EPS_LATE_CON is in bits 11-14 (4 bits, 0-14)
  if (msg->addr == VINFAST_ADAS_EPS_LATE_CON) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  // ALV_ADAS_LKA is in bits 11-14
  if (msg->addr == VINFAST_ADAS_LKA) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  // Alive_ACC_STATUS is in bits 11-14
  if (msg->addr == VINFAST_ADAS_ACC_STATUS) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  // ALV_IDB_STATUS is in bits 11-14
  if (msg->addr == VINFAST_IDB_STATUS) {
    return (msg->data[1] >> 3) & 0xFU;
  }
  return 0;
}

static uint32_t vinfast_get_checksum(const CANPacket_t *msg) {
  // Extract checksum from message
  // CHKSM/CRC is typically in byte 0, bits 7-0
  if (msg->addr == VINFAST_ADAS_EPS_LATE_CON) {
    return msg->data[0];
  }
  if (msg->addr == VINFAST_ADAS_IDB_APA) {
    return msg->data[0];
  }
  if (msg->addr == VINFAST_ADAS_LKA) {
    return msg->data[0];
  }
  if (msg->addr == VINFAST_ADAS_ACC_STATUS) {
    return msg->data[0];  // CRC_ACC_STATUS
  }
  if (msg->addr == VINFAST_IDB_STATUS) {
    return msg->data[0];
  }
  return 0;
}

// CRC-8 lookup table for VinFast checksum (from /data/card implementation)
static const uint8_t vinfast_crc8_table[256] = {
  0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF,
  0x9C, 0x81, 0xA6, 0xBB, 0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
  0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76, 0x87, 0x9A, 0xBD, 0xA0,
  0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
  0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85,
  0xD6, 0xCB, 0xEC, 0xF1, 0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
  0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8, 0xDE, 0xC3, 0xE4, 0xF9,
  0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
  0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B,
  0x08, 0x15, 0x32, 0x2F, 0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
  0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2, 0x26, 0x3B, 0x1C, 0x01,
  0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
  0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24,
  0x77, 0x6A, 0x4D, 0x50, 0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
  0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A, 0x6C, 0x71, 0x56, 0x4B,
  0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
  0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA,
  0xA9, 0xB4, 0x93, 0x8E, 0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
  0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43, 0xB2, 0xAF, 0x88, 0x95,
  0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
  0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0,
  0xE3, 0xFE, 0xD9, 0xC4
};

static uint32_t vinfast_compute_checksum(const CANPacket_t *msg) {
  // VinFast uses CRC-8 checksum on bytes 1 to end (excluding first byte)
  // Initial value: 0xFF, Final XOR: 0xFF
  uint8_t crc = 0xFF;
  int len = GET_LEN(msg);
  
  // Calculate checksum on bytes 1 to len-1 (skip first byte)
  for (int i = 1; i < len; i++) {
    crc = vinfast_crc8_table[crc ^ msg->data[i]];
  }
  crc ^= 0xFF;  // Final XOR with 0xFF
  
  return (uint32_t)crc;
}

static bool vinfast_get_quality_flag_valid(const CANPacket_t *msg) {
  // TODO: Implement quality flag validation if VinFast uses it
  (void)msg;
  return true;
}

static void vinfast_rx_hook(const CANPacket_t *msg) {
  // Parse incoming CAN messages to update vehicle state
  
  if (msg->bus == VINFAST_CHASSIS_BUS) {
    // Vehicle speed from IDB_STATUS
    if (msg->addr == VINFAST_IDB_STATUS) {
      // VehicleSpd: start bit 23, length 13 bits, factor 0.05625
      // Bits 23-35 span bytes 2-4 (little endian)
      uint32_t speed_raw = GET_BYTES(msg, 2, 3) >> 7U;  // Get 3 bytes, shift right 7 bits
      speed_raw &= 0x1FFFU;  // Mask to 13 bits
      float speed_kph = (float)speed_raw * 0.05625f;
      UPDATE_VEHICLE_SPEED(speed_kph * KPH_TO_MS);
      
      // Standstill check: ESC_VehicleStandstill is bit 24
      bool standstill = GET_BIT(msg, 24U);
      vehicle_moving = !standstill;
    }
    
    // Steering torque feedback from EPS
    if (msg->addr == VINFAST_EPS_STEERING_TRQ) {
      // EPS_SteeringDriverTorque: start bit 30, length 12 bits, factor 0.01, offset -10.24
      // Bits 30-41 span bytes 3-5 (little endian)
      uint32_t torque_raw = GET_BYTES(msg, 3, 3) >> 6U;  // Get 3 bytes, shift right 6 bits
      torque_raw &= 0xFFFU;  // Mask to 12 bits
      // Convert from 0.01 Nm units: raw value represents (value * 0.01 - 10.24)
      // For safety checks, we use raw units, so multiply by 100 to get centi-Nm
      int torque_driver_new = (int)torque_raw * 100;
      update_sample(&torque_driver, torque_driver_new);
    }
    
    // Steering angle from SAS_Sensor (for angle-based control)
    if (msg->addr == VINFAST_SAS_SENSOR) {
      // SAS_SteerWheelAngle: start bit 47, length 16 bits, factor 0.0238, offset -780
      // Bits 47-62 span bytes 5-7 (little endian)
      uint32_t angle_raw = GET_BYTES(msg, 5, 3) >> 7U;  // Get 3 bytes, shift right 7 bits
      angle_raw &= 0xFFFFU;  // Mask to 16 bits
      // Convert to degrees: (raw * 0.0238) - 780
      float angle_deg = ((float)angle_raw * 0.0238f) - 780.0f;
      // Update angle measurement for safety checks (convert to centi-degrees)
      int angle_meas_new = (int)(angle_deg * 100.0f);
      update_sample(&angle_meas, angle_meas_new);
    }
    
    // Brake light status from BCM_CLAMP_STAT
    if (msg->addr == VINFAST_BCM_CLAMP_STAT) {
      // STAT_BLS: start bit 15, length 2 bits
      // 0=off, 1=on, 2=error, 3=not available
      uint32_t brake_light = (GET_BYTES(msg, 1, 2) >> 7U) & 0x3U;
      brake_pressed = (brake_light == 1U);
    }
  }
  
  // Camera bus messages (SCAM)
  if (msg->bus == VINFAST_CAMERA_BUS) {
    // ACC status messages might be here
    // TODO: Add ACC status parsing if needed
  }
}

static bool vinfast_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  
  // Check steering control messages (angle-based for VF8)
  if (msg->addr == VINFAST_ADAS_EPS_LATE_CON) {
    // Extract angle request (ADAS_EPS_AOLReq)
    // Start bit 39, length 16 bits, factor 0.0238, offset -780, range -780 to 779.7 degrees
    // Bits 39-54 span bytes 4-6 (little endian)
    uint32_t angle_raw = GET_BYTES(msg, 4, 3) >> 7U;  // Get 3 bytes, shift right 7 bits
    angle_raw &= 0xFFFFU;  // Mask to 16 bits
    // Convert to degrees: (raw * 0.0238) - 780
    float desired_angle_deg = ((float)angle_raw * 0.0238f) - 780.0f;
    int desired_angle = (int)(desired_angle_deg * 100.0f);  // Convert to centi-degrees
    
    // Check AOLAct: Angle Override Limit Active (bit 27-28)
    uint32_t aol_act = (GET_BYTES(msg, 3, 1) >> 3U) & 0x3U;
    bool steer_control_enabled = (aol_act == 1U);
    
    // Use angle-based steering checks
    if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, VINFAST_STEERING_LIMITS)) {
      tx = false;
    }
  }
  
  // Check ACC control messages (ADAS_IDB_APA)
  if (msg->addr == VINFAST_ADAS_IDB_APA) {
    // Validate ACC control request
    // ADAS_IDB_ControlReq is bit 31
    bool control_req = GET_BIT(msg, 31U);
    
    // Only allow ACC control when controls are allowed
    if (control_req && !controls_allowed) {
      tx = false;
    }
    
    // TODO: Add longitudinal acceleration checks if needed
    // ADAS_IDB_TargetDecel: start bit 30, length 10 bits, factor 0.01
  }
  
  // Check ACC status messages (ADAS_ACC_Status) - longitudinal control
  if (msg->addr == VINFAST_ADAS_ACC_STATUS) {
    // Extract acceleration command (ADAS_ACC_AccelDecel_Cmd)
    // Start bit 19, length 12 bits, factor 0.005, offset -6, range -6 to 6 m/s²
    // Bits 19-30 span bytes 2-4 (little endian)
    uint32_t accel_raw = GET_BYTES(msg, 2, 3) >> 3U;  // Get 3 bytes, shift right 3 bits
    accel_raw &= 0xFFFU;  // Mask to 12 bits
    // Convert to signed: raw value represents (value * 0.005 - 6.0)
    int accel_cmd = (int)accel_raw;
    if (accel_cmd > 2047) {
      accel_cmd = accel_cmd - 4096;  // Convert to signed
    }
    // Convert to 1/100 m/s² units for safety checks
    int desired_accel = (int)((accel_cmd * 0.005f - 6.0f) * 100.0f);
    
    // Use longitudinal acceleration checks
    if (longitudinal_accel_checks(desired_accel, VINFAST_LONG_LIMITS)) {
      tx = false;
    }
    
    // Check ACC_Main_Mode: only allow when controls are allowed
    bool acc_main_mode = GET_BIT(msg, 15U);
    if (acc_main_mode && !controls_allowed) {
      tx = false;
    }
  }
  
  // Check LKA control messages
  if (msg->addr == VINFAST_ADAS_LKA) {
    // LSS_Activation: start bit 15, length 2 bits
    // Bits 15-16 span bytes 1-2
    uint32_t lss_raw = GET_BYTES(msg, 1, 2) >> 7U;
    int lss_activation = (int)(lss_raw & 0x3U);
    
    // Only allow LKA activation when controls are allowed
    // 2 = active, 3 = error
    if ((lss_activation == 2U) && !controls_allowed) {
      tx = false;
    }
  }
  
  return tx;
}

static bool vinfast_fwd_hook(int bus_num, int addr) {
  // Forwarding logic - allow forwarding by default
  // Block specific messages if needed
  (void)bus_num;
  (void)addr;
  return false;  // false = allow forwarding
}

static safety_config vinfast_init(uint16_t param) {
  UNUSED(param);
  
  // Define TX messages that openpilot will send
  static const CanMsg VINFAST_TX_MSGS[] = {
    {VINFAST_ADAS_EPS_LATE_CON, VINFAST_CHASSIS_BUS, 8, .check_relay = true},   // Steering control
    {VINFAST_ADAS_IDB_APA,      VINFAST_CHASSIS_BUS, 8, .check_relay = true},   // ACC control (legacy)
    {VINFAST_ADAS_ACC_STATUS,   VINFAST_CAMERA_BUS,  5, .check_relay = true},   // ACC status (longitudinal control)
    {VINFAST_ADAS_LKA,          VINFAST_CAMERA_BUS,  4, .check_relay = true},   // LKA control
  };
  
  // Define RX messages to monitor for safety
  static RxCheck vinfast_rx_checks[] = {
    {.msg = {{VINFAST_IDB_STATUS,       VINFAST_CHASSIS_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VINFAST_EPS_STEERING_TRQ, VINFAST_CHASSIS_BUS, 6, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VINFAST_SAS_SENSOR,       VINFAST_CHASSIS_BUS, 7, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{VINFAST_BCM_CLAMP_STAT,   VINFAST_CHASSIS_BUS, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  
  return BUILD_SAFETY_CFG(vinfast_rx_checks, VINFAST_TX_MSGS);
}

const safety_hooks vinfast_hooks = {
  .init = vinfast_init,
  .rx = vinfast_rx_hook,
  .tx = vinfast_tx_hook,
  .fwd = vinfast_fwd_hook,
  .get_counter = vinfast_get_counter,
  .get_checksum = vinfast_get_checksum,
  .compute_checksum = vinfast_compute_checksum,
  .get_quality_flag_valid = vinfast_get_quality_flag_valid,
};
