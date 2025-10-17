#pragma once

#include "opendbc/safety/safety_declarations.h"

static uint8_t vinfast_get_counter(const CANPacket_t *msg) {
(void)msg;
  print("I'm from vinfast_get_counter\n");
  return 0;
}

static uint32_t vinfast_get_checksum(const CANPacket_t *msg) {
(void)msg;
  print("I'm from vinfast_get_checksum\n");
  return 0;
}

static uint32_t vinfast_compute_checksum(const CANPacket_t *msg) {

(void)msg;

  print("I'm from vinfast_compute_checksum\n");
  return 0;}

static bool vinfast_get_quality_flag_valid(const CANPacket_t *msg) {
  (void )msg;
  print("I'm from vinfast_get_quality_flag_valid\n");
  return true;}

static void vinfast_rx_hook(const CANPacket_t *msg) {

  (void )msg;
  print("I'm from vinfast_rx_hook\n");
}

static bool vinfast_tx_hook(const CANPacket_t *msg) {
  (void )msg;
  print("I'm from vinfast_tx_hook hehe\n");
  return true;}

static bool vinfast_fwd_hook(int bus_num, int addr) {
  (void )bus_num;
  (void )addr;

  print("I'm from vinfast_fwd_hook\n");
  return true;}

static safety_config vinfast_init(uint16_t param) {
  (void)param;
  safety_config config = {
    .rx_checks = NULL,
    .rx_checks_len = 0,
    .tx_msgs = NULL,
    .tx_msgs_len = 0,
    .disable_forwarding = false
  };
  print("I'm from vinfast_init\n");
  return config;
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

