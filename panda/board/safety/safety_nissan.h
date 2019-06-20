const int NISSAN_MAX_STEER = 2047; // 1s
// real time torque limit to prevent controls spamming
// the real time limit is 1500/sec
const int NISSAN_MAX_RT_DELTA = 940;          // max delta torque allowed for real time checks
const int32_t NISSAN_RT_INTERVAL = 250000;    // 250ms between real time checks
const int NISSAN_MAX_RATE_UP = 50;
const int NISSAN_MAX_RATE_DOWN = 70;
const int NISSAN_DRIVER_TORQUE_ALLOWANCE = 60;
const int NISSAN_DRIVER_TORQUE_FACTOR = 10;

int nissan_cruise_engaged_last = 0;
int nissan_rt_torque_last = 0;
int nissan_desired_torque_last = 0;
uint32_t nissan_ts_last = 0;
struct sample_t nissan_torque_driver;         // last few driver torques measured

static void nissan_init(int16_t param) {
  #ifdef PANDA
    lline_relay_init();
  #endif
}

static void nissan_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus_number = (to_push->RDTR >> 4) & 0xFF;
  uint32_t addr = to_push->RIR >> 21;

  // if ((addr == 0x119) && (bus_number == 0)){
  //   int torque_driver_new = ((to_push->RDLR >> 16) & 0x7FF);
  //   torque_driver_new = to_signed(torque_driver_new, 11);
  //   // update array of samples
  //   update_sample(&nissan_torque_driver, torque_driver_new);
  // }

  // enter controls on rising edge of ACC, exit controls on ACC off
  if ((addr == 0x1b6) && (bus_number == 1)) {
    int cruise_engaged = (to_push->RDHR >> 6) & 1;
    if (cruise_engaged && !nissan_cruise_engaged_last) {
      controls_allowed = 1;
    } else if (!cruise_engaged) {
      controls_allowed = 0;
    }
    nissan_cruise_engaged_last = cruise_engaged;
  }
}

static int nissan_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  uint32_t addr = to_send->RIR >> 21;

  // // steer cmd checks
  // if (addr == 0x122) {
  //   int desired_torque = ((to_send->RDLR >> 16) & 0x1FFF);
  //   int violation = 0;
  //   uint32_t ts = TIM2->CNT;
  //   desired_torque = to_signed(desired_torque, 13);

  //   if (controls_allowed) {

  //     // *** global torque limit check ***
  //     violation |= max_limit_check(desired_torque, NISSAN_MAX_STEER, -NISSAN_MAX_STEER);

  //     // *** torque rate limit check ***
  //     int desired_torque_last = nissan_desired_torque_last;
  //     violation |= driver_limit_check(desired_torque, desired_torque_last, &nissan_torque_driver,
  //       NISSAN_MAX_STEER, NISSAN_MAX_RATE_UP, NISSAN_MAX_RATE_DOWN,
  //       NISSAN_DRIVER_TORQUE_ALLOWANCE, NISSAN_DRIVER_TORQUE_FACTOR);

  //     // used next time
  //     nissan_desired_torque_last = desired_torque;

  //     // *** torque real time rate limit check ***
  //     violation |= rt_rate_limit_check(desired_torque, nissan_rt_torque_last, NISSAN_MAX_RT_DELTA);

  //     // every RT_INTERVAL set the new limits
  //     uint32_t ts_elapsed = get_ts_elapsed(ts, nissan_ts_last);
  //     if (ts_elapsed > NISSAN_RT_INTERVAL) {
  //       nissan_rt_torque_last = desired_torque;
  //       nissan_ts_last = ts;
  //     }
  //   }

  //   // no torque if controls is not allowed
  //   if (!controls_allowed && (desired_torque != 0)) {
  //     violation = 1;
  //   }

  //   // reset to 0 if either controls is not allowed or there's a violation
  //   if (violation || !controls_allowed) {
  //     nissan_desired_torque_last = 0;
  //     nissan_rt_torque_last = 0;
  //     nissan_ts_last = ts;
  //   }

  //   if (violation) {
  //     return false;
  //   }

  // }
  return true;
}

static int nissan_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  // shifts bits 29 > 11
  int32_t addr = to_fwd->RIR >> 21;

  // forward CAN 0 > 2
  if (bus_num == 0) {

    return 2;
  }
  // forward CAN 2 > 0
  else if (bus_num == 2) {

    return 0; // Main CAN
  }

  // fallback to do not forward
  return -1;
}

const safety_hooks nissan_hooks = {
  .init = nissan_init,
  .rx = nissan_rx_hook,
  .tx = nissan_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = default_ign_hook,
  .fwd = nissan_fwd_hook,
  .relay = alloutput_relay_hook,
};
