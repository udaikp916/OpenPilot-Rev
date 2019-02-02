const int SUBARU_MAX_STEER = 2047;
const int SUBARU_MAX_RT_DELTA = 800;          // max delta torque allowed for real time checks
const int32_t SUBARU_RT_INTERVAL = 250000;    // 250ms between real time checks
const int SUBARU_MAX_RATE_UP = 60;
const int SUBARU_MAX_RATE_DOWN = 60;
const int SUBARU_DRIVER_TORQUE_ALLOWANCE = 20;
const int SUBARU_DRIVER_TORQUE_FACTOR = 1;

int subaru_rt_torque_last = 0;
int subaru_desired_torque_last = 0;
int subaru_cruise_engaged_last = 0;
uint32_t subaru_ts_last = 0;
struct sample_t subaru_torque_driver;         // last few driver torques measured

static void subaru_init(int16_t param) {
  controls_allowed = 1;
}

static int subaru_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  uint32_t addr;
  addr = to_send->RIR >> 21;

  // LKA STEER: safety check
  if (addr == 356) {
    int desired_torque = ((to_send->RDLR >> 8) & 0x1fff);
    uint32_t ts = TIM2->CNT;
    int violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, SUBARU_MAX_STEER, -SUBARU_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, subaru_desired_torque_last, &subaru_torque_driver,
        SUBARU_MAX_STEER, SUBARU_MAX_RATE_UP, SUBARU_MAX_RATE_DOWN,
        SUBARU_DRIVER_TORQUE_ALLOWANCE, SUBARU_DRIVER_TORQUE_FACTOR);

      // used next time
      subaru_desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, subaru_rt_torque_last, SUBARU_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, subaru_ts_last);
      if (ts_elapsed > SUBARU_RT_INTERVAL) {
        subaru_rt_torque_last = desired_torque;
        subaru_ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      subaru_desired_torque_last = 0;
      subaru_rt_torque_last = 0;
      subaru_ts_last = ts;
    }

    if (violation) {
      return false;
    }
  }

  // True allows the message through
  return true;
}

static int subaru_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  
  // shifts bits 29 > 11
  int32_t addr = to_fwd->RIR >> 21;

  // forward CAN 0 > 1
  if (bus_num == 0) {
    return 1; // ES CAN
  }
  // forward CAN 1 > 0, except ES_LKAS
  else if (bus_num == 1) {
    
    // subaru global
    //if (addr == 0x122) {
    //  return -1;
    //}
    // outback 2015
    if (addr == 356) {
      return -1;
    }

    return 0; // Main CAN
  }

  // fallback to do not forward
  return -1;
}

const safety_hooks subaru_hooks = {
  .init = nooutput_init,
  .rx = subaru_rx_hook,
  .tx = subaru_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = default_ign_hook,
  .fwd = subaru_fwd_hook,
};