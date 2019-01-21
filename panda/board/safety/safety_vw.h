void vw_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {}

int vw_ign_hook() {
  return -1; // use GPIO to determine ignition
}

// FIXME
// *** all output safety mode ***

static void vw_init(int16_t param) {
  controls_allowed = 1;
}

static int vw_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  return true;
}

static int vw_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  return true;
}

static int vw_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  
  // shifts bits from 29 to 11
  int32_t addr = to_fwd->RIR >> 21;

  // forward messages from extended to camera
  if (bus_num == 0) {
    return 2; //camera 
  }
  // forward messages from radar to camera
  else if (bus_num == 1) {
    return 2; // camera 
  }
  // forward messages from camera to radar
  else if (bus_num == 2) {
    return 1; // radar 
  }

  // fallback to do not forward
  return -1;
}

const safety_hooks vw_hooks = {
  .init = vw_init,
  .rx = vw_rx_hook,
  .tx = vw_tx_hook,
  .tx_lin = vw_tx_lin_hook,
  .ignition = vw_ign_hook,
  .fwd = vw_fwd_hook,
};