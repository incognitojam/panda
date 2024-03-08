// Safety-relevant CAN messages for PSA vehicles.
#define PSA_CRUISE           0
#define PSA_CRUISE_BUTTONS   0
#define PSA_POWERTRAIN       520
#define PSA_WHEEL_SPEEDS     781
#define PSA_LANE_KEEP_ASSIST 1010
#define PSA_DRIVER           1390

// CAN bus numbers.
#define PSA_MAIN_BUS 0U
#define PSA_CAM_BUS  2U

const CanMsg PSA_TX_MSGS[] = {
  {PSA_CRUISE_BUTTONS, 0, 8},
  {PSA_LANE_KEEP_ASSIST, 0, 8},
};

RxCheck psa_rx_checks[] = {
  // TODO: check frequencies
  // TODO: counters and checksums
  {.msg = {{PSA_CRUISE, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},
  {.msg = {{PSA_POWERTRAIN, 0, 8, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{PSA_WHEEL_SPEEDS, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{PSA_DRIVER, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},
};

static bool psa_lkas_msg_check(int addr) {
  return (addr == PSA_LANE_KEEP_ASSIST);
}

// TODO: update rate limits
const SteeringLimits PSA_STEERING_LIMITS = {
  .angle_deg_to_can = 10,
  .angle_rate_up_lookup = {
    {0., 5., 15.},
    {10., 1.6, .30},
  },
  .angle_rate_down_lookup = {
    {0., 5., 15.},
    {10., 7.0, .8},
  },
};

static void psa_rx_hook(const CANPacket_t *to_push) {
  if (GET_BUS(to_push) == PSA_MAIN_BUS) {
    int addr = GET_ADDR(to_push);

    // TODO: find cruise state

    // Update wheel speeds and in motion state
    if (addr == PSA_WHEEL_SPEEDS) {
      // TODO: find front/rear wheels
      int wheel_speed_1 = (GET_BYTE(to_push, 0) << 8) | GET_BYTE(to_push, 1);
      int wheel_speed_2 = (GET_BYTE(to_push, 2) << 8) | GET_BYTE(to_push, 3);
      int wheel_speed_3 = (GET_BYTE(to_push, 4) << 8) | GET_BYTE(to_push, 5);
      int wheel_speed_4 = (GET_BYTE(to_push, 6) << 8) | GET_BYTE(to_push, 7);
      // TODO: find standstill signal
      vehicle_moving = (wheel_speed_1 | wheel_speed_2 | wheel_speed_3 | wheel_speed_4) != 0U;
      UPDATE_VEHICLE_SPEED((wheel_speed_1 + wheel_speed_2 + wheel_speed_3 + wheel_speed_4) / 4.0 * 0.01);
    }

    // Update gas pedal
    if (addr == PSA_DRIVER) {
      // Pedal position: (0.5 * val)% (up to 99.5%)
      gas_pressed = GET_BYTE(to_push, 3) > 0U;
    }

    // Update brake pedal
    if (addr == PSA_POWERTRAIN) {
      brake_pressed = GET_BIT(to_push, 33);
    }

    bool stock_ecu_detected = psa_lkas_msg_check(addr);
    generic_rx_checks(stock_ecu_detected);
  }
}

static bool psa_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);

  // Safety check for cruise buttons
  if (addr == PSA_CRUISE_BUTTONS) {
    bool violation = false;
    // TODO: check resume is not pressed when controls not allowed
    // TODO: check cancel is not pressed when cruise isn't engaged

    if (violation) {
      tx = false;
    }
  }

  // Safety check for LKA
  if (addr == PSA_LANE_KEEP_ASSIST) {
    int desired_angle = to_signed((GET_BYTE(to_send, 6) << 6) | ((GET_BYTE(to_send, 7) & 0xFC) >> 2), 14);
    bool lka_active = GET_BIT(to_send, 36);

    if (steer_angle_cmd_checks(desired_angle, lka_active, PSA_STEERING_LIMITS)) {
      tx = false;
    }
  }
}

static int psa_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;
  switch (bus_num) {
    case PSA_MAIN_BUS: {
      // Forward all traffic from MAIN to CAM
      bus_fwd = PSA_CAM_BUS;
      break;
    }
    case PSA_CAM_BUS: {
      if (psa_lkas_msg_check(addr)) {
        // Block stock LKAS messages
        bus_fwd = -1;
      } else {
        // Forward all other traffic from CAM to MAIN
        bus_fwd = PSA_MAIN_BUS;
      }
      break;
    }
    default: {
      // No other buses should be in use; fallback to block
      bus_fwd = -1;
      break;
    }
  }
  return bus_fwd;
}

static safety_config psa_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(psa_rx_checks, PSA_TX_MSGS);
}

const safety_hooks psa_hooks = {
  .init = psa_init,
  .rx = psa_rx_hook,
  .tx = psa_tx_hook,
  .fwd = psa_fwd_hook,
};
