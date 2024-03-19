// Safety-relevant CAN messages for PSA vehicles.
#define PSA_DAT_BSI              1042
#define PSA_LANE_KEEP_ASSIST     1010
// Messages on the ADAS bus.
#define PSA_HS2_DYN_ABR_38D      909
#define PSA_HS2_BGE_DYN5_CMM_228 552
#define PSA_HS2_DAT_MDD_CMD_452  1106

// CAN bus numbers.
#define PSA_MAIN_BUS 0U
#define PSA_ADAS_BUS 1U
#define PSA_CAM_BUS  2U

const CanMsg PSA_TX_MSGS[] = {
  {PSA_LANE_KEEP_ASSIST, 0, 8},
};

RxCheck psa_rx_checks[] = {
  // TODO: counters and checksums
  {.msg = {{PSA_DAT_BSI, PSA_MAIN_BUS, 8, .frequency = 20U}, { 0 }, { 0 }}},
  {.msg = {{PSA_HS2_DYN_ABR_38D, PSA_ADAS_BUS, 8, .frequency = 25U}, { 0 }, { 0 }}},
  {.msg = {{PSA_HS2_BGE_DYN5_CMM_228, PSA_ADAS_BUS, 8, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{PSA_HS2_DAT_MDD_CMD_452, PSA_ADAS_BUS, 6, .frequency = 20U}, { 0 }, { 0 }}},
};

static bool psa_lkas_msg_check(int addr) {
  return addr == PSA_LANE_KEEP_ASSIST;
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
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == PSA_MAIN_BUS) {
    // Update brake pedal
    if (addr == PSA_DAT_BSI) {
      // Signal: P013_MainBrake
      brake_pressed = GET_BIT(to_push, 5);
    }

    bool stock_ecu_detected = psa_lkas_msg_check(addr);
    generic_rx_checks(stock_ecu_detected);
  }

  if (bus == PSA_ADAS_BUS) {
    // Update vehicle speed and in motion state
    if (addr == PSA_HS2_DYN_ABR_38D) {
      // Signal: VITESSE_VEHICULE_ROUES
      int speed = (GET_BYTE(to_push, 0) << 8) | GET_BYTE(to_push, 1);
      vehicle_moving = speed > 0;
      UPDATE_VEHICLE_SPEED(speed * 0.01);
    }

    // Update gas pedal
    if (addr == PSA_HS2_BGE_DYN5_CMM_228) {
      // Signal: EFCMNT_PDLE_ACCEL
      gas_pressed = GET_BYTE(to_push, 2) > 0U;
    }

    // Update cruise state
    if (addr == PSA_HS2_DAT_MDD_CMD_452) {
      // Signal: DDE_ACTIVATION_RVV_ACC
      pcm_cruise_check(GET_BIT(to_push, 23));
    }
  }
}

static bool psa_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);

  // TODO: Safety check for cruise buttons
  // TODO: check resume is not pressed when controls not allowed
  // TODO: check cancel is not pressed when cruise isn't engaged

  // Safety check for LKA
  if (addr == PSA_LANE_KEEP_ASSIST) {
    // Signal: ANGLE
    int desired_angle = to_signed((GET_BYTE(to_send, 6) << 6) | ((GET_BYTE(to_send, 7) & 0xFC) >> 2), 14);
    // Signal: STATUS
    bool lka_active = (GET_BYTE(to_send, 4) & 0x18) == 2U;

    if (steer_angle_cmd_checks(desired_angle, lka_active, PSA_STEERING_LIMITS)) {
      tx = false;
    }
  }

  return tx;
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
