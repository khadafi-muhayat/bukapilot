def create_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Toyota Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
  }
  return packer.make_can_msg("STEERING_LKA", 0, values)


def create_lta_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Toyota LTA Steer Command."""

  values = {
    "COUNTER": raw_cnt + 128,
    "SETME_X1": 1,
    "SETME_X3": 3,
    "PERCENTAGE": 100,
    "SETME_X64": 0x64,
    "ANGLE": 0,  # Rate limit? Lower values seeem to work better, but needs more testing
    "STEER_ANGLE_CMD": steer,
    "STEER_REQUEST": steer_req,
    "STEER_REQUEST_2": steer_req,
    "BIT": 0,
  }
  return packer.make_can_msg("STEERING_LTA", 0, values)

def create_acttr_steer_command(packer, command, direction, enable, idx):
  """Creates a CAN message for the Kommu actuator steering command."""

  values = {
    "INTERCEPTOR_MAIN_TORQUE": abs(command),
    "INTERCEPTOR_SUB_TORQUE": abs(command),
    "DIRECTION": 0 if direction else 1,
    "ENABLE": 1 if enable else 0,
    "COUNTER_STEERING": idx & 0xF,
  }

  return packer.make_can_msg("TORQUE_COMMAND", 0, values)

def create_acttr_gas_command(packer, gas_amount, enable, idx):
  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    # the value 3000 is a limiting constant, allow an addition of
    # 2500/4095 * 3.3 = 2.01V.
    values["GAS_COMMAND"] = gas_amount
    values["GAS_COMMAND2"] = gas_amount

    return packer.make_can_msg("GAS_COMMAND", 0, values)

def create_accel_command(packer, accel, pcm_cancel, standstill_req, lead, acc_type):
  # TODO: find the exact canceling bit that does not create a chime
  values = {
    "ACCEL_CMD": accel,
    "ACC_TYPE": acc_type,
    "DISTANCE": 0,
    "MINI_CAR": lead,
    "SET_ME_X3": 3,
    "PERMIT_BRAKING": 1,
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
  }
  return packer.make_can_msg("ACC_CONTROL", 0, values)


def create_acc_cancel_command(packer):
  values = {
    "GAS_RELEASED": 0,
    "CRUISE_ACTIVE": 0,
    "STANDSTILL_ON": 0,
    "ACCEL_NET": 0,
    "CRUISE_STATE": 0,
    "CANCEL_REQ": 1,
  }
  return packer.make_can_msg("PCM_CRUISE", 0, values)


def create_fcw_command(packer, fcw):
  values = {
    "FCW": fcw,
    "SET_ME_X20": 0x20,
    "SET_ME_X10": 0x10,
    "SET_ME_X80": 0x80,
  }
  return packer.make_can_msg("ACC_HUD", 0, values)


def create_ui_command(packer, steer, chime, left_line, right_line, left_lane_depart, right_lane_depart):
  values = {
    "RIGHT_LINE": 3 if right_lane_depart else 1 if right_line else 2,
    "LEFT_LINE": 3 if left_lane_depart else 1 if left_line else 2,
    "BARRIERS" : 3 if left_lane_depart else 2 if right_lane_depart else 0,
    "SET_ME_X0C": 0x0c,
    "SET_ME_X2C": 0x2c,
    "SET_ME_X38": 0x38,
    "SET_ME_X02": 0x02,
    "SET_ME_X01": 1,
    "SET_ME_X01_2": 1,
    "REPEATED_BEEPS": 0,
    "TWO_BEEPS": chime,
    "LDA_ALERT": steer,
  }
  return packer.make_can_msg("LKAS_HUD", 0, values)
