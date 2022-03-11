from common.numpy_fast import clip
from cereal import car

VisualAlert = car.CarControl.HUDControl.VisualAlert

def perodua_checksum(addr,dat):
  return ( addr + len(dat) + 1 + 1 + sum(dat)) & 0xFF

def perodua_acc_checksum(addr,dat):
  return ( addr + len(dat) + 1 + 2 + sum(dat)) & 0xFF

def create_can_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""

  values = {
    "STEER_REQ": steer_req,
    "STEER_CMD": -steer if steer_req else 0,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
    "SET_ME_1_2": 1,
  }

  dat = packer.make_can_msg("STEERING_LKAS", 0, values)[2]
  crc = perodua_checksum(0x1d0, dat[:-1])
  values["CHECKSUM"] = crc


  return packer.make_can_msg("STEERING_LKAS", 0, values)

def crc8_interceptor(data):
  crc = 0xFF                                                         # standard init value
  poly = 0xD5                                                        # standard crc8: x8+x7+x6+x4+x2+1
  size = len(data)
  for i in range(size - 1, -1, -1):
    crc ^= data[i]
    for _ in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
  return crc

def create_steer_command(packer, command, direction, enable, idx):
  """Creates a CAN message for the steering command."""

  values = {
    "INTERCEPTOR_MAIN_TORQUE": abs(command),
    "INTERCEPTOR_SUB_TORQUE": abs(command),
    "DIRECTION": 0 if direction else 1,
    "ENABLE": 1 if enable else 0,
    "COUNTER_STEERING": idx & 0xF,
  }

  dat = packer.make_can_msg("TORQUE_COMMAND", 0, values)[2]

  crc = crc8_interceptor(dat[:-1])
  values["CHECKSUM_STEERING"] = crc

  return packer.make_can_msg("TORQUE_COMMAND", 0, values)

def perodua_create_gas_command(packer, gas_amount, enable, idx):

  values = {
    "ENABLE": enable,
    "COUNTER_PEDAL": idx & 0xF,
  }

  if enable:
    # the value 3000 is a limiting constant, allow an addition of
    # 2500/4095 * 3.3 = 2.01V.
    values["GAS_COMMAND"] = gas_amount
    values["GAS_COMMAND2"] = gas_amount

  dat = packer.make_can_msg("GAS_COMMAND", 0, values)[2]

  checksum = crc8_interceptor(dat[:-1])
  values["CHECKSUM_PEDAL"] = checksum

  return packer.make_can_msg("GAS_COMMAND", 0, values)

def perodua_aeb_brake(packer, brake_amount):

  values = {
    "AEB_ALARM": 1 if (brake_amount > 0.5) else 0,
    "SET_ME_XB2": 0xB2,
  }

  return packer.make_can_msg("FWD_CAM3", 0, values)

def perodua_create_brake_command(packer, enabled, decel_cmd, idx):
  #decel_req = decel_cmd > 0
  decel_req = enabled

  values = {
    "COUNTER": idx,
    "BRAKE_REQ": decel_req and enabled,
    "RISING_PUMP_ON": enabled,
    #"SET_ME_XC8_WHEN_IDLE": -0.3599 if (enabled and decel_req) else 0,
    "SET_ME_XC8_WHEN_IDLE": -0.55 if (enabled and decel_req) else 0,
    "SET_ME_1_WHEN_ENGAGE": 1 if enabled else 0,
    "BRAKE_CMD": -1.95 if (enabled and decel_req) else 0,
  }

  dat = packer.make_can_msg("ACC_BRAKE", 0, values)[2]
  crc = (perodua_acc_checksum(0x271, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("ACC_BRAKE", 0, values)

def perodua_create_accel_command(packer, set_speed, accel_req, rising, accel_cmd, accel_brake):
  values = {
    "SET_SPEED": set_speed,
    "FOLLOW_DISTANCE": 0,
    "IS_LEAD": 1,
    "IS_ACCEL": 0 if (accel_brake <= 0 and accel_req) else 0,
    "IS_DECEL": 1 if accel_req else 0,
    "IS_DECEL": 0 if (accel_brake > 0 and accel_req) else 0,
    "SET_ME_1_2": 1,
    "SET_ME_1": 1,
    "SET_0_WHEN_ENGAGE": 0 if accel_req else 1,
    "SET_1_WHEN_ENGAGE": 1 if accel_req else 0,
    "NANI": 0,
    "ACC_CMD": accel_cmd if accel_req else 0,
  }

  dat = packer.make_can_msg("ACC_CMD_HUD", 0, values)[2]
  crc = (perodua_acc_checksum(0x273, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("ACC_CMD_HUD", 0, values)

