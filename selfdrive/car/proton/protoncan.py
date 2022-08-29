from common.numpy_fast import clip, interp
from cereal import car
from selfdrive.config import Conversions as CV

SetDistance = car.CarState.CruiseState.SetDistance

<<<<<<< HEAD
# reference: http://sunshine2k.de/articles/coding/crc/understanding_crc.html#ch3
crc8_lut_8h2f = []

def init_lut_crc8_8h2f():
  poly = 0x2F

  for i in range(256):
    crc = i
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
    crc8_lut_8h2f.append(crc)

def get_crc8_8h2f(dat):
  crc = 0xFF    # initial value for crc8_8h2f
  for i in range(len(dat)):
    crc ^= dat[i]
    crc = crc8_lut_8h2f[crc]

  return crc ^ 0xFF

=======
>>>>>>> snapshot
def compute_set_distance(state):
  if state == SetDistance.aggresive:
    return 2
  elif state == SetDistance.normal:
    return 1
  else:
    return 0

<<<<<<< HEAD
=======
def proton_checksum(addr,dat):
  return ( addr + len(dat) + 1 + 2 + sum(dat)) & 0xFF

>>>>>>> snapshot
def create_can_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""

  values = {
<<<<<<< HEAD
    "LKAS_ENGAGED1": steer_req,
    "LKAS_ENGAGED2": steer_req,
    "STEER_CMD": steer if steer_req else 0,
    "STEER_DIR": 1 if steer_req > 0 else 0,
    "COUNTER": raw_cnt,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "SET_ME_1_3": 1,
    "SET_ME_48": 0x48,
    "HAND_ON_WHEEL": 0,
    "WHEEL_WARNING_CHIME": 1,
  }

  dat = packer.make_can_msg("ADAS_LKAS", 0, values)[2]
  crc = get_crc8_8h2f(dat[:-1])
  values["CHECKSUM"] = crc

  return packer.make_can_msg("STEERING_LKAS", 0, values)

init_lut_crc8_8h2f()
=======
    "STEER_REQ": steer_req,
    "STEER_CMD": -steer if steer_req else 0,
    "COUNTER": raw_cnt,
    "SET_ME_1": 1,
    "SET_ME_1_2": 1,
  }

  dat = packer.make_can_msg("STEERING_LKAS", 0, values)[2]
  crc = proton_checksum(0x1d0, dat[:-1])
  values["CHECKSUM"] = crc


  return packer.make_can_msg("STEERING_LKAS", 0, values)

def create_hud(packer, lkas_rdy, enabled, llane_visible, rlane_visible, ldw, fcw, aeb, front_depart):

  values = {
    "LKAS_SET": lkas_rdy,
    "LKAS_ENGAGED": enabled,
    "LDA_ALERT": ldw,
    "LANE_RIGHT_DETECT": rlane_visible,
    "LANE_LEFT_DETECT": llane_visible,
    "SET_ME_X02": 0x2,
    "AEB_ALARM": fcw,
    "AEB_BRAKE": aeb,
    "FRONT_DEPART": front_depart,
  }

  dat = packer.make_can_msg("LKAS_HUD", 0, values)[2]
  crc = (proton_checksum(0x274, dat[:-1]))
  values["CHECKSUM"] = crc

  return packer.make_can_msg("LKAS_HUD", 0, values)


>>>>>>> snapshot
