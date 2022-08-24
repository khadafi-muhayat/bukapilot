import copy
from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
# from common.conversions import Conversions as CV
from selfdrive.config import Conversions as CV

from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.car.wuling.values import DBC, CanBus, HUD_MULTIPLIER, STEER_THRESHOLD, CAR, PREGLOBAL_CARS 
from time import time

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
   
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL"]["TRANSMISSION_STATE"]

    self.lka_steering_cmd_counter = 0
    self.engineRPM = 0
    
    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.low_speed_alert = False
    self.lkas_allowed_speed = False
    self.lkas_disabled = False
    self.cruise_speed = 30 * CV.KPH_TO_MS
    self.cruise_speed_counter = 0
    self.is_cruise_latch = False
    self.rising_edge_since = 0
    self.last_frame = time() # todo: existing infra to reuse?
    self.dt = 0

  def update(self, pt_cp, cp_cam, loopback_cp):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons

    self.engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM'] * 0.25

    ret.wheelSpeeds = self.get_wheel_speeds(
      pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"],
      pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    
    # ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["RightSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2
    
    # ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
    #             pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
    #             pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
    #             pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)
    
    ret.brakePressed = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0
    # ret.brake = pt_cp.vl["ECMEngineStatus"]["Brake_Pressed"] != 0
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]["TRANSMISSION_STATE"], None))

    ret.vEgoCluster = ret.vEgoRaw * HUD_MULTIPLIER

    self.park_brake = pt_cp.vl["EPBStatus"]["EPBSTATUS"]
    self.pcm_acc_status = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSTATE"]
    # dp - brake lights
    # ret.brakeLights = ret.brakePressed
    
    ret.cruiseState.enabled = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSTATE"] != 0
    # ret.cruiseState.enabled = True
    # ret.cruiseActualEnabled = ret.cruiseState.enabled
    ret.cruiseState.available = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSTATE"] != 0
    # ret.cruiseState.available = True
    ret.cruiseState.speed = pt_cp.vl["ASCMActiveCruiseControlStatus"]["ACCSpeedSetpoint"] * CV.KPH_TO_MS
    
    ret.steeringTorque = pt_cp.vl["PSCMSteeringAngle"]["SteeringTorque"]

    print('Cruise speed :  %d' % ret.cruiseState.speed)

    #trans state 15 "PARKING" 1 "DRIVE" 14 "BACKWARD" 13 "NORMAL"
    
    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      
      ("TurnSignals", "BCMTurnSignals"),
      ("SteeringWheelAngle", "PSCMSteeringAngle"),
      ("SteeringWheelRate", "PSCMSteeringAngle"),
      ("SteeringTorque", "PSCMSteeringAngle"),
      
      ("FLWheelSpd", "EBCMWheelSpdFront"),
      ("FRWheelSpd", "EBCMWheelSpdFront"),
      ("RLWheelSpd", "EBCMWheelSpdRear"),
      ("RRWheelSpd", "EBCMWheelSpdRear"),
      ("EngineRPM", "ECMEngineStatus"),
      
      ("FrontLeftDoor", "BCMDoorBeltStatus"),
      ("FrontRightDoor", "BCMDoorBeltStatus"),
      ("RearLeftDoor", "BCMDoorBeltStatus"),
      ("RearRightDoor", "BCMDoorBeltStatus"),
      ("LeftSeatBelt", "BCMDoorBeltStatus"),
      ("RightSeatBelt", "BCMDoorBeltStatus"),
      ("EPBClosed", "EPBStatus"),
      ("CruiseMainOn", "ECMEngineStatus"),
      ("Brake_Pressed", "ECMEngineStatus"),
      ("EPBSTATUS", "EPBStatus"),
      ("ACCBUTTON", "ASCMActiveCruiseControlStatus"),
      ("ACCSTATE", "ASCMActiveCruiseControlStatus"),
      ("ACCSpeedSetpoint", "ASCMActiveCruiseControlStatus"),
      ("TRANSMISSION_STATE", "ECMPRDNL"),
    ]

    checks = [
      ("BCMTurnSignals", 1),
      ("ECMEngineStatus", 10),
      ("EPBStatus", 10),
      ("ECMPRDNL", 10),
      ("BCMDoorBeltStatus", 10),
      ("EBCMWheelSpdFront", 20),
      ("EBCMWheelSpdRear", 20),
      ("ASCMActiveCruiseControlStatus", 20),
      ("PSCMSteeringAngle", 100),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.POWERTRAIN)

  @staticmethod
  def get_loopback_can_parser(CP):
    signals = [
      ("RollingCounter", "PSCMSteeringAngle"),
    ]

    checks = [
      ("PSCMSteeringAngle", 10),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CanBus.LOOPBACK)
