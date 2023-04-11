#!/usr/bin/env python3
from cereal import car
# from common.conversions import Conversions as CV

from selfdrive.car.wuling.values import CAR, CruiseButtons,AccState
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, get_safety_config
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from selfdrive.config import Conversions as CV
from common.op_params import opParams

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType
NetworkLocation = car.CarParams.NetworkLocation
BUTTONS_DICT = {CruiseButtons.RES_ACCEL: ButtonType.accelCruise, CruiseButtons.DECEL_SET: ButtonType.decelCruise,
                CruiseButtons.MAIN: ButtonType.altButton3, CruiseButtons.CANCEL: ButtonType.cancel}
class CarInterface(CarInterfaceBase):

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=None, disable_radar=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    op_params = opParams("wuling car_interface.py for lateral override")

    ret.carName = "wuling"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.wuling)]
    # ret.safetyConfigs[0].safetyParam = 2
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.radarOffCan = True
    ret.lateralTuning.init('pid')
    ret.pcmCruise = False
    ret.enableApgs = False 
    ret.enableDsu = False
    
    # ret.dashcamOnly = False
    # ret.dashcamOnly = candidate not in (CAR.CX5_2022, CAR.CX9_2021)
    ret.openpilotLongitudinalControl = True

    ret.steerRateCost = 0.7
    ret.steerLimitTimer = 0.1
    ret.mass = 1900. + STD_CARGO_KG
    ret.wheelbase = 2.75
    ret.centerToFront = ret.wheelbase * 0.4
    ret.steerRatio = op_params.get('steer_ratio', force_update=True)
    
    # -----
    ret.steerActuatorDelay = 0.1   # end-to-end angle controller
    # ret.lateralTuning.pid.kf = 0.00003
    # ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[40, 70], [300, 830]]
    
    # tuning wuling PID
    ret.lateralTuning.pid.kf = 0.00003
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 0.], [0., 0.]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0., 0.], [0., 0.]]

    # Live tune
    bp = [i * CV.MPH_TO_MS for i in op_params.get("TUNE_LAT_PID_bp_mph", force_update=True)]
    kpV = [i for i in op_params.get("TUNE_LAT_PID_kp", force_update=True)]
    kiV = [i for i in op_params.get("TUNE_LAT_PID_ki", force_update=True)]
    ret.lateralTuning.pid.kpV = kpV
    ret.lateralTuning.pid.kiV = kiV
    ret.lateralTuning.pid.kpBP = bp
    ret.lateralTuning.pid.kiBP = bp
    ret.lateralTuning.pid.kf = op_params.get('TUNE_LAT_PID_kf', force_update=True)
    
    # ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    # ret.longitudinalTuning.kpV = [0.9, 0.8, 0.8]
    
    # ret.longitudinalTuning.kpV = [0.8, 0.9, 1.0]
    # ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.08], [0.32]]
    # ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[10, 26], [200, 430]]

    # ----

    # tuning
    # ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    # ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2], [0.00]]
    # ret.lateralTuning.pid.kf = 0.00004   # full torque for 20 deg at 80mph means 0.00007818594
    # ret.steerRateCost = 1.0
    # ret.steerActuatorDelay = 0.1  # Default delay, not measured yet
    
    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)


    return ret

  # returns a car.CarState
  def updateollllll (self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_loopback.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)

    
    # dp
    # ret.cruiseState.enabled = common_interface_atl(ret, dragonconf.dpAtl)
    ret.canValid = self.cp.can_valid and self.cp_loopback.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

   # events
    events = self.create_common_events(ret)

    # if self.CS.lkas_disabled:
    #   events.add(EventName.lkasDisabled)
    # elif self.CS.low_speed_alert:
    #   events.add(EventName.belowSteerSpeed)
    
    if ret.vEgo < self.CP.minEnableSpeed:
      events.add(EventName.belowEngageSpeed)
    if self.CS.park_brake:
      events.add(EventName.parkBrake)
    if ret.cruiseState.standstill:
      events.add(EventName.resumeRequired)
    if self.CS.pcm_acc_status == AccState.FAULTED:
      events.add(EventName.accFaulted)
    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    # handle button presses
    for b in ret.buttonEvents:
      # do enable on both accel and decel buttons
      if b.type in (ButtonType.accelCruise, ButtonType.decelCruise) and not b.pressed:
        events.add(EventName.buttonEnable)
      # do disable on button down
      if b.type == ButtonType.cancel and b.pressed:
        events.add(EventName.buttonCancel)


    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  # returns a car.CarState
  def update(self, c, can_strings):
    # to receive CAN Messages
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam, self.cp_loopback)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # if self.CS.cruise_buttons != self.CS.prev_cruise_buttons and self.CS.prev_cruise_buttons != CruiseButtons.INIT:
    #   be = create_button_event(self.CS.cruise_buttons, self.CS.prev_cruise_buttons, BUTTONS_DICT, CruiseButtons.UNPRESS)

    #   # Suppress resume button if we're resuming from stop so we don't adjust speed.
    #   if be.type == ButtonType.accelCruise and (ret.cruiseState.enabled and ret.standstill):
    #     be.type = ButtonType.unknown

    #   ret.buttonEvents = [be]

    events = self.create_common_events(ret)
    
    
     # create events for auto lane change below allowable speed
    if ret.vEgo < LANE_CHANGE_SPEED_MIN and (ret.leftBlinker or ret.rightBlinker):
      events.add(EventName.belowLaneChangeSpeed)

    if ret.vEgo < self.CP.minEnableSpeed:
      events.add(EventName.belowEngageSpeed)
    if ret.cruiseState.standstill:
      events.add(EventName.resumeRequired)
    if ret.vEgo < self.CP.minSteerSpeed:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    # handle button presses
    # events.events.extend(create_button_enable_events(ret.buttonEvents, pcm_cruise=self.CP.pcmCruise))

    ret.events = events.to_msg()
    # print("Events : %s" % events.to_msg())

    return ret
  
  def apply(self, c):
    hud_control = c.hudControl
    isLdw = c.hudControl.leftLaneDepart or c.hudControl.rightLaneDepart

    ret = self.CC.update(c, c.enabled, self.CS, self.frame, c.actuators,
                         c.cruiseControl.cancel, 
                         hud_control.visualAlert,
                         hud_control.setSpeed,
                         hud_control.leftLaneVisible, 
                         hud_control.rightLaneVisible, 
                         hud_control.leftLaneDepart, 
                         hud_control.rightLaneDepart)
    self.frame += 1
    return ret
