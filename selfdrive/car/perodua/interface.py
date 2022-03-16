#!/usr/bin/env python3
from cereal import car
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.perodua.values import CAR

class CarInterface(CarInterfaceBase):

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "perodua"
    ret.safetyModel = car.CarParams.SafetyModel.perodua

    # perodua port is a community feature
    ret.communityFeature = True
    ret.radarOffCan = True

    ret.steerRateCost = 0.7                # Lateral MPC cost on steering rate, higher value = sharper turn
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerControlType = car.CarParams.SteerControlType.torque # or car.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.48           # Steering wheel actuator delay in seconds, it was 0.1

    # Tire stiffness factor fictitiously lower if it includes the steering column torsion effect.
    # For modeling details, see p.198-200 in "The Science of Vehicle Dynamics (2014), M. Guiggiani"
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.gasMaxBP = [0., 9., 35]
    ret.gasMaxV = [0.4, 0.4, 0.55]
    ret.longitudinalTuning.kpV = [0.9, 0.8, 0.8]
    ret.startAccel = 1                     # Required acceleraton to overcome creep braking

    # common interfaces
    stop_and_go = False
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.enableApgs = False                 # advanced parking guidance system
    ret.safetyParam = 1
    ret.enableGasInterceptor = 0x201 in fingerprint[0]
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.AXIA:
      ret.wheelbase = 2.455                # meter
      ret.steerRatio = 16.54
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.8371
      ret.mass = 850. + STD_CARGO_KG

      ret.lateralTuning.pid.kf = 0.0000917
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.12], [0.30]]
      ret.longitudinalTuning.kpV = [0.8, 0.9, 1.0]

    elif candidate == CAR.MYVI:
      ret.wheelbase = 2.5
      ret.steerRatio = 12.14
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.8371
      ret.mass = 1015. + STD_CARGO_KG

      ret.lateralTuning.pid.kf = 0.0000917
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.16], [0.30]]

    elif candidate == CAR.BEZZA:
      ret.wheelbase = 2.455
      ret.steerRatio = 16.54
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.8371
      ret.mass = 940. + STD_CARGO_KG

      ret.lateralTuning.pid.kf = 0.0000917
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.16], [0.41]]

    elif candidate == CAR.ARUZ:
      ret.wheelbase = 2.685
      ret.steerRatio = 16.54
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.6371
      ret.mass = 1310. + STD_CARGO_KG

      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.098], [0.135]]
      ret.longitudinalTuning.kpV = [1.6, 1.1, 1.1]

    elif candidate == CAR.ATIVA:
      # min speed to enable ACC. if car can do stop and go or has gas interceptor,
      # then set enabling speed to a negative value, so it won't matter.
      ret.minEnableSpeed = -1
      ret.wheelbase = 2.525
      ret.steerRatio = 14.54
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.6371
      ret.mass = 1035. + STD_CARGO_KG

      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.098], [0.135]]
      ret.longitudinalTuning.kpV = [1.2, 1.2, 1.2]
      ret.longitudinalTuning.kiV = [0.3, 0.3, 0.3]

      ret.stoppingBrakeRate = 0.2  # reach stopping target smoothly
      ret.startingBrakeRate = 3.0  # release brakes fast

    else:
      ret.dashcamOnly = True
      ret.safetyModel = car.CarParams.SafetyModel.noOutput

    ret.enableDsu = False

    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # to receive CAN Messages
    self.cp.update_strings(can_strings)

    ret = self.CS.update(self.cp)
    ret.yawRate = self.VM.yaw_rate(ret.steeringAngleDeg * CV.DEG_TO_RAD, ret.vEgo)
    ret.canValid = self.cp.can_valid
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # events
    events = self.create_common_events(ret)
    ret.events = events.to_msg()

    # Alex Comments: might need these if ativa have low engage limit. test first
    # if self.CS.low_speed_lockout and self.CP.openpilotLongitudinalControl:
    #   events.add(EventName.lowSpeedLockout)
    # if ret.vEgo < self.CP.minEnableSpeed and self.CP.openpilotLongitudinalControl:
    #   events.add(EventName.belowEngageSpeed)
    #   if c.actuators.gas > 0.1:
    #     # some margin on the actuator to not false trigger cancellation while stopping
    #     events.add(EventName.speedTooLow)
    #   if ret.vEgo < 0.001:
    #     # while in standstill, send a user alert
    #     events.add(EventName.manualRestart)

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c):

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                               c.hudControl.visualAlert, c.cruiseControl.cancel)

    self.frame += 1
    return can_sends
