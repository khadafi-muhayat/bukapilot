#!/usr/bin/env python3
from cereal import car
from selfdrive.swaglog import cloudlog
from selfdrive.config import Conversions as CV
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.car.proton.values import CAR

class CarInterface(CarInterfaceBase):

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=None):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)
    ret.carName = "proton"
    ret.safetyModel = car.CarParams.SafetyModel.proton

    # proton port is a community feature
    ret.communityFeature = True
    ret.radarOffCan = True

    ret.steerRateCost = 0.7                # Lateral MPC cost on steering rate, higher value = sharper turn
    ret.steerLimitTimer = 0.4              # time before steerLimitAlert is issued
    ret.steerControlType = car.CarParams.SteerControlType.torque # or car.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.4           # Steering wheel actuator delay in seconds, it was 0.1

    # Tire stiffness factor fictitiously lower if it includes the steering column torsion effect.
    # For modeling details, see p.198-200 in "The Science of Vehicle Dynamics (2014), M. Guiggiani"
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 20, 30], [0., 20, 30]]
    ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.06, 0.07, 0.12], [0.25, 0.26, 0.30]]
    ret.lateralTuning.pid.kf = 0.000166

    ret.gasMaxBP = [0., 9., 35]
    ret.gasMaxV = [0.4, 0.5, 1.0]
    ret.longitudinalTuning.kpV = [1.4, 0.9, 0.9]
    ret.startAccel = 1                     # Required acceleraton to overcome creep braking

    # common interfaces
    stop_and_go = False
    ret.transmissionType = car.CarParams.TransmissionType.automatic
    ret.enableApgs = False                 # advanced parking guidance system
    ret.safetyParam = 1
    ret.enableGasInterceptor = True
    ret.openpilotLongitudinalControl = True

    if candidate == CAR.PROTON_SAGA:
      ret.wheelbase = 2.465                # meter
      ret.steerRatio = 16.54
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.8371
      ret.mass = 1080. + STD_CARGO_KG

    elif candidate == CAR.PROTON_IRIZ:
      ret.wheelbase = 2.5
      ret.steerRatio = 16.54
      ret.centerToFront = ret.wheelbase * 0.44
      tire_stiffness_factor = 0.6371
      ret.mass = 1015. + STD_CARGO_KG

    elif candidate == CAR.PROTON_PERSONA:
      ret.wheelbase = 2.455
      ret.steerRatio = 16.54
      ret.centerToFront = ret.wheelbase * 0.55
      tire_stiffness_factor = 0.6371
      ret.mass = 940. + STD_CARGO_KG

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

    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl to be called at 100hz
  def apply(self, c):

    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                               c.hudControl.visualAlert, c.cruiseControl.cancel)

    self.frame += 1
    return can_sends
