from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.wuling import wulingcan
from selfdrive.car.wuling.values import DBC, CanBus, PREGLOBAL_CARS, CarControllerParams
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car import make_can_msg
from common.numpy_fast import interp

from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from cereal import car

VisualAlert = car.CarControl.HUDControl.VisualAlert

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.start_time = 0.
    self.frame = 0
    self.last_button_frame = 0
    
    # dp
    self.last_blinker_on = False
    self.blinker_end_frame = 0.
    self.enabled_last = False
    self.apply_steer_last = 0
    self.es_distance_cnt = -1
    self.es_lkas_cnt = -1
    self.cruise_button_prev = 0
    self.steer_rate_limited = False
    self.steer_alert_last = False
    self.lkas_action = 0
    
    self.apply_gas = 0
    self.apply_brake = 0

    self.lka_steering_cmd_counter_last = -1
    self.lka_icon_status_last = (False, False)

    print(CP.carFingerprint)
    print(DBC[CP.carFingerprint])
    
    self.params = CarControllerParams()

    self.p = CarControllerParams()
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])

  def update(self, CC, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert, hud_speed, 
             left_line, right_line, left_lane_depart, 
              right_lane_depart):

    P = self.p
    actuators = CC.actuators
    hud_control = CC.hudControl
    hud_alert = hud_control.visualAlert
    hud_v_cruise = hud_control.setSpeed
    
    can_sends = []
    steer_alert = visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lkas_active = True
    
    apply_steer = actuators.steer
    
    if CS.lka_steering_cmd_counter != self.lka_steering_cmd_counter_last:
      self.lka_steering_cmd_counter_last = CS.lka_steering_cmd_counter
    elif (frame % 4 == 0) :
      lkas_enabled = CC.active and not (CS.out.steerWarning or CS.out.steerError) and CS.out.vEgo > P.MIN_STEER_SPEED
      # lkas_enabled = True
      print('lkas_enabled :  %s' % lkas_enabled)
      if lkas_enabled:
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0

      # dp
      blinker_on = CS.out.leftBlinker or CS.out.rightBlinker
      # if not enabled:
      #   self.blinker_end_frame = 0
      # if self.last_blinker_on and not blinker_on:
      #   self.blinker_end_frame = frame + dragonconf.dpSignalOffDelay
      # apply_steer = common_controller_ctrl(enabled,
      #                                      dragonconf,
      #                                      blinker_on or frame < self.blinker_end_frame,
      #                                      apply_steer, CS.out.vEgo)
      self.last_blinker_on = blinker_on

      self.apply_steer_last = apply_steer
      # GM EPS faults on any gap in received message counters. To handle transient OP/Panda safety sync issues at the
      # moment of disengaging, increment the counter based on the last message known to pass Panda safety checks.
    
      idx = (CS.lka_steering_cmd_counter + 1) % 4
      # can_sends.append(wulingcan.create_steering_control(self.packer, apply_steer, idx, 1))

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        if not CC.active:
          # Stock ECU sends max regen when not enabled
          self.apply_gas = self.params.MAX_ACC_REGEN
          self.apply_brake = 0
          
        else:
          self.apply_gas = int(round(interp(actuators.accel, self.params.GAS_LOOKUP_BP, self.params.GAS_LOOKUP_V)))
          self.apply_brake = int(round(interp(actuators.accel, self.params.BRAKE_LOOKUP_BP, self.params.BRAKE_LOOKUP_V)))

          idx = (self.frame // 4) % 4

          at_full_stop = CC.longActive and CS.out.standstill
          near_stop = CC.longActive and (CS.out.vEgo < self.params.NEAR_STOP_BRAKE_PHASE)
          
          print('Apply gas %d' % self.apply_gas)
          print('Apply brake %d' % self.apply_brake)
          
          # GasRegenCmdActive needs to be 1 to avoid cruise faults. It describes the ACC state, not actuation
          # can_sends.append(wulingcan.create_gas_regen_command(self.packer_pt, CanBus.POWERTRAIN, self.apply_gas, idx, CC.enabled, at_full_stop))
          # can_sends.append(wulingcan.create_friction_brake_command(self.packer_ch, CanBus.CHASSIS, self.apply_brake, idx, near_stop, at_full_stop))

          # Send dashboard UI commands (ACC status)
          send_fcw = hud_alert == VisualAlert.fcw
          # can_sends.append(wulingcan.create_acc_dashboard_command(self.packer_pt, CanBus.POWERTRAIN, CC.enabled, hud_v_cruise * CV.MS_TO_KPH, hud_control.leadVisible, send_fcw))
    if (frame % 2) == 0:
      print('UI Command HUD Speed :  %s' % hud_speed)
      # can_sends.append(make_can_msg(0x373, b"\x82\x01\x00\x00\xac\x90\x02\xc1", 0))
      # can_sends.append(make_can_msg(0x260, b"\x0f\xe8\x00\x80\x00\x06\xfe\x7b", 0))
      # can_sends.append(make_can_msg(0x260, b"\xc0\x6f\x00\x80\x04\x10\xd8\x9b", 0))
      # can_sends.append(make_can_msg(0x128, b"\xcf\xe8\x00\x03\x00\x00\x00\xba", 0))
      # can_sends.append(make_can_msg(0x191, b"\x46\x8d\x06\x99\x06\x8e\x00\x06", 0))
      # can_sends.append(make_can_msg(0x269, b"\xe7\x06\x35\x50\x00\x11\x40\xc3", 0))
      # can_sends.append(make_can_msg(0x269, b"\x00\x00\x15\x50\x60\x00\x40\x05", 0))

      # can_sends.append(wulingcan.create_acc_dashboard_command(self.packer, CanBus.POWERTRAIN, enabled, hud_speed * CV.MS_TO_KPH, 0, 0))

    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / P.STEER_MAX
    new_actuators.gas = self.apply_gas
    new_actuators.brake = self.apply_brake
    
    print('Last enable :  %s' % enabled)
    
    if (enabled):
        print('enable adas')

    # if enabled and  (frame % 100) == 0:
      #  can_sends.append(make_can_msg(0x373, b"\xc6\x3d\x01\x00\xac\x90\x02\x42", 0))
      
    self.enabled_last = enabled
    self.main_on_last = CS.out.cruiseState.available
    self.steer_alert_last = steer_alert

    print('Steer :  %s' % apply_steer)

    return new_actuators, can_sends
