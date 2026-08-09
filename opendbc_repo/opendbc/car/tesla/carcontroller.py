import math
import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, apply_std_steer_angle_limits, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.tesla.ars408_can import ARS408CAN
from opendbc.car.tesla.teslacan import TeslaCAN
from opendbc.car.tesla.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_angle_last = 0
    self.packer = CANPacker(dbc_names[Bus.party])
    self.tesla_can = TeslaCAN(self.packer)
    self.ars408_can = ARS408CAN()

  def _radar_motion(self, CS):
    reverse = CS.out.gearShifter == structs.CarState.GearShifter.reverse
    standstill = CS.out.standstill or abs(float(CS.out.vEgo)) < 0.05
    direction = 0 if standstill else (2 if reverse else 1)
    signed_speed = -abs(float(CS.out.vEgo)) if reverse else abs(float(CS.out.vEgo))

    steer_ratio = max(float(self.CP.steerRatio), 1.0)
    wheelbase = max(float(self.CP.wheelbase), 0.1)
    road_wheel_angle = math.radians(float(CS.out.steeringAngleDeg) / steer_ratio)
    yaw_rate_deg_s = math.degrees(signed_speed * math.tan(road_wheel_angle) / wheelbase)
    if standstill:
      yaw_rate_deg_s = 0.0
    return direction, yaw_rate_deg_s

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Bus 1 is dedicated to the ARS408. RadarCfg and FilterCfg remain
    # commissioning-only, while motion inputs are refreshed at 20 Hz. The
    # sensor times these inputs out after 500 ms and otherwise assumes a
    # stationary platform, which degrades object dynamics used for lead control.
    if not self.CP.radarUnavailable and self.frame % 5 == 0:
      direction, yaw_rate_deg_s = self._radar_motion(CS)
      can_sends.append(self.ars408_can.create_speed_information(CS.out.vEgo, direction))
      can_sends.append(self.ars408_can.create_yaw_rate_information(yaw_rate_deg_s))

    # Disengage and allow for user override on high torque inputs
    # TODO: move this to a generic disengageRequested carState field and set CC.cruiseControl.cancel based on it
    hands_on_fault = CS.hands_on_level >= 3
    cruise_cancel = CC.cruiseControl.cancel or hands_on_fault
    lat_active = CC.latActive and not hands_on_fault

    if self.frame % 2 == 0:
      # Angular rate limit based on speed
      self.apply_angle_last = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgo,
                                                           CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

      can_sends.append(self.tesla_can.create_steering_control(self.apply_angle_last, lat_active, (self.frame // 2) % 16))

    if self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_steering_allowed((self.frame // 10) % 16))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      if self.frame % 4 == 0:
        state = 13 if cruise_cancel else 4  # 4=ACC_ON, 13=ACC_CANCEL_GENERIC_SILENT
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        cntr = (self.frame // 4) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(state, accel, cntr, CS.out.vEgo, CC.longActive))

    else:
      # Increment counter so cancel is prioritized even without openpilot longitudinal
      if cruise_cancel:
        cntr = (CS.das_control["DAS_controlCounter"] + 1) % 8
        can_sends.append(self.tesla_can.create_longitudinal_command(13, 0, cntr, CS.out.vEgo, False))

    # TODO: HUD control
    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
