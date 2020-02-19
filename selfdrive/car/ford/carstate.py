from opendbc.can.parser import CANParser
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.ford.values import DBC

WHEEL_RADIUS = 0.33

def get_can_parser(CP):

  signals = [
    # sig_name, sig_address, default
    ("WhlRr_W_Meas", "WheelSpeed_CG1", 0.),
    ("WhlRl_W_Meas", "WheelSpeed_CG1", 0.),
    ("WhlFr_W_Meas", "WheelSpeed_CG1", 0.),
    ("WhlFl_W_Meas", "WheelSpeed_CG1", 0.),
    ("SteWhlRelInit_An_Sns", "Steering_Wheel_Data_CG1", 0.),
    ("Cruise_State", "Cruise_Status", 0.),
    ("Set_Speed", "Cruise_Status", 0.),
    ("LaActAvail_D_Actl", "Lane_Keep_Assist_Status", 0),
    ("LaHandsOff_B_Actl", "Lane_Keep_Assist_Status", 0),
    ("LaActDeny_B_Actl", "Lane_Keep_Assist_Status", 0),
    ("ApedPosScal_Pc_Actl", "EngineData_14", 0.),
    ("Dist_Incr", "Steering_Buttons", 0.),
    ("Brake_Drv_Appl", "Cruise_Status", 0.),
    ("Brake_Lights", "BCM_to_HS_Body", 0.),
  ]

  checks = [
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)


class CarState(CarStateBase):
  def update(self, cp):
    # update prevs, update must run once per loop
    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    # calc best v_ego estimate, by averaging two opposite corners
    self.v_wheel_fl = cp.vl["WheelSpeed_CG1"]['WhlRr_W_Meas'] * WHEEL_RADIUS
    self.v_wheel_fr = cp.vl["WheelSpeed_CG1"]['WhlRl_W_Meas'] * WHEEL_RADIUS
    self.v_wheel_rl = cp.vl["WheelSpeed_CG1"]['WhlFr_W_Meas'] * WHEEL_RADIUS
    self.v_wheel_rr = cp.vl["WheelSpeed_CG1"]['WhlFl_W_Meas'] * WHEEL_RADIUS
    self.v_ego_raw = mean([self.v_wheel_fl, self.v_wheel_fr, self.v_wheel_rl, self.v_wheel_rr])
    self.v_ego, self.a_ego = self.update_speed_kf(self.v_ego_raw)
    self.standstill = not self.v_ego_raw > 0.001

    self.angle_steers = cp.vl["Steering_Wheel_Data_CG1"]['SteWhlRelInit_An_Sns']
    self.v_cruise_pcm = cp.vl["Cruise_Status"]['Set_Speed'] * CV.MPH_TO_MS
    self.pcm_acc_status = cp.vl["Cruise_Status"]['Cruise_State']
    self.main_on = cp.vl["Cruise_Status"]['Cruise_State'] != 0
    self.lkas_state = cp.vl["Lane_Keep_Assist_Status"]['LaActAvail_D_Actl']
    # TODO: we also need raw driver torque, needed for Assisted Lane Change
    self.steer_override = not cp.vl["Lane_Keep_Assist_Status"]['LaHandsOff_B_Actl']
    self.steer_error = cp.vl["Lane_Keep_Assist_Status"]['LaActDeny_B_Actl']
    self.user_gas = cp.vl["EngineData_14"]['ApedPosScal_Pc_Actl']
    self.brake_pressed = bool(cp.vl["Cruise_Status"]["Brake_Drv_Appl"])
    self.brake_lights = bool(cp.vl["BCM_to_HS_Body"]["Brake_Lights"])
    self.generic_toggle = bool(cp.vl["Steering_Buttons"]["Dist_Incr"])
