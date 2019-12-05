from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.controls.lane_hugging import LaneHugging
from cereal import car
from cereal import log
from selfdrive.kegman_conf import kegman_conf


class LatControlPID():
  def __init__(self, CP):
    self.kegman = kegman_conf(CP)
    self.deadzone = float(self.kegman.conf['deadzone'])
    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0)
    self.angle_steers_des = 0.
    self.lane_hugging = LaneHugging()
    self.mpc_frame = 0

  def reset(self):
    self.pid.reset()
    
  def live_tune(self, CP):
    self.mpc_frame += 1
    if self.mpc_frame % 300 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      self.kegman = kegman_conf()
      if self.kegman.conf['tuneGernby'] == "1":
        self.steerKpV = [float(self.kegman.conf['Kp'])]
        self.steerKiV = [float(self.kegman.conf['Ki'])]
        self.steerKf = float(self.kegman.conf['Kf'])
        self.pid = PIController((CP.lateralTuning.pid.kpBP, self.steerKpV),
                            (CP.lateralTuning.pid.kiBP, self.steerKiV),
                            k_f=self.steerKf, pos_limit=1.0)
        self.deadzone = float(self.kegman.conf['deadzone'])
        
      self.mpc_frame = 0    

      
  def update(self, active, v_ego, angle_steers, angle_steers_rate, eps_torque, steer_override, CP, path_plan):
    kegman = kegman_conf()
    #self.live_tune(CP)
    
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steerAngle = float(angle_steers)
    pid_log.steerRate = float(angle_steers_rate)

    if v_ego < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
     # if kegman.conf['shane_mod'] == "1":
     #   self.angle_steers_des = self.lane_hugging.init(path_plan.angleSteers)  # get from MPC/PathPlanner and initialize lane hugging mod 
     # else:
      self.angle_steers_des = path_plan.angleSteers # get from MPC/PathPlanner

      steers_max = get_steer_max(CP, v_ego)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.angle_steers_des   # feedforward desired angle
      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # TODO: feedforward something based on path_plan.rateSteers
        steer_feedforward -= path_plan.angleOffset   # subtract the offset, since it does not contribute to resistive torque
        steer_feedforward *= v_ego**2  # proportional to realigning tire momentum (~ lateral accel)
      
      deadzone = self.deadzone    
        
      output_steer = self.pid.update(self.angle_steers_des, angle_steers, check_saturation=(v_ego > 10), override=steer_override,
                                     feedforward=steer_feedforward, speed=v_ego, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)

    self.sat_flag = self.pid.saturated
    return output_steer, float(self.angle_steers_des), pid_log
