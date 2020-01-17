from collections import namedtuple
from cereal import car
from common.realtime import DT_CTRL
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.controls.lib.drive_helpers import rate_limit
from common.numpy_fast import clip
from selfdrive.car import create_gas_command
from selfdrive.car.honda import hondacan
from selfdrive.car.honda.values import CruiseButtons, CruiseSettings, CAR, VISUAL_HUD
from opendbc.can.packer import CANPacker
from selfdrive.kegman_conf import kegman_conf

kegman = kegman_conf()
VisualAlert = car.CarControl.HUDControl.VisualAlert

def actuator_hystereses(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params
  brake_hyst_on = 0.02     # to activate brakes exceed this value
  brake_hyst_off = 0.005                     # to deactivate brakes below this value
  brake_hyst_gap = 0.01                      # don't change brake command for small oscillations within this value

  #*** hysteresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  if (car_fingerprint in (CAR.ACURA_ILX, CAR.CRV)) and brake > 0.0:
    brake += 0.15

  return brake, braking, brake_steady


def brake_pump_hysteresis(apply_brake, apply_brake_last, last_pump_ts, ts):
  pump_on = False

  # reset pump timer if:
  # - there is an increment in brake request
  # - we are applying steady state brakes and we haven't been running the pump
  #   for more than 20s (to prevent pressure bleeding)
  if apply_brake > apply_brake_last or (ts - last_pump_ts > 20. and apply_brake > 0):
    last_pump_ts = ts

  # once the pump is on, run it for at least 0.2s
  if ts - last_pump_ts < 0.2 and apply_brake > 0:
    pump_on = True

  return pump_on, last_pump_ts


def process_hud_alert(hud_alert):
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0

  # priority is: FCW, steer required, all others
  if hud_alert == VisualAlert.fcw:
    fcw_display = VISUAL_HUD[hud_alert.raw]
  elif hud_alert == VisualAlert.steerRequired:
    steer_required = VISUAL_HUD[hud_alert.raw]
  else:
    acc_alert = VISUAL_HUD[hud_alert.raw]

  return fcw_display, steer_required, acc_alert


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "car",
                      "lanes", "fcw", "acc_alert", "steer_required", "dist_lines", "dashed_lanes"])

class CarController():
  def __init__(self, dbc_name, CP):
    self.apply_steer_last = 0
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.packer = CANPacker(dbc_name)
    self.new_radar_config = False
    self.prev_lead_distance = 0.0 # a non-linear value
    self.stopped_lead_distance = 0.0
    self.lead_distance_counter = 1.0 # seconds since last update
    self.lead_distance_counter_prev = 1
    self.rough_lead_speed = 0.0 #delta m/s
    self.desiredTR = 0 # the desired distance bar
    self.eps_modified = False
    for fw in CP.carFw:
      if fw.ecu == "eps" and b"," in fw.fwVersion:
        print("EPS FW MODIFIED!")
        self.eps_modified = True

  def honda_bosch_to_meters(self, lead_distance):
      if lead_distance < 100:
          return (0.1 * lead_distance) * 0.9144
      elif lead_distance < 160:
          t = (lead_distance - 100) / 60.0
          return ((35 * t ** 2) + (10 * (1 - t) ** 2) + (28 * t * (1 - t))) * 0.9144
      else:
          return (lead_distance - 125) * 0.9144

  # lead_distance is non linear
  # Must be called every frame and assumes 100hz (frames per second)
  def rough_speed(self, lead_distance):
    #If we got an updated lead distance calculate the closing rate
    if self.prev_lead_distance != lead_distance:
      #delta distance is negative when approaching
      delta_distance = self.honda_bosch_to_meters(lead_distance) - self.honda_bosch_to_meters(self.prev_lead_distance)
      #delta_speed is distance / time (seconds when called at 100hz)
      delta_speed = delta_distance / self.lead_distance_counter
      #set the rough lead speed by feathering in the updated values
      self.rough_lead_speed = 0.5 * delta_speed + 0.5 * self.rough_lead_speed
      #reset lead distance counter
      self.lead_distance_counter = 0.0
      #update previous lead distance
      self.prev_lead_distance = lead_distance
    #If it has been a while since the last lead distance update, assume delta speed is zero
    elif self.lead_distance_counter >= 2.0:
      #reduce the lead speed to zero
      self.rough_lead_speed = 0
      #set distance counter to above zero to avoid high initial values
      self.lead_distance_counter = 1.0
    #increase counter by 0.01 (1/100 of a second)
    self.lead_distance_counter += 0.01
    print("{0} m/s".format(self.rough_lead_speed))

  # relative distance is in ft from output
  # v_ego is current car speed
  # stopped is a CS telling whether vehicle is in stopped state or not
  def get_TR(self, lead_distance, v_ego, stopped):
    # Radar picks up at about 190ft to 200ft
    # Testing out a bunch of random numbers..
    # If car and lead car is moving set to 1 always and let different speed rules take over..
    if (v_ego >= 1) and (self.rough_lead_speed >= 1):
      self.desiredTR = 1
    # If in slower speeds >33.55mph
    elif (v_ego >= 15) and (self.rough_lead_speed > 0.1):
      self.desiredTR = 1
      # car is slowing down
      if (v_ego < 15) and (self.rough_lead_speed <= 16):
        self.desiredTR = 3
      elif (v_ego < 10) and (self.rough_lead_speed < 12):
        self.desiredTR = 2
      elif (v_ego < 7) and (self.rough_lead_speed < 8):
        self.desiredTR = 1
    # If caught some traction >45mph, lead up closer to moving lead car.
    elif (v_ego >= 20) and (self.rough_lead_speed > 0.1):
      self.desiredTR = 1
      if (v_ego < 20) and (self.rough_lead_speed <= 16):
        self.desiredTR = 4
      elif (v_ego < 17) and (self.rough_lead_speed <= 12):
        self.desiredTR = 3
      elif (v_ego < 14) and (self.rough_lead_speed <= 8):
        self.desiredTR = 2
      elif (v_ego < 7) and (self.rough_lead_speed < 4):
        self.desiredTR = 1
    # if detects car between 180ft and 254ft and if car is going >55mph and rough lead_speed is less than 20 ft/s
    if (lead_distance < 255 and lead_distance >= 180) and (v_ego >= 25) and (self.rough_lead_speed <= 20):
        self.desiredTR = 3
    elif (lead_distance < 180) and (v_ego < 25) and (self.rough_lead_speed <= 12):
        self.desiredTR = 2
    # No lead car found
    if lead_distance == 255:
        self.desiredTR = 1
    # Reset to 1 if car is stopped in front of car
    if stopped:
        self.desiredTR = 1

    return self.desiredTR

  def update(self, enabled, CS, frame, actuators, \
             pcm_speed, pcm_override, pcm_cancel_cmd, pcm_accel, \
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):
    # *** apply brake hysteresis ***
    brake, self.braking, self.brake_steady = actuator_hystereses(actuators.brake, self.braking, self.brake_steady, CS.v_ego, CS.CP.carFingerprint)

    # *** no output if not enabled ***
    if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = True

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(brake, self.brake_last, -2., DT_CTRL)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    if hud_show_lanes and CS.lkMode: #and not CS.left_blinker_on and not CS.right_blinker_on:
      hud_lanes = 1
    else:
      hud_lanes = 0

    if enabled:
      if hud_show_car:
        hud_car = 2
      else:
        hud_car = 1
    else:
      hud_car = 0

    fcw_display, steer_required, acc_alert = process_hud_alert(hud_alert)

    hud = HUDData(int(pcm_accel), int(round(hud_v_cruise)), 1, hud_car,
                  0xc1, hud_lanes, fcw_display, acc_alert, steer_required, CS.read_distance_lines, CS.lkMode)

    # **** process the car messages ****

    # *** compute control surfaces ***
    BRAKE_MAX = 1024//4
    if CS.CP.carFingerprint in (CAR.ACURA_ILX):
      STEER_MAX = 0xF00
    elif CS.CP.carFingerprint in (CAR.CRV, CAR.ACURA_RDX):
      STEER_MAX = 0x3e8  # CR-V only uses 12-bits and requires a lower value
    elif CS.CP.carFingerprint in (CAR.ODYSSEY_CHN):
      STEER_MAX = 0x7FFF
    elif CS.CP.carFingerprint in (CAR.CIVIC) and self.eps_modified:
      STEER_MAX = 0x1400
    else:
      STEER_MAX = 0x1000

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_gas = clip(actuators.gas, 0., 1.)
    apply_brake = int(clip(self.brake_last * BRAKE_MAX, 0, BRAKE_MAX - 1))
    apply_steer = int(clip(-actuators.steer * STEER_MAX, -STEER_MAX, STEER_MAX))
    self.apply_steer_last = apply_steer

    if CS.CP.carFingerprint in (CAR.CIVIC) and self.eps_modified:
      if apply_steer > 0xA00:
        apply_steer = (apply_steer - 0xA00) / 2 + 0xA00
      elif apply_steer < -0xA00:
        apply_steer = (apply_steer + 0xA00) / 2 - 0xA00

    lkas_active = enabled and not CS.steer_not_allowed and CS.lkMode

    # Send CAN commands.
    can_sends = []

    # Send steering command.
    idx = frame % 4
    can_sends.append(hondacan.create_steering_control(self.packer, apply_steer,
      lkas_active, CS.CP.carFingerprint, idx, CS.CP.isPandaBlack))

    # Send dashboard UI commands.
    if (frame % 10) == 0:
      idx = (frame//10) % 4
      can_sends.extend(hondacan.create_ui_commands(self.packer, pcm_speed, hud, CS.CP.carFingerprint, CS.is_metric, idx, CS.CP.isPandaBlack, CS.stock_hud))

    #if CS.CP.carFingerprint in (CAR.INSIGHT):
    #  self.rough_speed(CS.lead_distance)
    #  if kegman.conf['simpledd'] == "1":
    #    #Get the desiredTR before using it.
    #    self.get_TR(CS.lead_distance, CS.v_ego, CS.stopped)
    #    # update to CS so we can push it to ui through cereal
    #    CS.desiredTR = self.desiredTR
    #    if frame % 13 < 2 and CS.hud_distance != (self.desiredTR % 4):
    #      # press distance bar button
    #      can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RESET, CruiseSettings.LEAD_DISTANCE, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
    #      # always set cruise setting to 0 after button press
    #      if frame % 25 < 5:
    #        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RESET, CruiseSettings.RESET, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))


    if CS.CP.radarOffCan:
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.CANCEL, CruiseSettings.RESET, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
      elif CS.stopped:
        if CS.CP.carFingerprint in (CAR.ACCORD, CAR.ACCORD_15, CAR.ACCORDH, CAR.INSIGHT):
          if CS.lead_distance > (self.prev_lead_distance + float(kegman.conf['leadDistance'])):
            can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, CruiseSettings.RESET, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
        elif CS.CP.carFingerprint in (CAR.CIVIC_BOSCH):
          if CS.hud_lead == 1:
            can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, CruiseSettings.RESET, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
        else:
          can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, CruiseSettings.RESET, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
      else:
        self.prev_lead_distance = CS.lead_distance

    else:
      # Send gas and brake commands.
      if (frame % 2) == 0:
        idx = frame // 2
        ts = frame * DT_CTRL
        pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)
        can_sends.append(hondacan.create_brake_command(self.packer, apply_brake, pump_on,
          pcm_override, pcm_cancel_cmd, hud.fcw, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack, CS.stock_brake))
        self.apply_brake_last = apply_brake

        if CS.CP.enableGasInterceptor:
          # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
          # This prevents unexpected pedal range rescaling
          can_sends.append(create_gas_command(self.packer, apply_gas, idx))

    return can_sends
