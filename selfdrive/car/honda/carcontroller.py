from collections import namedtuple
from common.realtime import DT_CTRL
from selfdrive.controls.lib.drive_helpers import rate_limit
from common.numpy_fast import clip
from selfdrive.car import create_gas_command
from selfdrive.car.honda import hondacan
from selfdrive.car.honda.values import AH, CruiseButtons, CAR, CruiseSettings
from selfdrive.can.packer import CANPacker
from selfdrive.kegman_conf import kegman_conf

kegman = kegman_conf()


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
  if hud_alert == AH.NONE:          # no alert
    pass
  elif hud_alert == AH.FCW:         # FCW
    fcw_display = hud_alert[1]
  elif hud_alert == AH.STEER:       # STEER
    steer_required = hud_alert[1]
  else:                             # any other ACC alert
    acc_alert = hud_alert[1]

  return fcw_display, steer_required, acc_alert


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "mini_car", "car", "X4",
                      "lanes", "beep", "chime", "fcw", "acc_alert", "steer_required", "dashed_lanes"])


class CarController(object):
  def __init__(self, dbc_name):
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.apply_brake_last = 0
    self.last_pump_ts = 0.
    self.packer = CANPacker(dbc_name)
    self.new_radar_config = False

    self.prev_lead_distance = 0.0 #previous lead distance in feet
    self.stopped_lead_distance = 0.0
    self.lead_distance_counter = 1.0 #seconds since last update
    self.rough_lead_speed = 0.0 #delta ft/s
    self.desired_lead_distance = 1 #distance bar

  # ft/s - lead_distance is in ft
  # Must be called every frame and assumes 100hz (frames per second)
  def rough_speed(self, lead_distance):

    #If we got an updated lead distance calculate the closing rate
    if self.prev_lead_distance != lead_distance:
      #delta distance is negative when approaching
      delta_distance = lead_distance - self.prev_lead_distance
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

  def get_TR(self, lead_distance, stopped):
    # Slow down sequentially if coming in at higher speed. Radar picks up at about 190ft to 200ft
    if not stopped and (lead_distance <= 255):
      if lead_distance >= 140:
        self.desired_lead_distance = 4
      elif lead_distance >= 110 and lead_distance < 140:
        self.desired_lead_distance = 3
      elif lead_distance >= 60 and lead_distance < 110:
        self.desired_lead_distance = 2
      elif lead_distance >= 0 and lead_distance < 60:
        self.desired_lead_distance = 1
    elif stopped:
      self.desired_lead_distance = 1
    # If caught some traction, lead up closer to moving lead car.
    if (v_ego >= 22) and (lead_distance < 255):
      self.desired_lead_distance = 1
      
    return self.desired_lead_distance

  def update(self, enabled, CS, frame, actuators, \
             pcm_speed, pcm_override, pcm_cancel_cmd, pcm_accel, \
             hud_v_cruise, hud_show_lanes, hud_show_car, \
             hud_alert, snd_beep, snd_chime):

    # *** apply brake hysteresis ***
    brake, self.braking, self.brake_steady = actuator_hystereses(actuators.brake, self.braking, self.brake_steady, CS.v_ego, CS.CP.carFingerprint)

    # *** no output if not enabled ***
    if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = True

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(brake, self.brake_last, -2., 1./100)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    if hud_show_lanes and CS.lkMode:
      hud_lanes = 1
    else:
      hud_lanes = 0

    # Always detect lead car on HUD even without ACC engaged
    if hud_show_car:
      hud_car = 2
    else:
      hud_car = 1

    # For lateral control-only, send chimes as a beep since we don't send 0x1fa
    if CS.CP.radarOffCan:
      snd_beep = snd_beep if snd_beep != 0 else snd_chime

    # Do not send audible alert when steering is disabled
    if not CS.lkMode:
      snd_beep = 0
      snd_chime = 0

    #print("{0} {1} {2}".format(chime, alert_id, hud_alert))
    fcw_display, steer_required, acc_alert = process_hud_alert(hud_alert)

    hud = HUDData(int(pcm_accel), int(round(hud_v_cruise)), 1, hud_car,
                  0xc1, hud_lanes, int(snd_beep), snd_chime, fcw_display, acc_alert, steer_required, CS.lkMode)

    # **** process the car messages ****

    # *** compute control surfaces ***
    BRAKE_MAX = 1024//4
    if CS.CP.carFingerprint in (CAR.ACURA_ILX):
      STEER_MAX = 0xF00
    elif CS.CP.carFingerprint in (CAR.CRV, CAR.ACURA_RDX):
      STEER_MAX = 0x3e8  # CR-V only uses 12-bits and requires a lower value (max value from energee)
    elif CS.CP.carFingerprint in (CAR.ODYSSEY_CHN):
      STEER_MAX = 0x7FFF
    else:
      STEER_MAX = 0x1000

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_gas = clip(actuators.gas, 0., 1.)
    apply_brake = int(clip(self.brake_last * BRAKE_MAX, 0, BRAKE_MAX - 1))
    apply_steer = int(clip(-actuators.steer * STEER_MAX, -STEER_MAX, STEER_MAX))

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
      can_sends.extend(hondacan.create_ui_commands(self.packer, pcm_speed, hud, CS.CP.carFingerprint, CS.is_metric, idx, CS.CP.isPandaBlack))

#    if CS.CP.carFingerprint in (CAR.INSIGHT):
    if CS.CP.carFingerprint in (CAR.INSIGHT, CAR.ACCORD):
      #update the rough speed value store as self.rough_lead_speed
      self.rough_speed(CS.lead_distance)
      #if we are tying to modify distance bars do this:
      if kegman.conf['simpledd'] == True:
        #get the desired distance bar and store it as self.desired_lead_distance
        self.get_TR(CS.lead_distance, CS.stopped)
        #limit the rate distance bars can be pressed
        if frame % 25 < 5 and CS.hud_distance != (self.desired_lead_distance % 4):
        # press distance bar button
          can_sends.append(hondacan.spam_buttons_command(self.packer, 0, CruiseSettings.LEAD_DISTANCE, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
          #print("     spamming distance: " + str((self.desired_lead_distance % 4)))
          # always set cruise setting to 0 after button press
          if frame % 50 < 15:
            can_sends.append(hondacan.spam_buttons_command(self.packer, 0, CruiseSettings.RESET, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
            #print("     spamming distance reset")

    if CS.CP.radarOffCan:
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.CANCEL, 0, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
      elif CS.stopped:
        if CS.CP.carFingerprint in (CAR.INSIGHT):
          if CS.lead_distance > (self.stopped_lead_distance + 8.0) or self.rough_lead_speed > 0.1:
            self.stopped_lead_distance = 0.0
            can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, 0, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
            print("spamming")
          print(self.stopped_lead_distance, CS.lead_distance, self.rough_lead_speed)
        else:
          can_sends.append(hondacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, 0, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
      else:
        self.stopped_lead_distance = CS.lead_distance
        #this gets set in rough speed
        #self.prev_lead_distance = CS.lead_distance
    else:
      # Send gas and brake commands.
      if (frame % 2) == 0:
        idx = frame // 2
        ts = frame * DT_CTRL
        pump_on, self.last_pump_ts = brake_pump_hysteresis(apply_brake, self.apply_brake_last, self.last_pump_ts, ts)
        can_sends.append(hondacan.create_brake_command(self.packer, apply_brake, pump_on,
          pcm_override, pcm_cancel_cmd, hud.chime, hud.fcw, idx, CS.CP.carFingerprint, CS.CP.isPandaBlack))
        self.apply_brake_last = apply_brake

        if CS.CP.enableGasInterceptor:
          # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
          # This prevents unexpected pedal range rescaling
          can_sends.append(create_gas_command(self.packer, apply_gas, idx))

    return can_sends
