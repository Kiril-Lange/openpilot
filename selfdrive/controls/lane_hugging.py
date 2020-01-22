#credits to shane and zorro
from common.numpy_fast import interp
from selfdrive.kegman_conf import kegman_conf

class LaneHugging:
  def __init__(self):
    self.kegman = kegman_conf()
    self.lane_hug_direction = str(self.kegman.conf['lane_hug_direction'])  # if lane hugging is present and which side. None, 'left', or 'right'
    self.lane_hug_mod = float(self.kegman.conf['lane_hug_mod'])  # how much to reduce angle by. float from 1.0 to 2.0
    self.lane_hug_angle = float(self.kegman.conf['lane_hug_angle']), 5)  # where to end increasing angle modification. from 0 to this

  def init(self, angle_steers):
    angle_steers_des = angle_steers
    if self.lane_hug_direction == 'left' and angle_steers > 0:
      angle_steers_des = angle_steers / interp(angle_steers, [0, self.lane_hug_angle], [1.0, self.lane_hug_mod])  # suggestion thanks to zorrobyte
    elif self.lane_hug_direction == 'right' and angle_steers < 0:
      angle_steers_des = angle_steers / interp(angle_steers, [0, self.lane_hug_angle], [1.0, self.lane_hug_mod])
    else:
      return angle_steers

    return angle_steers_des
