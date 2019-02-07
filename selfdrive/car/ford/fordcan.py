import struct
from common.numpy_fast import clip

# *** Ford specific ***
MAX_ANGLE = 87.  # make sure we never command the extremes (0xfff) which cause latching fault

def make_can_msg(addr, dat, alt, cks=False):
  return [addr, 0, dat, alt]


def create_steer_command(packer, angle_cmd, angle_steers, curvature):

  angle_cmd = angle_steers/MAX_ANGLE

  angle_cmd = clip(angle_cmd * MAX_ANGLE, - MAX_ANGLE, MAX_ANGLE)

  values = {
    "LaRampType_B_Req": 0,
    "LaCurvature_No_Calc": clip(curvature, -0.01, 0.01),
    "Steer_Angle_Req": angle_cmd
  }
  return packer.make_can_msg("CAM_Lane_Assist_Data2", 0, values)