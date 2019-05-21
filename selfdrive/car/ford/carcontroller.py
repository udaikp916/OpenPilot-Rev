from common.numpy_fast import clip, interp
from cereal import car
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.ford.fordcan import make_can_msg, create_steer_command
from selfdrive.can.packer import CANPacker

class CarController(object):
  def __init__(self, dbc_name, enable_camera, vehicle_model):
    self.packer = CANPacker(dbc_name)
    self.vehicle_model = vehicle_model

  def update(self, sendcan, enabled, CS, frame, actuators):

    can_sends = []
    '''
    if (frame % 1) == 0:
      if enabled:
        can_sends(0x3ca, 0, "\x9f\x00\x00\x00\x00\x00\xac\x00", 0)
      else:
        can_sends(0x3ca, 0, "\xff\x00\x00\x00\x00\x00\xbe\x00", 0)

    if (frame % 1) == 0:
      apply_steer = actuators.steer
      curvature = self.vehicle_model.calc_curvature(actuators.steerAngle*3.1415/180., CS.v_ego)

      can_sends.append(create_steer_command(self.packer, apply_steer, CS.angle_steers, curvature))

    if (frame % 10) == 0:
      can_sends(0x3c1, 0, "\x00\x00\x00\x00\x00\x00\x00\x00", 0)

    if (frame % 100) == 0:
      if enabled:
        can_sends(0x3d8, 0, "\x00\x00\x00\x00\x00\x00\xac\x00", 0)
      else:
        can_sends(0x3d8, 0, "\x00\x00\x00\x00\x00\x00\xbe\x00", 0)
    '''
    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
