from common.realtime import sec_since_boot
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.nissan import nissancan
from selfdrive.can.packer import CANPacker


class CarControllerParams():
  def __init__(self, car_fingerprint):
    MAX_STEER_DELTA = 1

class CarController(object):
  def __init__(self, car_fingerprint):
    self.start_time = sec_since_boot()
    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.params = CarControllerParams(car_fingerprint)
    print(DBC)
    self.packer = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, sendcan, enabled, CS, frame, actuators):
    """ Controls thread """

    P = self.params

    # Send CAN commands.
    can_sends = []

    ### STEER ###

    apply_steer = actuators.steer

    can_sends.append(nissancan.create_steering_control(self.packer, CS.CP.carFingerprint, apply_steer, frame, enabled))

    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
