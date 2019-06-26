from common.realtime import sec_since_boot
from selfdrive.car.nissan import nissancan
from selfdrive.car.nissan.values import CAR, DBC
from selfdrive.can.packer import CANPacker


class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.STEER_MAX = 1              # max_steer 4095

class CarController(object):
  def __init__(self, car_fingerprint):
    self.start_time = sec_since_boot()
    self.car_fingerprint = car_fingerprint

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.params = CarControllerParams(car_fingerprint)
    print(DBC)
    self.packer = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, CS, frame, actuators): #, pcm_cancel_cmd, visual_alert, left_line, right_line):
    """ Controls thread """

    P = self.params

    # Send CAN commands.
    can_sends = []

    if (frame % 2) == 0:
      ### STEER ###
      steer_on = CS.steer_on
      apply_steer = actuators.steer
      lkas = CS.lkas

      can_sends.append(nissancan.create_steering_control(self.packer, CS.CP.carFingerprint, apply_steer, frame, steer_on, lkas))

    return can_sends
