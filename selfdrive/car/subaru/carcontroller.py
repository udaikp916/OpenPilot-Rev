from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.subaru.carstate import CarState, get_powertrain_can_parser, get_eyesight_can_parser
from selfdrive.car.subaru import subarucan
from selfdrive.car.subaru.values import CAR, DBC
from selfdrive.can.packer import CANPacker


class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.STEER_MAX = 2047                 # max_steer 4095
    self.STEER_STEP = 1                   # how often we update the steer cmd
    self.STEER_DELTA_UP = 60              # torque increase per refresh
    self.STEER_DELTA_DOWN = 60            # torque decrease per refresh
    if car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):
      self.STEER_DRIVER_ALLOWANCE = 400   # allowed driver torque before start limiting FIXME: NOT YET SCALED
    else:
      self.STEER_DRIVER_ALLOWANCE = 2500   # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 5      # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 80         # scale driver torque to lka torque



class CarController(object):
  def __init__(self, canbus, car_fingerprint):
    self.start_time = sec_since_boot()
    self.lkas_active = False
    self.steer_idx = 0
    self.apply_steer_last = 0
    self.last_blinker_frame = 0
    self.car_fingerprint = car_fingerprint

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.canbus = canbus
    self.params = SteerLimitParams(car_fingerprint)
    print(DBC)
    self.packer_pt = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, sendcan, enabled, CS, frame, actuators):
    """ Controls thread """

    P = self.params

    # Send CAN commands.
    can_sends = []
    canbus = self.canbus

    ### STEER ###

    if (frame % P.STEER_STEP) == 0:

      final_steer = actuators.steer if enabled else 0.
      apply_steer = apply_std_steer_torque_limits(final_steer*P.STEER_MAX, self.apply_steer_last, CS.steer_torque_driver, self.params)
      self.apply_steer_last = apply_steer

      lkas_enabled = enabled and not CS.steer_not_allowed

      if CS.left_blinker_on or CS.right_blinker_on:
        self.last_blinker_frame = frame

      if (frame - self.last_blinker_frame) < 50:
        lkas_enabled = False

      if not lkas_enabled:
          apply_steer = 0

      if self.car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):

        if apply_steer != 0:
          chksm_steer = apply_steer * -1
          chksm_engage = 1
        else:
          chksm_steer = 0
          chksm_engage = 0

        #counts from 0 to 7 then back to 0
        idx = (frame / P.STEER_STEP) % 8
        steer2 = (chksm_steer >> 8) & 0x1F
        steer1 =  chksm_steer - (steer2 << 8)
        checksum = (idx + steer2 + steer1 + chksm_engage) % 256

      if (self.car_fingerprint == CAR.XV):

        #counts from 0 to 15 then back to 0 + 16 for enable bit
        if apply_steer != 0:
          chksm_steer = apply_steer * -1
          chksm_engage = 32
        else:
          chksm_engage = 0

        idx = ((frame / P.STEER_STEP) % 16)
        steer2 = (chksm_steer >> 8) & 0x1F
        steer1 =  chksm_steer - (steer2 << 8)
        byte2 = steer2 + chksm_engage

        checksum = ((idx + 16 + steer1 + byte2 + 35) % 256)

      can_sends.append(subarucan.create_steering_control(self.packer_pt, canbus.powertrain, CS.CP.carFingerprint, idx, apply_steer, checksum))

    if self.car_fingerprint in (CAR.OUTBACK, CAR.LEGACY) and not enabled and CS.acc_active == 1:
      can_sends.append(subarucan.create_door_control(self.packer_pt, canbus.eyesight, CS.CP.carFingerprint))


    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())