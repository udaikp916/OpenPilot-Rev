from common.numpy_fast import clip, interp
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.vw.carstate import CarState, get_gateway_can_parser, get_extended_can_parser
from selfdrive.car.vw import vwcan
from selfdrive.car.vw.values import CAR, DBC
from selfdrive.can.packer import CANPacker


class CarControllerParams():
  def __init__(self, car_fingerprint):
    self.STEER_MAX = 300               # max stock steer 300
    self.STEER_STEP = 2                # how often we update the steer cmd
    self.STEER_DELTA_UP = 50           # torque increase per refresh
    self.STEER_DELTA_DOWN = 50         # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = 20   # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 1
    self.STEER_DRIVER_FACTOR = 1

    self.HUD_STEP = 10                  # how often we send the LDW_02 HUD update



class CarController(object):
  def __init__(self, canbus, car_fingerprint):
    self.start_time = sec_since_boot()
    self.counter = 0
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint

    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    self.canbus = canbus
    self.params = CarControllerParams(car_fingerprint)
    print(DBC)
    self.packer_gw = CANPacker(DBC[car_fingerprint]['pt'])

  def update(self, sendcan, enabled, CS, frame, actuators):
    """ Controls thread """

    P = self.params

    # Send CAN commands.
    can_sends = []
    canbus = self.canbus

    #
    # Prepare HCA_01 steering torque message
    #
    if (frame % P.STEER_STEP) == 0:
      if enabled and not CS.standstill:
        # TODO: apply safety boundaries here for increase and decrease rates
        lkas_enabled = 1
        apply_steer = int(clip(actuators.steer * P.STEER_MAX, -P.STEER_MAX, P.STEER_MAX))
        # steer must be a positive value
        if apply_steer > -1:
          steer = apply_steer
          right = 0
        else:
          steer = abs(apply_steer)
          right = 1
      else:
        lkas_enabled = 0
        steer = 0
        right = 0
      idx = (frame / P.STEER_STEP) % 16
      can_sends.append(vwcan.create_steering_control(self.packer_gw, canbus.gateway, CS.CP.carFingerprint, steer, idx, lkas_enabled, right))

    #
    # Prepare LDW_02 HUD message with lane lines and confidence levels
    # TODO: Include lane recognition confidence levels instead of statically turning lane-lines on
    #
    if (frame % P.HUD_STEP) == 0:
      if enabled and not CS.standstill:
        lkas_enabled = 1
      else:
        lkas_enabled = 0
      can_sends.append(vwcan.create_hud_control(self.packer_gw, canbus.gateway, CS.CP.carFingerprint, lkas_enabled))

    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())