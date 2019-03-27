import numpy as np
<<<<<<< HEAD
from cereal import car
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.can.parser import CANParser
from selfdrive.car.subaru.values import DBC, CAR

def get_powertrain_can_parser(CP, canbus):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("LEFT_BLINKER", "Dashlights", 0),
    ("RIGHT_BLINKER", "Dashlights", 0),
    ("Steering_Angle", "Steering_Torque", 0),
    ("Steer_Torque_Sensor", "Steering_Torque", 0),
    ("Brake_Pedal", "Brake_Pedal", 0),
    ("Throttle_Pedal", "Throttle", 0),
    ("FL", "WHEEL_SPEEDS", 0),
    ("FR", "WHEEL_SPEEDS", 0),
    ("RL", "WHEEL_SPEEDS", 0),
    ("RR", "WHEEL_SPEEDS", 0),
    ("DOOR_OPEN_FR", "DOORS_STATUS", 0),
    ("DOOR_OPEN_FL", "DOORS_STATUS", 0),
    ("DOOR_OPEN_RR", "DOORS_STATUS", 0),
    ("DOOR_OPEN_RL", "DOORS_STATUS", 0),
    ("DOOR_OPEN_Hatch", "DOORS_STATUS", 0),
=======
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.can.parser import CANParser
from selfdrive.car.subaru.values import DBC, STEER_THRESHOLD

def get_powertrain_can_parser(CP):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("Steer_Torque_Sensor", "Steering_Torque", 0),
    ("Steering_Angle", "Steering_Torque", 0),
    ("Cruise_On", "CruiseControl", 0),
    ("Cruise_Activated", "CruiseControl", 0),
    ("Brake_Pedal", "Brake_Pedal", 0),
    ("Throttle_Pedal", "Throttle", 0),
    ("LEFT_BLINKER", "Dashlights", 0),
    ("RIGHT_BLINKER", "Dashlights", 0),
    ("SEATBELT_FL", "Dashlights", 0),
    ("FL", "Wheel_Speeds", 0),
    ("FR", "Wheel_Speeds", 0),
    ("RL", "Wheel_Speeds", 0),
    ("RR", "Wheel_Speeds", 0),
    ("DOOR_OPEN_FR", "BodyInfo", 1),
    ("DOOR_OPEN_FL", "BodyInfo", 1),
    ("DOOR_OPEN_RR", "BodyInfo", 1),
    ("DOOR_OPEN_RL", "BodyInfo", 1),
>>>>>>> parent of b264e8b... Revert "Merge branch 'devel' of https://github.com/commaai/openpilot into devel"
  ]

  checks = [
    # sig_address, frequency
    ("Dashlights", 10),
<<<<<<< HEAD
    ("Steering_Torque", 50),
    ("Brake_Pedal", 50),
    ("Throttle", 50),
    ("WHEEL_SPEEDS", 50),
    ("DOORS_STATUS", 10),
  ]

  if CP.carFingerprint in (CAR.OUTBACK, CAR.LEGACY):
    signals += [
      ("Gear", "Transmission", 0),
    ]


  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.powertrain)

def get_eyesight_can_parser(CP, canbus):

  signals = [
    # sig_name, sig_address, default
    ("Cruise_Activated", "ES_Status", 0),
  ]

  checks = []

  if CP.carFingerprint in (CAR.OUTBACK, CAR.LEGACY):
    signals += [
    ("Cruise_On", "ES_Status", 0),
    ("Saved_Speed", "ES_Status", 0),
    ]

  if CP.carFingerprint in (CAR.XV):
    signals += [
    ("Saved_Speed", "ES_DashStatus", 0),
    ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.eyesight)

class CarState(object):
  def __init__(self, CP, canbus):
=======
    ("CruiseControl", 20),
    ("Wheel_Speeds", 50),
    ("Steering_Torque", 100),
    ("BodyInfo", 10),
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

class CarState(object):
  def __init__(self, CP):
>>>>>>> parent of b264e8b... Revert "Merge branch 'devel' of https://github.com/commaai/openpilot into devel"
    # initialize can parser
    self.CP = CP

    self.car_fingerprint = CP.carFingerprint
    self.left_blinker_on = False
    self.prev_left_blinker_on = False
    self.right_blinker_on = False
    self.prev_right_blinker_on = False
    self.steer_torque_driver = 0
    self.steer_not_allowed = False
    self.main_on = False
<<<<<<< HEAD
    self.angle_steers_rate = 0
    self.angle_steers_prev = 0
    self.steer_counter = 1
=======
>>>>>>> parent of b264e8b... Revert "Merge branch 'devel' of https://github.com/commaai/openpilot into devel"

    # vEgo kalman filter
    dt = 0.01
    self.v_ego_kf = KF1D(x0=np.matrix([[0.], [0.]]),
                         A=np.matrix([[1., dt], [0., 1.]]),
                         C=np.matrix([1., 0.]),
                         K=np.matrix([[0.12287673], [0.29666309]]))
    self.v_ego = 0.

<<<<<<< HEAD
  def update(self, pt_cp, es_cp):

    self.can_valid = True

    self.v_wheel_fl = pt_cp.vl["WHEEL_SPEEDS"]['FL'] * CV.KPH_TO_MS
    self.v_wheel_fr = pt_cp.vl["WHEEL_SPEEDS"]['FR'] * CV.KPH_TO_MS
    self.v_wheel_rl = pt_cp.vl["WHEEL_SPEEDS"]['RL'] * CV.KPH_TO_MS
    self.v_wheel_rr = pt_cp.vl["WHEEL_SPEEDS"]['RR'] * CV.KPH_TO_MS
    speed_estimate = float(np.mean([self.v_wheel_fl, self.v_wheel_fr, self.v_wheel_rl, self.v_wheel_rr]))

    self.v_ego_raw = speed_estimate
    v_ego_x = self.v_ego_kf.update(speed_estimate)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = self.v_ego_raw < 0.01

    self.left_blinker_on = pt_cp.vl["Dashlights"]['LEFT_BLINKER'] == 1
    self.right_blinker_on = pt_cp.vl["Dashlights"]['RIGHT_BLINKER'] == 1
    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    self.steer_torque_driver = pt_cp.vl["Steering_Torque"]['Steer_Torque_Sensor']
    self.angle_steers = pt_cp.vl["Steering_Torque"]['Steering_Angle']
    self.pedal_gas = pt_cp.vl["Throttle"]['Throttle_Pedal']
    self.user_gas_pressed = self.pedal_gas > 0
    self.brake_pressure = pt_cp.vl["Brake_Pedal"]['Brake_Pedal']
    self.brake_pressed = self.brake_pressure > 0
    self.brake_lights = bool(self.brake_pressed)

    if self.car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):
      self.cruise_set_speed = es_cp.vl["ES_Status"]['Saved_Speed']
      self.steer_override = abs(self.steer_torque_driver) > 8
      gear = pt_cp.vl["Transmission"]["Gear"]
      if gear == 0:
        self.gear_shifter = "neutral"
      elif gear == 15:
        self.gear_shifter = "park"
      elif gear == 14:
        self.gear_shifter = "reverse"
      else:
        self.gear_shifter = "drive"
      self.main_on = es_cp.vl["ES_Status"]['Cruise_On']

    if self.car_fingerprint == CAR.XV:
      self.cruise_set_speed = es_cp.vl["ES_DashStatus"]['Saved_Speed']
      self.steer_override = abs(self.steer_torque_driver) > 20.0
      self.gear_shifter = "drive"
      self.main_on = es_cp.vl["ES_Status"]['Cruise_Activated']

    self.acc_active = es_cp.vl["ES_Status"]['Cruise_Activated']

    # calculate steer angle change/s
    self.angle_steers_rate = (self.angle_steers - self.angle_steers_prev) * 50
    self.angle_steers_prev = self.angle_steers

    self.door_open = any([pt_cp.vl["DOORS_STATUS"]['DOOR_OPEN_RR'], pt_cp.vl["DOORS_STATUS"]['DOOR_OPEN_RL'], pt_cp.vl["DOORS_STATUS"]['DOOR_OPEN_FR'], pt_cp.vl["DOORS_STATUS"]['DOOR_OPEN_FL'], pt_cp.vl["DOORS_STATUS"]['DOOR_OPEN_Hatch']])
=======
  def update(self, cp):

    self.can_valid = True
    self.pedal_gas = cp.vl["Throttle"]['Throttle_Pedal']
    self.brake_pressure = cp.vl["Brake_Pedal"]['Brake_Pedal']
    self.user_gas_pressed = self.pedal_gas > 0
    self.brake_pressed = self.brake_pressure > 0
    self.brake_lights = bool(self.brake_pressed)

    self.v_wheel_fl = cp.vl["Wheel_Speeds"]['FL'] * CV.KPH_TO_MS
    self.v_wheel_fr = cp.vl["Wheel_Speeds"]['FR'] * CV.KPH_TO_MS
    self.v_wheel_rl = cp.vl["Wheel_Speeds"]['RL'] * CV.KPH_TO_MS
    self.v_wheel_rr = cp.vl["Wheel_Speeds"]['RR'] * CV.KPH_TO_MS

    v_wheel = (self.v_wheel_fl + self.v_wheel_fr + self.v_wheel_rl + self.v_wheel_rr) / 4.
    # Kalman filter, even though Hyundai raw wheel speed is heaviliy filtered by default
    if abs(v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = np.matrix([[v_wheel], [0.0]])

    self.v_ego_raw = v_wheel
    v_ego_x = self.v_ego_kf.update(v_wheel)

    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = self.v_ego_raw < 0.01

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.left_blinker_on = cp.vl["Dashlights"]['LEFT_BLINKER'] == 1
    self.right_blinker_on = cp.vl["Dashlights"]['RIGHT_BLINKER'] == 1
    self.seatbelt_unlatched = cp.vl["Dashlights"]['SEATBELT_FL'] == 1
    self.steer_torque_driver = cp.vl["Steering_Torque"]['Steer_Torque_Sensor']
    self.acc_active = cp.vl["CruiseControl"]['Cruise_Activated']
    self.main_on = cp.vl["CruiseControl"]['Cruise_On']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.car_fingerprint]
    self.angle_steers = cp.vl["Steering_Torque"]['Steering_Angle']
    self.door_open = any([cp.vl["BodyInfo"]['DOOR_OPEN_RR'],
      cp.vl["BodyInfo"]['DOOR_OPEN_RL'],
      cp.vl["BodyInfo"]['DOOR_OPEN_FR'],
      cp.vl["BodyInfo"]['DOOR_OPEN_FL']])
>>>>>>> parent of b264e8b... Revert "Merge branch 'devel' of https://github.com/commaai/openpilot into devel"


