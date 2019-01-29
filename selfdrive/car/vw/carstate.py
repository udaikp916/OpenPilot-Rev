import numpy as np
from cereal import car
from common.kalman.simple_kalman import KF1D
from selfdrive.config import Conversions as CV
from selfdrive.can.parser import CANParser
from selfdrive.car.vw.values import DBC, CAR

def get_powertrain_can_parser(CP, canbus):
  # this function generates lists for signal, messages and initial values
  signals = [
    # sig_name, sig_address, default
    ("LWI_Lenkradwinkel", "LWI_01", 0),
    ("LWI_VZ_Lenkradwinkel", "LWI_01", 0),
    ("LWI_Lenkradw_Geschw", "LWI_01", 0),
    ("LWI_VZ_Lenkradw_Geschw", "LWI_01", 0),
    ("ESP_HL_Radgeschw_02", "ESP_19", 0),
    ("ESP_HR_Radgeschw_02", "ESP_19", 0),
    ("ESP_VL_Radgeschw_02", "ESP_19", 0),
    ("ESP_VR_Radgeschw_02", "ESP_19", 0),
    ("ACC_Status_ACC", "ACC_06", 0),
  ]

  checks = [
    # sig_address, frequency
    ("LWI_01", 100),
    ("ESP_19", 100),
    ("ACC_06", 50),
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.powertrain)


def get_camera_parser(CP):

  signals = [
    # sig_name, sig_address, default
    ("ACC_Status_ACC", "ACC_06", 0),
  ]

  checks = []

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 1)
class CarState(object):
  def __init__(self, CP, canbus, cp_cam):
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
    self.angle_steers_rate = 0

    # vEgo kalman filter
    dt = 0.01
    self.v_ego_kf = KF1D(x0=np.matrix([[0.], [0.]]),
                         A=np.matrix([[1., dt], [0., 1.]]),
                         C=np.matrix([1., 0.]),
                         K=np.matrix([[0.12287673], [0.29666309]]))
    self.v_ego = 0.

  def update(self, pt_cp, cp_cam):

    self.can_valid = True

    self.v_wheel_fl = pt_cp.vl["ESP_19"]['ESP_HL_Radgeschw_02'] * CV.KPH_TO_MS
    self.v_wheel_fr = pt_cp.vl["ESP_19"]['ESP_HR_Radgeschw_02'] * CV.KPH_TO_MS
    self.v_wheel_rl = pt_cp.vl["ESP_19"]['ESP_VL_Radgeschw_02'] * CV.KPH_TO_MS
    self.v_wheel_rr = pt_cp.vl["ESP_19"]['ESP_VR_Radgeschw_02'] * CV.KPH_TO_MS
    speed_estimate = float(np.mean([self.v_wheel_fl, self.v_wheel_fr, self.v_wheel_rl, self.v_wheel_rr]))

    self.v_ego_raw = speed_estimate
    v_ego_x = self.v_ego_kf.update(speed_estimate)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = self.v_ego_raw < 0.01

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    self.left_blinker_on = False
    self.right_blinker_on = False
    self.steer_torque_driver = 0 #FIXME
    self.acc_active = 1 if cp_cam.vl["ACC_06"]['ACC_Status_ACC'] > 2 else 0
    self.main_on = self.acc_active

    self.steer_override = abs(self.steer_torque_driver) > 1.0

    if pt_cp.vl["LWI_01"]['LWI_VZ_Lenkradwinkel'] == 1:
      self.angle_steers = pt_cp.vl["LWI_01"]['LWI_Lenkradwinkel'] * -1
    else:
      self.angle_steers = pt_cp.vl["LWI_01"]['LWI_VZ_Lenkradwinkel']
    # calculate steer rate
    if pt_cp.vl["LWI_01"]['LWI_VZ_Lenkradw_Geschw'] == 1:
      self.angle_steers_rate = pt_cp.vl["LWI_01"]['LWI_Lenkradw_Geschw'] * -1
    else:
      self.angle_steers_rate = pt_cp.vl["LWI_01"]['LWI_Lenkradw_Geschw']
