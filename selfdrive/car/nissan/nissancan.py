import copy
import crcmod
from cereal import car
from selfdrive.car.nissan.values import CAR

nissan_checksum = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)

def nissan_checksum(packer, values, addr):
  dat = packer.make_can_msg(addr, 0, values)[2]
  dat = [ord(i) for i in dat]
  return (sum(dat[1:]) + (addr >> 8) + addr) & 0xff

def create_steering_control(packer, car_fingerprint, apply_steer, frame, enabled):  
  if car_fingerprint == CAR.XTRAIL:
    idx = (frame % 16)
    values = {
      "Des_Angle": apply_steer,
      "SET_0x80_2": 0x80,
      "SET_X80": 0x80,
      "NEW_SIGNAL_4": 0x06 if enabled else 0,
      "Counter": idx,
      "LKA_Active": 1 if enabled else 0,
    }
    
    dat = packer.make_can_msg("LKAS", 0, values)[2]

    values["CRC"] = nissan_checksum(dat)

  return packer.make_can_msg("LKAS", 0, values)
