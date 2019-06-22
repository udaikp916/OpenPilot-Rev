import crcmod
from selfdrive.car.nissan.values import CAR

nissan_checksum = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)

def create_steering_control(packer, car_fingerprint, apply_steer, frame, steer_on):  
  if car_fingerprint == CAR.XTRAIL:
    idx = (frame % 16)
    values = {
      "Des_Angle": apply_steer,
      "SET_0x80_2": 0x80,
      "SET_X80": 0x80,
      "NEW_SIGNAL_4": 0x06 if steer_on else 0,
      "Counter": idx,
      "LKA_Active": 1 if steer_on else 0,
    }
    
    dat = packer.make_can_msg("LKAS", 0, values)[2]

    values["CRC"] = nissan_checksum(dat[:14])

  return packer.make_can_msg("LKAS", 0, values)
