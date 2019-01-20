from selfdrive.car.vw.values import CAR, DBC

def create_steering_control(packer, bus, car_fingerprint, steer, idx, lkas_enabled):
  if car_fingerprint in (CAR.GOLF):

    values = {
      "CRC": crc,
      "Counter": idx,
      "3": 3,
      "Steer_Torque": steer,
      "LKAS_ON": lkas_enabled,
      "Torque_Not_0": 1 if steer != 0 else 0,
      "254": 254,
      "7": 7
    }
    return packer.make_can_msg("LKAS", 0, values)