from selfdrive.car.subaru.values import CAR, DBC

def create_steering_control(packer, bus, car_fingerprint, idx, steer, checksum):
  if car_fingerprint in (CAR.OUTBACK, CAR.LEGACY):
    values = {
      "Counter": idx,
      "LKAS_Output": steer,
      "LKAS_Active": 1 if steer != 0 else 0,
      "Checksum": checksum
    }
  if car_fingerprint in (CAR.XV):
    values = {
      "Counter": idx,
      "LKAS_Output": steer,
      "LKAS_Active": 1 if steer != 0 else 0,
      "Checksum": checksum,
      "SET_1": 1
    }
  return packer.make_can_msg("ES_LKAS", 0, values)

def create_door_control(packer, bus, car_fingerprint):
    values = {
      "DOOR_OPEN_FR": 1,
      "_UNKNOWN": 5,
    }
    return packer.make_can_msg("DOORS_STATUS", 1, values)
