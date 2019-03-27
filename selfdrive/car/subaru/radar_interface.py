#!/usr/bin/env python
from cereal import car
import time


class RadarInterface(object):
  def __init__(self, CP):
    # radar
<<<<<<< HEAD
    self.pts = {1} #return no points
=======
    self.pts = {}
>>>>>>> parent of b264e8b... Revert "Merge branch 'devel' of https://github.com/commaai/openpilot into devel"
    self.delay = 0.1

  def update(self):

    ret = car.RadarState.new_message()
    time.sleep(0.05)  # radard runs on RI updates
<<<<<<< HEAD
    
    return ret

if __name__ == "__main__":
  RI = RadarInterface()
=======

    return ret

if __name__ == "__main__":
  RI = RadarInterface(None)
>>>>>>> parent of b264e8b... Revert "Merge branch 'devel' of https://github.com/commaai/openpilot into devel"
  while 1:
    ret = RI.update()
    print(chr(27) + "[2J")
    print ret
