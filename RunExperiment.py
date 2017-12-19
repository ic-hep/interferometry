#!/usr/bin/env python

import time

from MotorDriver import KST101
from ADC import PocketCassy
import matplotlib.pyplot as plt

STEP_SIZE = 100
NUM_READINGS = 1000

def main():
  results = []
  mtr = KST101()
  mtr.en_update_msg(True)
  adc = PocketCassy()
  mtr.set_move(STEP_SIZE)
  for i in xrange(NUM_READINGS):
    mtr.do_move()
    time.sleep(0.1)
    res = adc.read(avg=10)
    print i, res
    results.append(res)
  plt.plot(results)
  plt.show()

if __name__ == '__main__':
  main()
