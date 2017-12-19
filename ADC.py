#!/usr/bin/env python
""" ADC support """

import os
import struct
import binascii

class PocketCassy(object):
  """ PocketCassy with 524 0621 UIP sensor unit. """

  def __init__(self, devnode="/dev/ldusb0"):
    """ Create instance, accessing given Linux device path. """
    self.__fd = os.open(devnode, os.O_RDWR)
    # Magic command to get a +/-10v ADC reading
    self.__cmd = binascii.unhexlify("0319030F00000000")

  def __del__(self):
    """ Close handles. """
    self.close()

  def close(self):
    """ Close device node.
    """
    if self.__fd:
      os.close(self.__fd)
      self.__fd = None

  def read(self, avg=1):
    """ Read +/-10v range voltage input averaged over "avg" readings.
        Return is voltage as a float.
    """
    real_volts = []
    for _ in xrange(avg):
      os.write(self.__fd, self.__cmd)
      raw_data = os.read(self.__fd, 8)
      _, _, raw_volt, _ = struct.unpack(">hhhh", raw_data)
      # Scale is 5mV / div
      real_volts.append(raw_volt * 0.005)
    return sum(real_volts) / float(len(real_volts))

def main():
  """ Print a single voltage reading (averaged over 10 samples). """
  inst = PocketCassy()
  print inst.read(avg=10)

if __name__ == '__main__':
  main()
