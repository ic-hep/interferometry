#!/usr/bin/env python
"""
Stepper Motor Drivers
"""

import time
import curses
import serial
import struct
import binascii
import threading

class KST101:

  MGMSG_MOD_IDENTIFY          = 0x0223
  MGMSG_HW_START_UPDATEMSGS   = 0x0011
  MGMSG_HW_STOP_UPDATEMSGS    = 0x0012
  MGMSG_MOT_MOVE_HOME         = 0x0443
  MGMSG_MOT_GET_STATUSUPDATE  = 0x0481
  MGMSG_MOT_SET_MOVERELPARAMS = 0x0445
  MGMSG_MOT_MOVE_RELATIVE     = 0x0448
  MGMSG_MOT_SET_MOVEABSPARAMS = 0x0450
  MGMSG_MOT_MOVE_ABSOLUTE     = 0x0453

  @staticmethod
  def step2mm(step):
    return step / 2184560.64

  @staticmethod
  def mm2step(pos):
    return pos * 2184560.64

  def __init__(self, port='/dev/ttyUSB0', debug=False, dst=0x50):
    # Motor parameters
    self.__pos = -1
    self.__status = 0xFFFFFFFF
    self.__param_lock = threading.Lock()
    #
    self.__debug = debug
    self.__src = 0x01 # 0x01 = PC Controller
    self.__dst = dst
    self.__ser =  serial.Serial(port, 115200, timeout=0, rtscts=True)
    if not self.__ser.is_open:
      self.__ser.open()
    self.__thread = threading.Thread(target=self.__recv)
    self.__thread.setDaemon(True)
    self.__thread.start()

  def __send_short(self, msg_id, param1, param2):
    data_out = struct.pack("<HBBBB", msg_id, param1, param2,
                           self.__dst, self.__src)
    if self.__debug:
      print ">>> %s" % binascii.hexlify(data_out)
    self.__ser.write(data_out)
    self.__ser.flush()

  def __send_long(self, msg_id, data):
    data_out = struct.pack("<HHBB", msg_id, len(data),
                           self.__dst | 0x80, self.__src)
    data_out += data
    if self.__debug:
      print ">>> %s" % binascii.hexlify(data_out)
    self.__ser.write(data_out)
    self.__ser.flush()

  def __decode_status(self, data):
    chan, pos, enc_count, status = struct.unpack("<HllL", data)
    if chan != 0x01:
      # Unknown channel, ignore
      return
    self.__param_lock.acquire()
    self.__pos = pos
    self.__status = status
    self.__param_lock.release()

  def __recv(self):
    in_buffer = ""
    self.__ser.reset_input_buffer()
    while True:
      data_in = self.__ser.read(64)
      if not len(in_buffer) and not len(data_in):
        time.sleep(0.1)
        continue
      if self.__debug:
        print "<<< %s" % binascii.hexlify(data_in)
      in_buffer += data_in
      if len(in_buffer) < 6:
        continue
      msg_id, param1, param2, dst, src = struct.unpack("<HBBBB", in_buffer[0:6])
      msg_len = 6
      if dst | 0x80:
        dst &= 0x7F
        msg_len += ((param2 << 8) | param1)
      # Check this message is from the controller and for us
      if dst != self.__src or src != self.__dst:
        in_buffer = ""
        continue
      if len(in_buffer) < msg_len:
        # Message is incomplete
        continue
      # Now look for messages we recognise
      if msg_id == self.MGMSG_MOT_GET_STATUSUPDATE:
        self.__decode_status(in_buffer[6:msg_len])
        in_buffer = in_buffer[msg_len:]
        continue
      # If we got here, the message was invalid
      # Clear the buffer to resynchronise
      in_buffer = ""
      continue

  def identify(self):
    self.__send_short(self.MGMSG_MOD_IDENTIFY, 0x00, 0x00)

  def en_update_msg(self, enable=True):
    if enable:
      self.__send_short(self.MGMSG_HW_START_UPDATEMSGS, 0x10, 0x00)
    else:
      self.__send_short(self.MGMSG_HW_STOP_UPDATEMSGS, 0x00, 0x00)

  def home(self):
    self.__send_short(self.MGMSG_MOT_MOVE_HOME, 0x01, 0x00)

  def set_move(self, dist, rel=True):
    cmd = self.MGMSG_MOT_SET_MOVEABSPARAMS
    if rel:
      cmd = self.MGMSG_MOT_SET_MOVERELPARAMS
    data = struct.pack("<Hl", 0x01, dist)
    self.__send_long(cmd, data)

  def do_move(self, rel=True):
    cmd = self.MGMSG_MOT_MOVE_ABSOLUTE
    if rel:
      cmd = self.MGMSG_MOT_MOVE_RELATIVE
    self.__send_short(cmd, 0x01, 0x00)

  def get_pos(self):
    self.__param_lock.acquire()  
    pos = self.__pos
    self.__param_lock.release()
    return pos

  def get_raw_status(self):
    self.__param_lock.acquire()  
    status = self.__status
    self.__param_lock.release()
    return status

  def get_home_state(self):
    raw_status = self.get_raw_status()
    is_home = raw_status & 0x400
    is_homing = raw_status & 0x200
    if is_homing:
      return 2
    if not is_home:
      return 1
    return 0

  def is_moving(self):
    is_moving = self.get_raw_status() & 0xF0
    return bool(is_moving)

  def is_lower_limit(self):
    is_lower = self.get_raw_status() & 0x1
    return bool(is_lower)

  def is_upper_limit(self):
    is_upper = self.get_raw_status() & 0x2
    return bool(is_upper)


def main(scr, mtr):
  step_size = 1
  old_step_size = step_size
  curses.curs_set(False)
  scr.nodelay(True)
  while True:
    in_key = scr.getch()
    if in_key == 49:   step_size = 1           # 1
    elif in_key == 50: step_size = 10          # 2
    elif in_key == 51: step_size = 100         # 3
    elif in_key == 52: step_size = 1000        # 4
    elif in_key == 53: step_size = 10000       # 5
    elif in_key == 54: step_size = 100000      # 6
    elif in_key == 55: step_size = 1000000     # 7
    elif in_key == 56: step_size = 10000000    # 8
    elif in_key == 114:                        # r
      step_size *= -1
    elif in_key == 32:                         # space
      mtr.do_move()
    elif in_key == 104:                        # h
      mtr.home()
    elif in_key == 48:                         # 0
      mtr.set_move(0, False)
      mtr.do_move(False)
      mtr.set_move(step_size)
    elif in_key == 113 or in_key == 27:        # q / escape
      break # Quit
    
    if (step_size != old_step_size):
      mtr.set_move(step_size)
      old_step_size = step_size

    scr.clear()
    pos = mtr.get_pos()
    scr.addstr(1, 2, "Steps: %u" % pos)
    scr.addstr(2, 2, "   mm: %0.4f" % mtr.step2mm(pos))
    scr.addstr(3, 2, " Size: %u" % step_size)
    home_state = mtr.get_home_state()
    if home_state == 2:
      scr.addstr(5, 2, " *** Motor homing, please wait... ***")
    elif home_state == 1:
      scr.addstr(5, 2, " *** Motor needs homing (press h) ***")
    elif mtr.is_lower_limit():
      scr.addstr(5, 2, " ***   Motor is at lower limit.   ***")
    elif mtr.is_upper_limit():
      scr.addstr(5, 2, " ***   Motor is at upper limit.   ***")
    elif mtr.is_moving():
      scr.addstr(5, 2, " *** Motor is moving to position. ***")
    
    scr.addstr(21, 2, "  Status: %08X" % mtr.get_raw_status())
    scr.addstr(20, 2, "  Key: %u" % int(in_key))
    scr.refresh()
    time.sleep(0.1)

if __name__ == '__main__':
  mtr = KST101()
  mtr.en_update_msg(True)
  #print "Please wait, homing..."
  #mtr.home()
  #while not mtr.is_homed():
  #  time.sleep(0.5)
  #print "Homing complete."

  curses.wrapper(main, mtr)

  #while True:
  #  pos = x.get_pos()
  #  print "%u (%0.4f)" % (pos, x.step2mm(pos))
  #  time.sleep(1.0)

