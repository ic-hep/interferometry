#!/usr/bin/env python
"""
Stepper Motor Drivers
"""

import time
import curses
import struct
import binascii
import threading
import serial

class KST101(object):
  """ A class for controlling a Thorlabs KST101 over USB.
      This is primarily focused on a ZFS13B stage
      (absolute positions may be wrong for other stages)
  """

  # Message IDs
  MGMSG_HW_START_UPDATEMSGS   = 0x0011
  MGMSG_HW_STOP_UPDATEMSGS    = 0x0012
  MGMSG_MOD_IDENTIFY          = 0x0223
  MGMSG_MOT_MOVE_HOME         = 0x0443
  MGMSG_MOT_MOVE_HOMED        = 0x0444
  MGMSG_MOT_SET_MOVERELPARAMS = 0x0445
  MGMSG_MOT_MOVE_RELATIVE     = 0x0448
  MGMSG_MOT_SET_MOVEABSPARAMS = 0x0450
  MGMSG_MOT_MOVE_ABSOLUTE     = 0x0453
  MGMSG_MOT_MOVE_COMPLETED    = 0x0464
  MGMSG_MOT_GET_STATUSUPDATE  = 0x0481

  # Status register masks
  STATUS_HOMING = 0x200
  STATUS_HOMED  = 0x400
  STATUS_MOVING = 0x0F0
  STATUS_LLIM   = 0x001
  STATUS_ULIM   = 0x002

  # Steps per rotation * (gear ratio) * thread pitch (1.0)
  STEPS_PER_MM = 49152.0 * (400.0 / 9.0)

  @staticmethod
  def step2mm(step):
    """ Convert a motor step position to absolute mm. """
    return step / KST101.STEPS_PER_MM

  @staticmethod
  def mm2step(pos):
    """ Convert an absolute position to a number of steps. """
    return pos * KST101.STEPS_PER_MM

  def __init__(self, port='/dev/ttyUSB0', debug=False, dst=0x50):
    """
    Open serial port and connect to controller.
    port should be a serial device node (or serial port name on Windows.
    If debug is set to true, raw messages are printed to stdout.
    dst is the motor address, defaults to value for directly attached motor.
    In-built source assumes this is running on a PC (0x01).
    This also assumes that the controller has a single motor channel.
    """
    # Motor parameters
    self.__pos = -1
    self.__status = 0xFFFFFFFF
    self.__param_lock = threading.Lock()
    #
    self.__chan = 0x01 # Controller only has one channel, number 1.
    self.__debug = debug
    self.__src = 0x01 # 0x01 = PC Controller
    self.__dst = dst
    self.__ser = serial.Serial(port, 115200, timeout=0, rtscts=True)
    if not self.__ser.is_open:
      self.__ser.open()
    self.__thread = threading.Thread(target=self.__recv)
    self.__thread.setDaemon(True)
    self.__thread.start()

  def __send_short(self, msg_id, param1, param2):
    """ Sends a single "short format" (6-byte) message to the controller.
        msg_id is the 2-byte message code with byte parameters param1 & 2.
        Returns nothing.
    """
    data_out = struct.pack("<HBBBB", msg_id, param1, param2,
                           self.__dst, self.__src)
    if self.__debug:
      print ">>> %s" % binascii.hexlify(data_out)
    self.__ser.write(data_out)
    self.__ser.flush()

  def __send_long(self, msg_id, data):
    """ Send long sends a packet in the "long-format" (>6-byte) format to
        the controller. msg_id is the 2-byte ID code and data should be the
        raw payload to include.
    """
    # Long format packets have the upper bit of DST set.
    data_out = struct.pack("<HHBB", msg_id, len(data),
                           self.__dst | 0x80, self.__src)
    data_out += data
    if self.__debug:
      print ">>> %s" % binascii.hexlify(data_out)
    self.__ser.write(data_out)
    self.__ser.flush()

  def __decode_status(self, data):
    """ Decodes a status message payload from the controller and updates
        the local state of the motor.
        This function is thread safe.
    """
    chan, pos, _, status = struct.unpack("<HllL", data)
    if chan != self.__chan:
      # Unknown channel, ignore
      return
    self.__param_lock.acquire()
    self.__pos = pos
    self.__status = status
    self.__param_lock.release()

  def __recv(self):
    """ Inner thread to handle messages recieved from the controller.
    """
    in_buffer = ""
    self.__ser.reset_input_buffer()
    while True:
      data_in = self.__ser.read(64)
      if not in_buffer and not data_in:
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
        # Packet is long format
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
      elif msg_id == self.MGMSG_MOT_MOVE_COMPLETED:
        in_buffer = in_buffer[msg_len:]
        continue
      elif msg_id == self.MGMSG_MOT_MOVE_HOMED:
        in_buffer = in_buffer[msg_len:]
        continue
      # If we got here, the message was invalid
      # Clear the buffer to resynchronise
      in_buffer = ""
      continue

  def identify(self):
    """ Flash the controller screen for a few seconds. """
    self.__send_short(self.MGMSG_MOD_IDENTIFY, 0x00, 0x00)

  def en_update_msg(self, enable=True, freq=64):
    """ Enable/disable automatic sending of update messages.
        Freq sets the frequency of the messages (if enabling, otherwise
        parameter is ignored.
    """
    if enable:
      self.__send_short(self.MGMSG_HW_START_UPDATEMSGS, freq, 0x00)
    else:
      self.__send_short(self.MGMSG_HW_STOP_UPDATEMSGS, 0x00, 0x00)

  def home(self):
    """ Start the homing procedure. """
    self.__send_short(self.MGMSG_MOT_MOVE_HOME, self.__chan, 0x00)

  def set_move(self, dist, rel=True):
    """ Prepare to move the motor:
        dist - if rel=True the number of steps to move the motor,
               otherwise the absolute number of steps to move the motor to.
    """
    cmd = self.MGMSG_MOT_SET_MOVEABSPARAMS
    if rel:
      cmd = self.MGMSG_MOT_SET_MOVERELPARAMS
    data = struct.pack("<Hl", self.__chan, dist)
    self.__send_long(cmd, data)

  def do_move(self, rel=True):
    """ Execure a move.
        This should be called after set_move with a similar value of
        rel.
    """
    cmd = self.MGMSG_MOT_MOVE_ABSOLUTE
    if rel:
      cmd = self.MGMSG_MOT_MOVE_RELATIVE
    self.__send_short(cmd, self.__chan, 0x00)

  def get_pos(self):
    """ Get the current position of the motor (in steps). """
    self.__param_lock.acquire()
    pos = self.__pos
    self.__param_lock.release()
    return pos

  def get_raw_status(self):
    """ Get the raw status field. """
    self.__param_lock.acquire()
    status = self.__status
    self.__param_lock.release()
    return status

  def get_home_state(self):
    """ Get the current home state of the motor.
        Returns 0 if the motor is correctly homed.
        1 if the motor requires homing.
        2 if the homing procedure is currently running.
    """
    raw_status = self.get_raw_status()
    is_home = raw_status & self.STATUS_HOMED
    is_homing = raw_status & self.STATUS_HOMING
    if is_homing:
      return 2
    if not is_home:
      return 1
    return 0

  def is_moving(self):
    """ Returns true if the motor is currently moving.
    """
    is_moving = self.get_raw_status() & self.STATUS_MOVING
    return bool(is_moving)

  def is_lower_limit(self):
    """ Returns true if the motor is at its lower limit.
    """
    is_lower = self.get_raw_status() & self.STATUS_LLIM
    return bool(is_lower)

  def is_upper_limit(self):
    """ Returns true if the motor is at its upper limit.
    """
    is_upper = self.get_raw_status() & self.STATUS_ULIM
    return bool(is_upper)


def main(scr):
  """ Simple test program entry point.
      scr is a curser screen (designed to be called with curses wrapper).
  """
  mtr = KST101()
  mtr.en_update_msg(True)

  step_size = 1
  old_step_size = -1
  null_point = 0
  curses.curs_set(False)
  scr.nodelay(True)
  while True:
    pos = mtr.get_pos()
    in_key = scr.getch()
    if in_key == 49:   step_size = 1    # 1
    elif in_key == 50: step_size = 1e1  # 2
    elif in_key == 51: step_size = 1e2  # 3
    elif in_key == 52: step_size = 1e3  # 4
    elif in_key == 53: step_size = 1e4  # 5
    elif in_key == 54: step_size = 1e5  # 6
    elif in_key == 55: step_size = 1e6  # 7
    elif in_key == 56: step_size = 1e7  # 8
    elif in_key == 57: step_size = 1e8  # 9
    elif in_key == 114:                 # r
      step_size *= -1
    elif in_key == 32:                  # space
      mtr.do_move()
    elif in_key == 104:                 # h
      mtr.home()
    elif in_key == 110:                 # n
      null_point = pos
    elif in_key == 122:                 # z
      null_point = 0
    elif in_key == 48:                  # 0
      mtr.set_move(null_point, False)
      mtr.do_move(False)
      mtr.set_move(step_size)
    elif in_key == 113 or in_key == 27: # q / escape
      break # Quit
 
    if step_size != old_step_size:
      mtr.set_move(step_size)
      old_step_size = step_size

    scr.clear()
    scr.addstr(1, 2, " Steps: %u" % (pos - null_point))
    scr.addstr(2, 2, "   mm: %0.4f" % mtr.step2mm(pos - null_point))
    scr.addstr(3, 2, " Size: %u" % step_size)
    scr.addstr(4, 2, " Null: %u" % null_point)
    home_state = mtr.get_home_state()
    if home_state == 2:
      scr.addstr(6, 2, " *** Motor homing, please wait... ***")
    elif home_state == 1:
      scr.addstr(6, 2, " *** Motor needs homing (press h) ***")
    elif mtr.is_lower_limit():
      scr.addstr(6, 2, " ***   Motor is at lower limit.   ***")
    elif mtr.is_upper_limit():
      scr.addstr(6, 2, " ***   Motor is at upper limit.   ***")
    elif mtr.is_moving():
      scr.addstr(6, 2, " *** Motor is moving to position. ***")
  
    # Lower status
    scr.addstr(16, 2, "  Controls: 1-9 - set step size to 10^(n-1)," \
                        " <spc> - move")
    scr.addstr(17, 2, "            r - reverse, h - home, 0 - go to null")
    scr.addstr(18, 2, "            n - set null, z - zero null")
    scr.addstr(21, 2, "  Status: %08X" % mtr.get_raw_status())
    scr.addstr(20, 2, "  Key: %u" % int(in_key))
    scr.refresh()
    time.sleep(0.1)

if __name__ == '__main__':
  curses.wrapper(main)

