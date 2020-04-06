# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dbw_mkz_msgs/BrakeReport.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import dbw_mkz_msgs.msg
import std_msgs.msg

class BrakeReport(genpy.Message):
  _md5sum = "a306c167d365176ae6159e3c4e3f3197"
  _type = "dbw_mkz_msgs/BrakeReport"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

# Brake pedal
# Unitless, range 0.15 to 0.50
float32 pedal_input
float32 pedal_cmd
float32 pedal_output

# Braking torque (Nm)
float32 torque_input
float32 torque_cmd
float32 torque_output

# Brake On Off (BOO), brake lights
bool boo_input
bool boo_cmd
bool boo_output

# Status
bool enabled  # Enabled
bool override # Driver override
bool driver   # Driver activity

# Watchdog Counter
WatchdogCounter watchdog_counter
bool watchdog_braking
bool fault_wdc

# Faults
bool fault_ch1
bool fault_ch2
bool fault_boo
bool fault_connector # This fault can be ignored

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: dbw_mkz_msgs/WatchdogCounter
uint8 source

uint8 NONE=0               # No source for watchdog counter fault
uint8 OTHER_BRAKE=1        # Fault determined by brake controller
uint8 OTHER_THROTTLE=2     # Fault determined by throttle controller
uint8 OTHER_STEERING=3     # Fault determined by steering controller
uint8 BRAKE_COUNTER=4      # Brake command counter failed to increment
uint8 BRAKE_DISABLED=5     # Brake transition to disabled while in gear or moving
uint8 BRAKE_COMMAND=6      # Brake command timeout after 100ms
uint8 BRAKE_REPORT=7       # Brake report timeout after 100ms
uint8 THROTTLE_COUNTER=8   # Throttle command counter failed to increment
uint8 THROTTLE_DISABLED=9  # Throttle transition to disabled while in gear or moving
uint8 THROTTLE_COMMAND=10  # Throttle command timeout after 100ms
uint8 THROTTLE_REPORT=11   # Throttle report timeout after 100ms
uint8 STEERING_COUNTER=12  # Steering command counter failed to increment
uint8 STEERING_DISABLED=13 # Steering transition to disabled while in gear or moving
uint8 STEERING_COMMAND=14  # Steering command timeout after 100ms
uint8 STEERING_REPORT=15   # Steering report timeout after 100ms
"""
  __slots__ = ['header','pedal_input','pedal_cmd','pedal_output','torque_input','torque_cmd','torque_output','boo_input','boo_cmd','boo_output','enabled','override','driver','watchdog_counter','watchdog_braking','fault_wdc','fault_ch1','fault_ch2','fault_boo','fault_connector']
  _slot_types = ['std_msgs/Header','float32','float32','float32','float32','float32','float32','bool','bool','bool','bool','bool','bool','dbw_mkz_msgs/WatchdogCounter','bool','bool','bool','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,pedal_input,pedal_cmd,pedal_output,torque_input,torque_cmd,torque_output,boo_input,boo_cmd,boo_output,enabled,override,driver,watchdog_counter,watchdog_braking,fault_wdc,fault_ch1,fault_ch2,fault_boo,fault_connector

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(BrakeReport, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pedal_input is None:
        self.pedal_input = 0.
      if self.pedal_cmd is None:
        self.pedal_cmd = 0.
      if self.pedal_output is None:
        self.pedal_output = 0.
      if self.torque_input is None:
        self.torque_input = 0.
      if self.torque_cmd is None:
        self.torque_cmd = 0.
      if self.torque_output is None:
        self.torque_output = 0.
      if self.boo_input is None:
        self.boo_input = False
      if self.boo_cmd is None:
        self.boo_cmd = False
      if self.boo_output is None:
        self.boo_output = False
      if self.enabled is None:
        self.enabled = False
      if self.override is None:
        self.override = False
      if self.driver is None:
        self.driver = False
      if self.watchdog_counter is None:
        self.watchdog_counter = dbw_mkz_msgs.msg.WatchdogCounter()
      if self.watchdog_braking is None:
        self.watchdog_braking = False
      if self.fault_wdc is None:
        self.fault_wdc = False
      if self.fault_ch1 is None:
        self.fault_ch1 = False
      if self.fault_ch2 is None:
        self.fault_ch2 = False
      if self.fault_boo is None:
        self.fault_boo = False
      if self.fault_connector is None:
        self.fault_connector = False
    else:
      self.header = std_msgs.msg.Header()
      self.pedal_input = 0.
      self.pedal_cmd = 0.
      self.pedal_output = 0.
      self.torque_input = 0.
      self.torque_cmd = 0.
      self.torque_output = 0.
      self.boo_input = False
      self.boo_cmd = False
      self.boo_output = False
      self.enabled = False
      self.override = False
      self.driver = False
      self.watchdog_counter = dbw_mkz_msgs.msg.WatchdogCounter()
      self.watchdog_braking = False
      self.fault_wdc = False
      self.fault_ch1 = False
      self.fault_ch2 = False
      self.fault_boo = False
      self.fault_connector = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_6f13B().pack(_x.pedal_input, _x.pedal_cmd, _x.pedal_output, _x.torque_input, _x.torque_cmd, _x.torque_output, _x.boo_input, _x.boo_cmd, _x.boo_output, _x.enabled, _x.override, _x.driver, _x.watchdog_counter.source, _x.watchdog_braking, _x.fault_wdc, _x.fault_ch1, _x.fault_ch2, _x.fault_boo, _x.fault_connector))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.watchdog_counter is None:
        self.watchdog_counter = dbw_mkz_msgs.msg.WatchdogCounter()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 37
      (_x.pedal_input, _x.pedal_cmd, _x.pedal_output, _x.torque_input, _x.torque_cmd, _x.torque_output, _x.boo_input, _x.boo_cmd, _x.boo_output, _x.enabled, _x.override, _x.driver, _x.watchdog_counter.source, _x.watchdog_braking, _x.fault_wdc, _x.fault_ch1, _x.fault_ch2, _x.fault_boo, _x.fault_connector,) = _get_struct_6f13B().unpack(str[start:end])
      self.boo_input = bool(self.boo_input)
      self.boo_cmd = bool(self.boo_cmd)
      self.boo_output = bool(self.boo_output)
      self.enabled = bool(self.enabled)
      self.override = bool(self.override)
      self.driver = bool(self.driver)
      self.watchdog_braking = bool(self.watchdog_braking)
      self.fault_wdc = bool(self.fault_wdc)
      self.fault_ch1 = bool(self.fault_ch1)
      self.fault_ch2 = bool(self.fault_ch2)
      self.fault_boo = bool(self.fault_boo)
      self.fault_connector = bool(self.fault_connector)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_get_struct_6f13B().pack(_x.pedal_input, _x.pedal_cmd, _x.pedal_output, _x.torque_input, _x.torque_cmd, _x.torque_output, _x.boo_input, _x.boo_cmd, _x.boo_output, _x.enabled, _x.override, _x.driver, _x.watchdog_counter.source, _x.watchdog_braking, _x.fault_wdc, _x.fault_ch1, _x.fault_ch2, _x.fault_boo, _x.fault_connector))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.watchdog_counter is None:
        self.watchdog_counter = dbw_mkz_msgs.msg.WatchdogCounter()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 37
      (_x.pedal_input, _x.pedal_cmd, _x.pedal_output, _x.torque_input, _x.torque_cmd, _x.torque_output, _x.boo_input, _x.boo_cmd, _x.boo_output, _x.enabled, _x.override, _x.driver, _x.watchdog_counter.source, _x.watchdog_braking, _x.fault_wdc, _x.fault_ch1, _x.fault_ch2, _x.fault_boo, _x.fault_connector,) = _get_struct_6f13B().unpack(str[start:end])
      self.boo_input = bool(self.boo_input)
      self.boo_cmd = bool(self.boo_cmd)
      self.boo_output = bool(self.boo_output)
      self.enabled = bool(self.enabled)
      self.override = bool(self.override)
      self.driver = bool(self.driver)
      self.watchdog_braking = bool(self.watchdog_braking)
      self.fault_wdc = bool(self.fault_wdc)
      self.fault_ch1 = bool(self.fault_ch1)
      self.fault_ch2 = bool(self.fault_ch2)
      self.fault_boo = bool(self.fault_boo)
      self.fault_connector = bool(self.fault_connector)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_6f13B = None
def _get_struct_6f13B():
    global _struct_6f13B
    if _struct_6f13B is None:
        _struct_6f13B = struct.Struct("<6f13B")
    return _struct_6f13B
