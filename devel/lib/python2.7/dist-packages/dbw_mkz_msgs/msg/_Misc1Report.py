# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dbw_mkz_msgs/Misc1Report.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import dbw_mkz_msgs.msg
import std_msgs.msg

class Misc1Report(genpy.Message):
  _md5sum = "9ecd16fb81815b3e46e0550feea1da2f"
  _type = "dbw_mkz_msgs/Misc1Report"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

# Turn Signal enumeration
TurnSignal turn_signal

# High beams
bool high_beam_headlights

# Front Windshield Wipers enumeration
Wiper wiper

# Ambient Light Sensor enumeration
AmbientLight ambient_light

# Buttons
bool btn_cc_on        # Cruise Control On
bool btn_cc_off       # Cruise Control Off
bool btn_cc_on_off    # Cruise Control On/Off Toggle
bool btn_cc_res       # Cruise Control Resume
bool btn_cc_cncl      # Cruise Control Cancel
bool btn_cc_res_cncl  # Cruise Control Resume/Cancel
bool btn_cc_set_inc   # Cruise Control Set+
bool btn_cc_set_dec   # Cruise Control Set-
bool btn_cc_gap_inc   # Cruise Control Gap+
bool btn_cc_gap_dec   # Cruise Control Gap-
bool btn_la_on_off    # Lane Assist On/Off Toggle

# Faults
bool fault_bus

# Doors
bool door_driver
bool door_passenger
bool door_rear_left
bool door_rear_right
bool door_hood
bool door_trunk

# Passenger seat
bool passenger_detect
bool passenger_airbag

# Seat Belts
bool buckle_driver
bool buckle_passenger

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
MSG: dbw_mkz_msgs/TurnSignal
uint8 value

uint8 NONE=0
uint8 LEFT=1
uint8 RIGHT=2

================================================================================
MSG: dbw_mkz_msgs/Wiper
uint8 status

uint8 OFF=0
uint8 AUTO_OFF=1
uint8 OFF_MOVING=2
uint8 MANUAL_OFF=3
uint8 MANUAL_ON=4
uint8 MANUAL_LOW=5
uint8 MANUAL_HIGH=6
uint8 MIST_FLICK=7
uint8 WASH=8
uint8 AUTO_LOW=9
uint8 AUTO_HIGH=10
uint8 COURTESYWIPE=11
uint8 AUTO_ADJUST=12
uint8 RESERVED=13
uint8 STALLED=14
uint8 NO_DATA=15

================================================================================
MSG: dbw_mkz_msgs/AmbientLight
uint8 status

uint8 DARK=0
uint8 LIGHT=1
uint8 TWILIGHT=2
uint8 TUNNEL_ON=3
uint8 TUNNEL_OFF=4
uint8 NO_DATA=7
"""
  __slots__ = ['header','turn_signal','high_beam_headlights','wiper','ambient_light','btn_cc_on','btn_cc_off','btn_cc_on_off','btn_cc_res','btn_cc_cncl','btn_cc_res_cncl','btn_cc_set_inc','btn_cc_set_dec','btn_cc_gap_inc','btn_cc_gap_dec','btn_la_on_off','fault_bus','door_driver','door_passenger','door_rear_left','door_rear_right','door_hood','door_trunk','passenger_detect','passenger_airbag','buckle_driver','buckle_passenger']
  _slot_types = ['std_msgs/Header','dbw_mkz_msgs/TurnSignal','bool','dbw_mkz_msgs/Wiper','dbw_mkz_msgs/AmbientLight','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,turn_signal,high_beam_headlights,wiper,ambient_light,btn_cc_on,btn_cc_off,btn_cc_on_off,btn_cc_res,btn_cc_cncl,btn_cc_res_cncl,btn_cc_set_inc,btn_cc_set_dec,btn_cc_gap_inc,btn_cc_gap_dec,btn_la_on_off,fault_bus,door_driver,door_passenger,door_rear_left,door_rear_right,door_hood,door_trunk,passenger_detect,passenger_airbag,buckle_driver,buckle_passenger

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Misc1Report, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.turn_signal is None:
        self.turn_signal = dbw_mkz_msgs.msg.TurnSignal()
      if self.high_beam_headlights is None:
        self.high_beam_headlights = False
      if self.wiper is None:
        self.wiper = dbw_mkz_msgs.msg.Wiper()
      if self.ambient_light is None:
        self.ambient_light = dbw_mkz_msgs.msg.AmbientLight()
      if self.btn_cc_on is None:
        self.btn_cc_on = False
      if self.btn_cc_off is None:
        self.btn_cc_off = False
      if self.btn_cc_on_off is None:
        self.btn_cc_on_off = False
      if self.btn_cc_res is None:
        self.btn_cc_res = False
      if self.btn_cc_cncl is None:
        self.btn_cc_cncl = False
      if self.btn_cc_res_cncl is None:
        self.btn_cc_res_cncl = False
      if self.btn_cc_set_inc is None:
        self.btn_cc_set_inc = False
      if self.btn_cc_set_dec is None:
        self.btn_cc_set_dec = False
      if self.btn_cc_gap_inc is None:
        self.btn_cc_gap_inc = False
      if self.btn_cc_gap_dec is None:
        self.btn_cc_gap_dec = False
      if self.btn_la_on_off is None:
        self.btn_la_on_off = False
      if self.fault_bus is None:
        self.fault_bus = False
      if self.door_driver is None:
        self.door_driver = False
      if self.door_passenger is None:
        self.door_passenger = False
      if self.door_rear_left is None:
        self.door_rear_left = False
      if self.door_rear_right is None:
        self.door_rear_right = False
      if self.door_hood is None:
        self.door_hood = False
      if self.door_trunk is None:
        self.door_trunk = False
      if self.passenger_detect is None:
        self.passenger_detect = False
      if self.passenger_airbag is None:
        self.passenger_airbag = False
      if self.buckle_driver is None:
        self.buckle_driver = False
      if self.buckle_passenger is None:
        self.buckle_passenger = False
    else:
      self.header = std_msgs.msg.Header()
      self.turn_signal = dbw_mkz_msgs.msg.TurnSignal()
      self.high_beam_headlights = False
      self.wiper = dbw_mkz_msgs.msg.Wiper()
      self.ambient_light = dbw_mkz_msgs.msg.AmbientLight()
      self.btn_cc_on = False
      self.btn_cc_off = False
      self.btn_cc_on_off = False
      self.btn_cc_res = False
      self.btn_cc_cncl = False
      self.btn_cc_res_cncl = False
      self.btn_cc_set_inc = False
      self.btn_cc_set_dec = False
      self.btn_cc_gap_inc = False
      self.btn_cc_gap_dec = False
      self.btn_la_on_off = False
      self.fault_bus = False
      self.door_driver = False
      self.door_passenger = False
      self.door_rear_left = False
      self.door_rear_right = False
      self.door_hood = False
      self.door_trunk = False
      self.passenger_detect = False
      self.passenger_airbag = False
      self.buckle_driver = False
      self.buckle_passenger = False

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
      buff.write(_get_struct_26B().pack(_x.turn_signal.value, _x.high_beam_headlights, _x.wiper.status, _x.ambient_light.status, _x.btn_cc_on, _x.btn_cc_off, _x.btn_cc_on_off, _x.btn_cc_res, _x.btn_cc_cncl, _x.btn_cc_res_cncl, _x.btn_cc_set_inc, _x.btn_cc_set_dec, _x.btn_cc_gap_inc, _x.btn_cc_gap_dec, _x.btn_la_on_off, _x.fault_bus, _x.door_driver, _x.door_passenger, _x.door_rear_left, _x.door_rear_right, _x.door_hood, _x.door_trunk, _x.passenger_detect, _x.passenger_airbag, _x.buckle_driver, _x.buckle_passenger))
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
      if self.turn_signal is None:
        self.turn_signal = dbw_mkz_msgs.msg.TurnSignal()
      if self.wiper is None:
        self.wiper = dbw_mkz_msgs.msg.Wiper()
      if self.ambient_light is None:
        self.ambient_light = dbw_mkz_msgs.msg.AmbientLight()
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
      end += 26
      (_x.turn_signal.value, _x.high_beam_headlights, _x.wiper.status, _x.ambient_light.status, _x.btn_cc_on, _x.btn_cc_off, _x.btn_cc_on_off, _x.btn_cc_res, _x.btn_cc_cncl, _x.btn_cc_res_cncl, _x.btn_cc_set_inc, _x.btn_cc_set_dec, _x.btn_cc_gap_inc, _x.btn_cc_gap_dec, _x.btn_la_on_off, _x.fault_bus, _x.door_driver, _x.door_passenger, _x.door_rear_left, _x.door_rear_right, _x.door_hood, _x.door_trunk, _x.passenger_detect, _x.passenger_airbag, _x.buckle_driver, _x.buckle_passenger,) = _get_struct_26B().unpack(str[start:end])
      self.high_beam_headlights = bool(self.high_beam_headlights)
      self.btn_cc_on = bool(self.btn_cc_on)
      self.btn_cc_off = bool(self.btn_cc_off)
      self.btn_cc_on_off = bool(self.btn_cc_on_off)
      self.btn_cc_res = bool(self.btn_cc_res)
      self.btn_cc_cncl = bool(self.btn_cc_cncl)
      self.btn_cc_res_cncl = bool(self.btn_cc_res_cncl)
      self.btn_cc_set_inc = bool(self.btn_cc_set_inc)
      self.btn_cc_set_dec = bool(self.btn_cc_set_dec)
      self.btn_cc_gap_inc = bool(self.btn_cc_gap_inc)
      self.btn_cc_gap_dec = bool(self.btn_cc_gap_dec)
      self.btn_la_on_off = bool(self.btn_la_on_off)
      self.fault_bus = bool(self.fault_bus)
      self.door_driver = bool(self.door_driver)
      self.door_passenger = bool(self.door_passenger)
      self.door_rear_left = bool(self.door_rear_left)
      self.door_rear_right = bool(self.door_rear_right)
      self.door_hood = bool(self.door_hood)
      self.door_trunk = bool(self.door_trunk)
      self.passenger_detect = bool(self.passenger_detect)
      self.passenger_airbag = bool(self.passenger_airbag)
      self.buckle_driver = bool(self.buckle_driver)
      self.buckle_passenger = bool(self.buckle_passenger)
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
      buff.write(_get_struct_26B().pack(_x.turn_signal.value, _x.high_beam_headlights, _x.wiper.status, _x.ambient_light.status, _x.btn_cc_on, _x.btn_cc_off, _x.btn_cc_on_off, _x.btn_cc_res, _x.btn_cc_cncl, _x.btn_cc_res_cncl, _x.btn_cc_set_inc, _x.btn_cc_set_dec, _x.btn_cc_gap_inc, _x.btn_cc_gap_dec, _x.btn_la_on_off, _x.fault_bus, _x.door_driver, _x.door_passenger, _x.door_rear_left, _x.door_rear_right, _x.door_hood, _x.door_trunk, _x.passenger_detect, _x.passenger_airbag, _x.buckle_driver, _x.buckle_passenger))
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
      if self.turn_signal is None:
        self.turn_signal = dbw_mkz_msgs.msg.TurnSignal()
      if self.wiper is None:
        self.wiper = dbw_mkz_msgs.msg.Wiper()
      if self.ambient_light is None:
        self.ambient_light = dbw_mkz_msgs.msg.AmbientLight()
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
      end += 26
      (_x.turn_signal.value, _x.high_beam_headlights, _x.wiper.status, _x.ambient_light.status, _x.btn_cc_on, _x.btn_cc_off, _x.btn_cc_on_off, _x.btn_cc_res, _x.btn_cc_cncl, _x.btn_cc_res_cncl, _x.btn_cc_set_inc, _x.btn_cc_set_dec, _x.btn_cc_gap_inc, _x.btn_cc_gap_dec, _x.btn_la_on_off, _x.fault_bus, _x.door_driver, _x.door_passenger, _x.door_rear_left, _x.door_rear_right, _x.door_hood, _x.door_trunk, _x.passenger_detect, _x.passenger_airbag, _x.buckle_driver, _x.buckle_passenger,) = _get_struct_26B().unpack(str[start:end])
      self.high_beam_headlights = bool(self.high_beam_headlights)
      self.btn_cc_on = bool(self.btn_cc_on)
      self.btn_cc_off = bool(self.btn_cc_off)
      self.btn_cc_on_off = bool(self.btn_cc_on_off)
      self.btn_cc_res = bool(self.btn_cc_res)
      self.btn_cc_cncl = bool(self.btn_cc_cncl)
      self.btn_cc_res_cncl = bool(self.btn_cc_res_cncl)
      self.btn_cc_set_inc = bool(self.btn_cc_set_inc)
      self.btn_cc_set_dec = bool(self.btn_cc_set_dec)
      self.btn_cc_gap_inc = bool(self.btn_cc_gap_inc)
      self.btn_cc_gap_dec = bool(self.btn_cc_gap_dec)
      self.btn_la_on_off = bool(self.btn_la_on_off)
      self.fault_bus = bool(self.fault_bus)
      self.door_driver = bool(self.door_driver)
      self.door_passenger = bool(self.door_passenger)
      self.door_rear_left = bool(self.door_rear_left)
      self.door_rear_right = bool(self.door_rear_right)
      self.door_hood = bool(self.door_hood)
      self.door_trunk = bool(self.door_trunk)
      self.passenger_detect = bool(self.passenger_detect)
      self.passenger_airbag = bool(self.passenger_airbag)
      self.buckle_driver = bool(self.buckle_driver)
      self.buckle_passenger = bool(self.buckle_passenger)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_26B = None
def _get_struct_26B():
    global _struct_26B
    if _struct_26B is None:
        _struct_26B = struct.Struct("<26B")
    return _struct_26B
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
