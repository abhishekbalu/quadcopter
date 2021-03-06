# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from height/full_pose.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import height.msg
import std_msgs.msg

class full_pose(genpy.Message):
  _md5sum = "a3ec521dd8d5d73781ebeb348bf58cdb"
  _type = "height/full_pose"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
geometry_msgs/Vector3 ang_vel
geometry_msgs/Vector3 lin_acc
Attitude attitude
float64 z
float64 x
float64 y
float64 vx
float64 vy
int8 quality

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: height/Attitude
float64 yaw
float64 pitch
float64 roll
"""
  __slots__ = ['header','ang_vel','lin_acc','attitude','z','x','y','vx','vy','quality']
  _slot_types = ['std_msgs/Header','geometry_msgs/Vector3','geometry_msgs/Vector3','height/Attitude','float64','float64','float64','float64','float64','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,ang_vel,lin_acc,attitude,z,x,y,vx,vy,quality

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(full_pose, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.ang_vel is None:
        self.ang_vel = geometry_msgs.msg.Vector3()
      if self.lin_acc is None:
        self.lin_acc = geometry_msgs.msg.Vector3()
      if self.attitude is None:
        self.attitude = height.msg.Attitude()
      if self.z is None:
        self.z = 0.
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.vx is None:
        self.vx = 0.
      if self.vy is None:
        self.vy = 0.
      if self.quality is None:
        self.quality = 0
    else:
      self.header = std_msgs.msg.Header()
      self.ang_vel = geometry_msgs.msg.Vector3()
      self.lin_acc = geometry_msgs.msg.Vector3()
      self.attitude = height.msg.Attitude()
      self.z = 0.
      self.x = 0.
      self.y = 0.
      self.vx = 0.
      self.vy = 0.
      self.quality = 0

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_14db.pack(_x.ang_vel.x, _x.ang_vel.y, _x.ang_vel.z, _x.lin_acc.x, _x.lin_acc.y, _x.lin_acc.z, _x.attitude.yaw, _x.attitude.pitch, _x.attitude.roll, _x.z, _x.x, _x.y, _x.vx, _x.vy, _x.quality))
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
      if self.ang_vel is None:
        self.ang_vel = geometry_msgs.msg.Vector3()
      if self.lin_acc is None:
        self.lin_acc = geometry_msgs.msg.Vector3()
      if self.attitude is None:
        self.attitude = height.msg.Attitude()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
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
      end += 113
      (_x.ang_vel.x, _x.ang_vel.y, _x.ang_vel.z, _x.lin_acc.x, _x.lin_acc.y, _x.lin_acc.z, _x.attitude.yaw, _x.attitude.pitch, _x.attitude.roll, _x.z, _x.x, _x.y, _x.vx, _x.vy, _x.quality,) = _struct_14db.unpack(str[start:end])
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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_14db.pack(_x.ang_vel.x, _x.ang_vel.y, _x.ang_vel.z, _x.lin_acc.x, _x.lin_acc.y, _x.lin_acc.z, _x.attitude.yaw, _x.attitude.pitch, _x.attitude.roll, _x.z, _x.x, _x.y, _x.vx, _x.vy, _x.quality))
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
      if self.ang_vel is None:
        self.ang_vel = geometry_msgs.msg.Vector3()
      if self.lin_acc is None:
        self.lin_acc = geometry_msgs.msg.Vector3()
      if self.attitude is None:
        self.attitude = height.msg.Attitude()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
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
      end += 113
      (_x.ang_vel.x, _x.ang_vel.y, _x.ang_vel.z, _x.lin_acc.x, _x.lin_acc.y, _x.lin_acc.z, _x.attitude.yaw, _x.attitude.pitch, _x.attitude.roll, _x.z, _x.x, _x.y, _x.vx, _x.vy, _x.quality,) = _struct_14db.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_14db = struct.Struct("<14db")
