# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from nav2d_navigator/ExploreFeedback.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class ExploreFeedback(genpy.Message):
  _md5sum = "e64a606b3357bbb098996ab9c2799a9f"
  _type = "nav2d_navigator/ExploreFeedback"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
geometry_msgs/Pose2D robot_pose
geometry_msgs/Pose2D target_pose
float32 distance


================================================================================
MSG: geometry_msgs/Pose2D
# Deprecated
# Please use the full 3D pose.

# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.

# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.


# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
"""
  __slots__ = ['robot_pose','target_pose','distance']
  _slot_types = ['geometry_msgs/Pose2D','geometry_msgs/Pose2D','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       robot_pose,target_pose,distance

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ExploreFeedback, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.robot_pose is None:
        self.robot_pose = geometry_msgs.msg.Pose2D()
      if self.target_pose is None:
        self.target_pose = geometry_msgs.msg.Pose2D()
      if self.distance is None:
        self.distance = 0.
    else:
      self.robot_pose = geometry_msgs.msg.Pose2D()
      self.target_pose = geometry_msgs.msg.Pose2D()
      self.distance = 0.

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
      buff.write(_get_struct_6df().pack(_x.robot_pose.x, _x.robot_pose.y, _x.robot_pose.theta, _x.target_pose.x, _x.target_pose.y, _x.target_pose.theta, _x.distance))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.robot_pose is None:
        self.robot_pose = geometry_msgs.msg.Pose2D()
      if self.target_pose is None:
        self.target_pose = geometry_msgs.msg.Pose2D()
      end = 0
      _x = self
      start = end
      end += 52
      (_x.robot_pose.x, _x.robot_pose.y, _x.robot_pose.theta, _x.target_pose.x, _x.target_pose.y, _x.target_pose.theta, _x.distance,) = _get_struct_6df().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6df().pack(_x.robot_pose.x, _x.robot_pose.y, _x.robot_pose.theta, _x.target_pose.x, _x.target_pose.y, _x.target_pose.theta, _x.distance))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.robot_pose is None:
        self.robot_pose = geometry_msgs.msg.Pose2D()
      if self.target_pose is None:
        self.target_pose = geometry_msgs.msg.Pose2D()
      end = 0
      _x = self
      start = end
      end += 52
      (_x.robot_pose.x, _x.robot_pose.y, _x.robot_pose.theta, _x.target_pose.x, _x.target_pose.y, _x.target_pose.theta, _x.distance,) = _get_struct_6df().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6df = None
def _get_struct_6df():
    global _struct_6df
    if _struct_6df is None:
        _struct_6df = struct.Struct("<6df")
    return _struct_6df
