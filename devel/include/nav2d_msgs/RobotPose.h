// Generated by gencpp from file nav2d_msgs/RobotPose.msg
// DO NOT EDIT!


#ifndef NAV2D_MSGS_MESSAGE_ROBOTPOSE_H
#define NAV2D_MSGS_MESSAGE_ROBOTPOSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose2D.h>

namespace nav2d_msgs
{
template <class ContainerAllocator>
struct RobotPose_
{
  typedef RobotPose_<ContainerAllocator> Type;

  RobotPose_()
    : header()
    , robot_id(0)
    , pose()  {
    }
  RobotPose_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , robot_id(0)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _robot_id_type;
  _robot_id_type robot_id;

   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::nav2d_msgs::RobotPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav2d_msgs::RobotPose_<ContainerAllocator> const> ConstPtr;

}; // struct RobotPose_

typedef ::nav2d_msgs::RobotPose_<std::allocator<void> > RobotPose;

typedef boost::shared_ptr< ::nav2d_msgs::RobotPose > RobotPosePtr;
typedef boost::shared_ptr< ::nav2d_msgs::RobotPose const> RobotPoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav2d_msgs::RobotPose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nav2d_msgs::RobotPose_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::nav2d_msgs::RobotPose_<ContainerAllocator1> & lhs, const ::nav2d_msgs::RobotPose_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.robot_id == rhs.robot_id &&
    lhs.pose == rhs.pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::nav2d_msgs::RobotPose_<ContainerAllocator1> & lhs, const ::nav2d_msgs::RobotPose_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace nav2d_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::nav2d_msgs::RobotPose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav2d_msgs::RobotPose_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_msgs::RobotPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_msgs::RobotPose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_msgs::RobotPose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_msgs::RobotPose_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav2d_msgs::RobotPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "da85cb23bda44bed5435973e99adc0ea";
  }

  static const char* value(const ::nav2d_msgs::RobotPose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xda85cb23bda44bedULL;
  static const uint64_t static_value2 = 0x5435973e99adc0eaULL;
};

template<class ContainerAllocator>
struct DataType< ::nav2d_msgs::RobotPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav2d_msgs/RobotPose";
  }

  static const char* value(const ::nav2d_msgs::RobotPose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav2d_msgs::RobotPose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"int32 robot_id\n"
"geometry_msgs/Pose2D pose\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose2D\n"
"# Deprecated\n"
"# Please use the full 3D pose.\n"
"\n"
"# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n"
"\n"
"# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n"
"\n"
"\n"
"# This expresses a position and orientation on a 2D manifold.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
;
  }

  static const char* value(const ::nav2d_msgs::RobotPose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav2d_msgs::RobotPose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.robot_id);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotPose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav2d_msgs::RobotPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nav2d_msgs::RobotPose_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "robot_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.robot_id);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAV2D_MSGS_MESSAGE_ROBOTPOSE_H
