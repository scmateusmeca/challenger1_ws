// Generated by gencpp from file nav2d_navigator/MoveToPosition2DResult.msg
// DO NOT EDIT!


#ifndef NAV2D_NAVIGATOR_MESSAGE_MOVETOPOSITION2DRESULT_H
#define NAV2D_NAVIGATOR_MESSAGE_MOVETOPOSITION2DRESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose2D.h>

namespace nav2d_navigator
{
template <class ContainerAllocator>
struct MoveToPosition2DResult_
{
  typedef MoveToPosition2DResult_<ContainerAllocator> Type;

  MoveToPosition2DResult_()
    : final_pose()
    , final_distance(0.0)  {
    }
  MoveToPosition2DResult_(const ContainerAllocator& _alloc)
    : final_pose(_alloc)
    , final_distance(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _final_pose_type;
  _final_pose_type final_pose;

   typedef float _final_distance_type;
  _final_distance_type final_distance;





  typedef boost::shared_ptr< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> const> ConstPtr;

}; // struct MoveToPosition2DResult_

typedef ::nav2d_navigator::MoveToPosition2DResult_<std::allocator<void> > MoveToPosition2DResult;

typedef boost::shared_ptr< ::nav2d_navigator::MoveToPosition2DResult > MoveToPosition2DResultPtr;
typedef boost::shared_ptr< ::nav2d_navigator::MoveToPosition2DResult const> MoveToPosition2DResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace nav2d_navigator

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav2d_navigator': ['/home/mateus/challenger1_ws/devel/share/nav2d_navigator/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1494b1c9041b641e97cee161a63a1b7b";
  }

  static const char* value(const ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1494b1c9041b641eULL;
  static const uint64_t static_value2 = 0x97cee161a63a1b7bULL;
};

template<class ContainerAllocator>
struct DataType< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav2d_navigator/MoveToPosition2DResult";
  }

  static const char* value(const ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"geometry_msgs/Pose2D final_pose\n"
"float32 final_distance\n"
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

  static const char* value(const ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.final_pose);
      stream.next(m.final_distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveToPosition2DResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nav2d_navigator::MoveToPosition2DResult_<ContainerAllocator>& v)
  {
    s << indent << "final_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.final_pose);
    s << indent << "final_distance: ";
    Printer<float>::stream(s, indent + "  ", v.final_distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAV2D_NAVIGATOR_MESSAGE_MOVETOPOSITION2DRESULT_H
