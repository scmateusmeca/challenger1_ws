// Generated by gencpp from file nav2d_navigator/MoveToPosition2DFeedback.msg
// DO NOT EDIT!


#ifndef NAV2D_NAVIGATOR_MESSAGE_MOVETOPOSITION2DFEEDBACK_H
#define NAV2D_NAVIGATOR_MESSAGE_MOVETOPOSITION2DFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace nav2d_navigator
{
template <class ContainerAllocator>
struct MoveToPosition2DFeedback_
{
  typedef MoveToPosition2DFeedback_<ContainerAllocator> Type;

  MoveToPosition2DFeedback_()
    : distance(0.0)  {
    }
  MoveToPosition2DFeedback_(const ContainerAllocator& _alloc)
    : distance(0.0)  {
  (void)_alloc;
    }



   typedef float _distance_type;
  _distance_type distance;





  typedef boost::shared_ptr< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct MoveToPosition2DFeedback_

typedef ::nav2d_navigator::MoveToPosition2DFeedback_<std::allocator<void> > MoveToPosition2DFeedback;

typedef boost::shared_ptr< ::nav2d_navigator::MoveToPosition2DFeedback > MoveToPosition2DFeedbackPtr;
typedef boost::shared_ptr< ::nav2d_navigator::MoveToPosition2DFeedback const> MoveToPosition2DFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6e77fb10f0c8b4833ec273aa9ac74459";
  }

  static const char* value(const ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6e77fb10f0c8b483ULL;
  static const uint64_t static_value2 = 0x3ec273aa9ac74459ULL;
};

template<class ContainerAllocator>
struct DataType< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav2d_navigator/MoveToPosition2DFeedback";
  }

  static const char* value(const ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"float32 distance\n"
"\n"
"\n"
;
  }

  static const char* value(const ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.distance);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveToPosition2DFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::nav2d_navigator::MoveToPosition2DFeedback_<ContainerAllocator>& v)
  {
    s << indent << "distance: ";
    Printer<float>::stream(s, indent + "  ", v.distance);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAV2D_NAVIGATOR_MESSAGE_MOVETOPOSITION2DFEEDBACK_H