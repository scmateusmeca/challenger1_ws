// Generated by gencpp from file nav2d_navigator/LocalizeFeedback.msg
// DO NOT EDIT!


#ifndef NAV2D_NAVIGATOR_MESSAGE_LOCALIZEFEEDBACK_H
#define NAV2D_NAVIGATOR_MESSAGE_LOCALIZEFEEDBACK_H


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
struct LocalizeFeedback_
{
  typedef LocalizeFeedback_<ContainerAllocator> Type;

  LocalizeFeedback_()
    {
    }
  LocalizeFeedback_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct LocalizeFeedback_

typedef ::nav2d_navigator::LocalizeFeedback_<std::allocator<void> > LocalizeFeedback;

typedef boost::shared_ptr< ::nav2d_navigator::LocalizeFeedback > LocalizeFeedbackPtr;
typedef boost::shared_ptr< ::nav2d_navigator::LocalizeFeedback const> LocalizeFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace nav2d_navigator

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "nav2d_navigator/LocalizeFeedback";
  }

  static const char* value(const ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
;
  }

  static const char* value(const ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocalizeFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::nav2d_navigator::LocalizeFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // NAV2D_NAVIGATOR_MESSAGE_LOCALIZEFEEDBACK_H
