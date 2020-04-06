// Generated by gencpp from file dataspeed_can_msgs/CanMessageStamped.msg
// DO NOT EDIT!


#ifndef DATASPEED_CAN_MSGS_MESSAGE_CANMESSAGESTAMPED_H
#define DATASPEED_CAN_MSGS_MESSAGE_CANMESSAGESTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <dataspeed_can_msgs/CanMessage.h>

namespace dataspeed_can_msgs
{
template <class ContainerAllocator>
struct CanMessageStamped_
{
  typedef CanMessageStamped_<ContainerAllocator> Type;

  CanMessageStamped_()
    : header()
    , msg()  {
    }
  CanMessageStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , msg(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::dataspeed_can_msgs::CanMessage_<ContainerAllocator>  _msg_type;
  _msg_type msg;





  typedef boost::shared_ptr< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> const> ConstPtr;

}; // struct CanMessageStamped_

typedef ::dataspeed_can_msgs::CanMessageStamped_<std::allocator<void> > CanMessageStamped;

typedef boost::shared_ptr< ::dataspeed_can_msgs::CanMessageStamped > CanMessageStampedPtr;
typedef boost::shared_ptr< ::dataspeed_can_msgs::CanMessageStamped const> CanMessageStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dataspeed_can_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'dataspeed_can_msgs': ['/home/kartik/Documents/gazebo_practice_ws/src/autonomous_vehicle/dataspeed_can/dataspeed_can_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "33747cb98e223cafb806d7e94cb4071f";
  }

  static const char* value(const ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x33747cb98e223cafULL;
  static const uint64_t static_value2 = 0xb806d7e94cb4071fULL;
};

template<class ContainerAllocator>
struct DataType< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dataspeed_can_msgs/CanMessageStamped";
  }

  static const char* value(const ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"CanMessage msg\n"
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
"MSG: dataspeed_can_msgs/CanMessage\n"
"uint8[8] data\n"
"uint32 id\n"
"bool extended\n"
"uint8 dlc\n"
;
  }

  static const char* value(const ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CanMessageStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dataspeed_can_msgs::CanMessageStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "msg: ";
    s << std::endl;
    Printer< ::dataspeed_can_msgs::CanMessage_<ContainerAllocator> >::stream(s, indent + "  ", v.msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DATASPEED_CAN_MSGS_MESSAGE_CANMESSAGESTAMPED_H