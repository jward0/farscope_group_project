// Generated by gencpp from file ur10_picking/PoseMessage.msg
// DO NOT EDIT!


#ifndef UR10_PICKING_MESSAGE_POSEMESSAGE_H
#define UR10_PICKING_MESSAGE_POSEMESSAGE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>

namespace ur10_picking
{
template <class ContainerAllocator>
struct PoseMessage_
{
  typedef PoseMessage_<ContainerAllocator> Type;

  PoseMessage_()
    : incremental(false)
    , pose()  {
    }
  PoseMessage_(const ContainerAllocator& _alloc)
    : incremental(false)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _incremental_type;
  _incremental_type incremental;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::ur10_picking::PoseMessage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ur10_picking::PoseMessage_<ContainerAllocator> const> ConstPtr;

}; // struct PoseMessage_

typedef ::ur10_picking::PoseMessage_<std::allocator<void> > PoseMessage;

typedef boost::shared_ptr< ::ur10_picking::PoseMessage > PoseMessagePtr;
typedef boost::shared_ptr< ::ur10_picking::PoseMessage const> PoseMessageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ur10_picking::PoseMessage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ur10_picking::PoseMessage_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ur10_picking::PoseMessage_<ContainerAllocator1> & lhs, const ::ur10_picking::PoseMessage_<ContainerAllocator2> & rhs)
{
  return lhs.incremental == rhs.incremental &&
    lhs.pose == rhs.pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ur10_picking::PoseMessage_<ContainerAllocator1> & lhs, const ::ur10_picking::PoseMessage_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ur10_picking

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ur10_picking::PoseMessage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ur10_picking::PoseMessage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur10_picking::PoseMessage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ur10_picking::PoseMessage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur10_picking::PoseMessage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ur10_picking::PoseMessage_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ur10_picking::PoseMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a78b0cf13928e95a7f4fb13c941b8e3e";
  }

  static const char* value(const ::ur10_picking::PoseMessage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa78b0cf13928e95aULL;
  static const uint64_t static_value2 = 0x7f4fb13c941b8e3eULL;
};

template<class ContainerAllocator>
struct DataType< ::ur10_picking::PoseMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ur10_picking/PoseMessage";
  }

  static const char* value(const ::ur10_picking::PoseMessage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ur10_picking::PoseMessage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool incremental\n"
"geometry_msgs/Pose pose\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::ur10_picking::PoseMessage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ur10_picking::PoseMessage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.incremental);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PoseMessage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ur10_picking::PoseMessage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ur10_picking::PoseMessage_<ContainerAllocator>& v)
  {
    s << indent << "incremental: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.incremental);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UR10_PICKING_MESSAGE_POSEMESSAGE_H
