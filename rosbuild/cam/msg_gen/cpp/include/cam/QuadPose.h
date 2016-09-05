/* Auto-generated by genmsg_cpp for file /home/ubuntu/PedroAbreu/quadcopter/rosbuild/cam/msg/QuadPose.msg */
#ifndef CAM_MESSAGE_QUADPOSE_H
#define CAM_MESSAGE_QUADPOSE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

namespace cam
{
template <class ContainerAllocator>
struct QuadPose_ {
  typedef QuadPose_<ContainerAllocator> Type;

  QuadPose_()
  : name()
  , position()
  , orientation()
  , pose_updated(0)
  {
  }

  QuadPose_(const ContainerAllocator& _alloc)
  : name(_alloc)
  , position(_alloc)
  , orientation(_alloc)
  , pose_updated(0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

  typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
   ::geometry_msgs::Point_<ContainerAllocator>  position;

  typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _orientation_type;
   ::geometry_msgs::Quaternion_<ContainerAllocator>  orientation;

  typedef int8_t _pose_updated_type;
  int8_t pose_updated;


  typedef boost::shared_ptr< ::cam::QuadPose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cam::QuadPose_<ContainerAllocator>  const> ConstPtr;
}; // struct QuadPose
typedef  ::cam::QuadPose_<std::allocator<void> > QuadPose;

typedef boost::shared_ptr< ::cam::QuadPose> QuadPosePtr;
typedef boost::shared_ptr< ::cam::QuadPose const> QuadPoseConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::cam::QuadPose_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::cam::QuadPose_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace cam

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::cam::QuadPose_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::cam::QuadPose_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::cam::QuadPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "180c6571038f982cd09da891297d01b6";
  }

  static const char* value(const  ::cam::QuadPose_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x180c6571038f982cULL;
  static const uint64_t static_value2 = 0xd09da891297d01b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::cam::QuadPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "cam/QuadPose";
  }

  static const char* value(const  ::cam::QuadPose_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::cam::QuadPose_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
geometry_msgs/Point position\n\
geometry_msgs/Quaternion orientation\n\
int8 pose_updated\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::cam::QuadPose_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::cam::QuadPose_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.position);
    stream.next(m.orientation);
    stream.next(m.pose_updated);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct QuadPose_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cam::QuadPose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::cam::QuadPose_<ContainerAllocator> & v) 
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "position: ";
s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "orientation: ";
s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.orientation);
    s << indent << "pose_updated: ";
    Printer<int8_t>::stream(s, indent + "  ", v.pose_updated);
  }
};


} // namespace message_operations
} // namespace ros

#endif // CAM_MESSAGE_QUADPOSE_H

