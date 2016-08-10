/* Auto-generated by genmsg_cpp for file /home/ubuntu/quadcopter/ros/laser/msg/EstimateSingle.msg */
#ifndef LASER_MESSAGE_ESTIMATESINGLE_H
#define LASER_MESSAGE_ESTIMATESINGLE_H
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

#include "std_msgs/Header.h"
#include "laser/Estimate.h"

namespace laser
{
template <class ContainerAllocator>
struct EstimateSingle_ {
  typedef EstimateSingle_<ContainerAllocator> Type;

  EstimateSingle_()
  : header()
  , estimate()
  {
  }

  EstimateSingle_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , estimate(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::laser::Estimate_<ContainerAllocator>  _estimate_type;
   ::laser::Estimate_<ContainerAllocator>  estimate;


  typedef boost::shared_ptr< ::laser::EstimateSingle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::laser::EstimateSingle_<ContainerAllocator>  const> ConstPtr;
}; // struct EstimateSingle
typedef  ::laser::EstimateSingle_<std::allocator<void> > EstimateSingle;

typedef boost::shared_ptr< ::laser::EstimateSingle> EstimateSinglePtr;
typedef boost::shared_ptr< ::laser::EstimateSingle const> EstimateSingleConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::laser::EstimateSingle_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::laser::EstimateSingle_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace laser

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::laser::EstimateSingle_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::laser::EstimateSingle_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::laser::EstimateSingle_<ContainerAllocator> > {
  static const char* value() 
  {
    return "aa91808c4d02906ac29fbe291106ae05";
  }

  static const char* value(const  ::laser::EstimateSingle_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xaa91808c4d02906aULL;
  static const uint64_t static_value2 = 0xc29fbe291106ae05ULL;
};

template<class ContainerAllocator>
struct DataType< ::laser::EstimateSingle_<ContainerAllocator> > {
  static const char* value() 
  {
    return "laser/EstimateSingle";
  }

  static const char* value(const  ::laser::EstimateSingle_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::laser::EstimateSingle_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
Estimate estimate\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: laser/Estimate\n\
std_msgs/String             name\n\
geometry_msgs/Vector3       position\n\
geometry_msgs/Vector3       velocity\n\
geometry_msgs/Vector3       perturbation\n\
geometry_msgs/Quaternion    orientation\n\
int8                        updated\n\
float64[]                   covariance\n\
std_msgs/String             sensors\n\
\n\
================================================================================\n\
MSG: std_msgs/String\n\
string data\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
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

  static const char* value(const  ::laser::EstimateSingle_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::laser::EstimateSingle_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::laser::EstimateSingle_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::laser::EstimateSingle_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.estimate);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct EstimateSingle_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::laser::EstimateSingle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::laser::EstimateSingle_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "estimate: ";
s << std::endl;
    Printer< ::laser::Estimate_<ContainerAllocator> >::stream(s, indent + "  ", v.estimate);
  }
};


} // namespace message_operations
} // namespace ros

#endif // LASER_MESSAGE_ESTIMATESINGLE_H
