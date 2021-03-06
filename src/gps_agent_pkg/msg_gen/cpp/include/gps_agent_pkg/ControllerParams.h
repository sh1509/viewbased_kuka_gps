/* Auto-generated by genmsg_cpp for file /home/suneet/gps/src/gps_agent_pkg/msg/ControllerParams.msg */
#ifndef GPS_AGENT_PKG_MESSAGE_CONTROLLERPARAMS_H
#define GPS_AGENT_PKG_MESSAGE_CONTROLLERPARAMS_H
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

#include "gps_agent_pkg/CaffeParams.h"
#include "gps_agent_pkg/LinGaussParams.h"
#include "gps_agent_pkg/TfParams.h"

namespace gps_agent_pkg
{
template <class ContainerAllocator>
struct ControllerParams_ {
  typedef ControllerParams_<ContainerAllocator> Type;

  ControllerParams_()
  : controller_to_execute(0)
  , caffe()
  , lingauss()
  , tf()
  {
  }

  ControllerParams_(const ContainerAllocator& _alloc)
  : controller_to_execute(0)
  , caffe(_alloc)
  , lingauss(_alloc)
  , tf(_alloc)
  {
  }

  typedef int8_t _controller_to_execute_type;
  int8_t controller_to_execute;

  typedef  ::gps_agent_pkg::CaffeParams_<ContainerAllocator>  _caffe_type;
   ::gps_agent_pkg::CaffeParams_<ContainerAllocator>  caffe;

  typedef  ::gps_agent_pkg::LinGaussParams_<ContainerAllocator>  _lingauss_type;
   ::gps_agent_pkg::LinGaussParams_<ContainerAllocator>  lingauss;

  typedef  ::gps_agent_pkg::TfParams_<ContainerAllocator>  _tf_type;
   ::gps_agent_pkg::TfParams_<ContainerAllocator>  tf;


  typedef boost::shared_ptr< ::gps_agent_pkg::ControllerParams_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_agent_pkg::ControllerParams_<ContainerAllocator>  const> ConstPtr;
}; // struct ControllerParams
typedef  ::gps_agent_pkg::ControllerParams_<std::allocator<void> > ControllerParams;

typedef boost::shared_ptr< ::gps_agent_pkg::ControllerParams> ControllerParamsPtr;
typedef boost::shared_ptr< ::gps_agent_pkg::ControllerParams const> ControllerParamsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::gps_agent_pkg::ControllerParams_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::gps_agent_pkg::ControllerParams_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace gps_agent_pkg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::ControllerParams_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::ControllerParams_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::gps_agent_pkg::ControllerParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b21eed6449b84a548fab33a89f3b3c3b";
  }

  static const char* value(const  ::gps_agent_pkg::ControllerParams_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb21eed6449b84a54ULL;
  static const uint64_t static_value2 = 0x8fab33a89f3b3c3bULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_agent_pkg::ControllerParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "gps_agent_pkg/ControllerParams";
  }

  static const char* value(const  ::gps_agent_pkg::ControllerParams_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::gps_agent_pkg::ControllerParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 controller_to_execute  # controller enum, defined in gps_pb2\n\
\n\
CaffeParams caffe\n\
LinGaussParams lingauss\n\
TfParams tf\n\
\n\
================================================================================\n\
MSG: gps_agent_pkg/CaffeParams\n\
string net_param # Serialized net parameter with weights (equivalent of prototxt file)\n\
float32[] bias\n\
float32[] scale\n\
float32[] noise\n\
int32 dim_bias\n\
uint32 dU\n\
\n\
================================================================================\n\
MSG: gps_agent_pkg/LinGaussParams\n\
# Time-varying Linear Gaussian controller\n\
uint32 dX\n\
uint32 dU\n\
float64[] K_t  # Should be T x Du x Dx\n\
float64[] k_t  # Should by T x Du\n\
\n\
================================================================================\n\
MSG: gps_agent_pkg/TfParams\n\
# Tf Params. just need to track dU.\n\
uint32 dU\n\
\n\
\n\
";
  }

  static const char* value(const  ::gps_agent_pkg::ControllerParams_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::gps_agent_pkg::ControllerParams_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.controller_to_execute);
    stream.next(m.caffe);
    stream.next(m.lingauss);
    stream.next(m.tf);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct ControllerParams_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_agent_pkg::ControllerParams_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::gps_agent_pkg::ControllerParams_<ContainerAllocator> & v) 
  {
    s << indent << "controller_to_execute: ";
    Printer<int8_t>::stream(s, indent + "  ", v.controller_to_execute);
    s << indent << "caffe: ";
s << std::endl;
    Printer< ::gps_agent_pkg::CaffeParams_<ContainerAllocator> >::stream(s, indent + "  ", v.caffe);
    s << indent << "lingauss: ";
s << std::endl;
    Printer< ::gps_agent_pkg::LinGaussParams_<ContainerAllocator> >::stream(s, indent + "  ", v.lingauss);
    s << indent << "tf: ";
s << std::endl;
    Printer< ::gps_agent_pkg::TfParams_<ContainerAllocator> >::stream(s, indent + "  ", v.tf);
  }
};


} // namespace message_operations
} // namespace ros

#endif // GPS_AGENT_PKG_MESSAGE_CONTROLLERPARAMS_H

