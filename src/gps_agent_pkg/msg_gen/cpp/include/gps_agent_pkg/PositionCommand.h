/* Auto-generated by genmsg_cpp for file /home/suneet/gps/src/gps_agent_pkg/msg/PositionCommand.msg */
#ifndef GPS_AGENT_PKG_MESSAGE_POSITIONCOMMAND_H
#define GPS_AGENT_PKG_MESSAGE_POSITIONCOMMAND_H
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


namespace gps_agent_pkg
{
template <class ContainerAllocator>
struct PositionCommand_ {
  typedef PositionCommand_<ContainerAllocator> Type;

  PositionCommand_()
  : id(0)
  , mode(0)
  , arm(0)
  , data()
  , pd_gains()
  {
  }

  PositionCommand_(const ContainerAllocator& _alloc)
  : id(0)
  , mode(0)
  , arm(0)
  , data(_alloc)
  , pd_gains(_alloc)
  {
  }

  typedef int32_t _id_type;
  int32_t id;

  typedef int8_t _mode_type;
  int8_t mode;

  typedef int8_t _arm_type;
  int8_t arm;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _data_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  data;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _pd_gains_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  pd_gains;


  typedef boost::shared_ptr< ::gps_agent_pkg::PositionCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_agent_pkg::PositionCommand_<ContainerAllocator>  const> ConstPtr;
}; // struct PositionCommand
typedef  ::gps_agent_pkg::PositionCommand_<std::allocator<void> > PositionCommand;

typedef boost::shared_ptr< ::gps_agent_pkg::PositionCommand> PositionCommandPtr;
typedef boost::shared_ptr< ::gps_agent_pkg::PositionCommand const> PositionCommandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::gps_agent_pkg::PositionCommand_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::gps_agent_pkg::PositionCommand_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace gps_agent_pkg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::PositionCommand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::PositionCommand_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::gps_agent_pkg::PositionCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "941c91297ac4d198ac50657705c8471d";
  }

  static const char* value(const  ::gps_agent_pkg::PositionCommand_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x941c91297ac4d198ULL;
  static const uint64_t static_value2 = 0xac50657705c8471dULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_agent_pkg::PositionCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "gps_agent_pkg/PositionCommand";
  }

  static const char* value(const  ::gps_agent_pkg::PositionCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::gps_agent_pkg::PositionCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 id  # ID must be echoed back in SampleResult\n\
int8 mode  # control mode (i.e. joint space, task space), enum in gps_pb2\n\
int8 arm  # which arm to execute on (actuator type enum in gps_pb2)\n\
float64[] data\n\
float64[] pd_gains\n\
\n\
";
  }

  static const char* value(const  ::gps_agent_pkg::PositionCommand_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::gps_agent_pkg::PositionCommand_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.id);
    stream.next(m.mode);
    stream.next(m.arm);
    stream.next(m.data);
    stream.next(m.pd_gains);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct PositionCommand_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_agent_pkg::PositionCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::gps_agent_pkg::PositionCommand_<ContainerAllocator> & v) 
  {
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.mode);
    s << indent << "arm: ";
    Printer<int8_t>::stream(s, indent + "  ", v.arm);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.data[i]);
    }
    s << indent << "pd_gains[]" << std::endl;
    for (size_t i = 0; i < v.pd_gains.size(); ++i)
    {
      s << indent << "  pd_gains[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.pd_gains[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // GPS_AGENT_PKG_MESSAGE_POSITIONCOMMAND_H

