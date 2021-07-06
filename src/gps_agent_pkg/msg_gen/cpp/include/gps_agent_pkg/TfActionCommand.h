/* Auto-generated by genmsg_cpp for file /home/suneet/gps/src/gps_agent_pkg/msg/TfActionCommand.msg */
#ifndef GPS_AGENT_PKG_MESSAGE_TFACTIONCOMMAND_H
#define GPS_AGENT_PKG_MESSAGE_TFACTIONCOMMAND_H
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
struct TfActionCommand_ {
  typedef TfActionCommand_<ContainerAllocator> Type;

  TfActionCommand_()
  : action()
  , dU(0)
  , id(0)
  {
  }

  TfActionCommand_(const ContainerAllocator& _alloc)
  : action(_alloc)
  , dU(0)
  , id(0)
  {
  }

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _action_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  action;

  typedef int32_t _dU_type;
  int32_t dU;

  typedef int32_t _id_type;
  int32_t id;


  typedef boost::shared_ptr< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator>  const> ConstPtr;
}; // struct TfActionCommand
typedef  ::gps_agent_pkg::TfActionCommand_<std::allocator<void> > TfActionCommand;

typedef boost::shared_ptr< ::gps_agent_pkg::TfActionCommand> TfActionCommandPtr;
typedef boost::shared_ptr< ::gps_agent_pkg::TfActionCommand const> TfActionCommandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace gps_agent_pkg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f309f95a21bb20b690ca72e110126ceb";
  }

  static const char* value(const  ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf309f95a21bb20b6ULL;
  static const uint64_t static_value2 = 0x90ca72e110126cebULL;
};

template<class ContainerAllocator>
struct DataType< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "gps_agent_pkg/TfActionCommand";
  }

  static const char* value(const  ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64[] action\n\
int32 dU\n\
int32 id\n\
\n\
";
  }

  static const char* value(const  ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.action);
    stream.next(m.dU);
    stream.next(m.id);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct TfActionCommand_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::gps_agent_pkg::TfActionCommand_<ContainerAllocator> & v) 
  {
    s << indent << "action[]" << std::endl;
    for (size_t i = 0; i < v.action.size(); ++i)
    {
      s << indent << "  action[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.action[i]);
    }
    s << indent << "dU: ";
    Printer<int32_t>::stream(s, indent + "  ", v.dU);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
  }
};


} // namespace message_operations
} // namespace ros

#endif // GPS_AGENT_PKG_MESSAGE_TFACTIONCOMMAND_H
