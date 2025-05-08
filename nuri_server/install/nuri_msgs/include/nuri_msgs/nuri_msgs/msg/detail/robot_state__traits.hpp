// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from nuri_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nuri_msgs/msg/robot_state.hpp"


#ifndef NURI_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
#define NURI_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "nuri_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace nuri_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: battery
  {
    out << "battery: ";
    rosidl_generator_traits::value_to_yaml(msg.battery, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: battery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery: ";
    rosidl_generator_traits::value_to_yaml(msg.battery, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobotState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace nuri_msgs

namespace rosidl_generator_traits
{

[[deprecated("use nuri_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const nuri_msgs::msg::RobotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  nuri_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use nuri_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const nuri_msgs::msg::RobotState & msg)
{
  return nuri_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<nuri_msgs::msg::RobotState>()
{
  return "nuri_msgs::msg::RobotState";
}

template<>
inline const char * name<nuri_msgs::msg::RobotState>()
{
  return "nuri_msgs/msg/RobotState";
}

template<>
struct has_fixed_size<nuri_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<nuri_msgs::msg::RobotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<nuri_msgs::msg::RobotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NURI_MSGS__MSG__DETAIL__ROBOT_STATE__TRAITS_HPP_
