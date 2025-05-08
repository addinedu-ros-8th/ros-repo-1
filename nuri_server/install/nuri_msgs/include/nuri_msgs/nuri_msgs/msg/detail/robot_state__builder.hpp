// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from nuri_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nuri_msgs/msg/robot_state.hpp"


#ifndef NURI_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
#define NURI_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "nuri_msgs/msg/detail/robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace nuri_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotState_battery
{
public:
  explicit Init_RobotState_battery(::nuri_msgs::msg::RobotState & msg)
  : msg_(msg)
  {}
  ::nuri_msgs::msg::RobotState battery(::nuri_msgs::msg::RobotState::_battery_type arg)
  {
    msg_.battery = std::move(arg);
    return std::move(msg_);
  }

private:
  ::nuri_msgs::msg::RobotState msg_;
};

class Init_RobotState_status
{
public:
  Init_RobotState_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotState_battery status(::nuri_msgs::msg::RobotState::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_RobotState_battery(msg_);
  }

private:
  ::nuri_msgs::msg::RobotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::nuri_msgs::msg::RobotState>()
{
  return nuri_msgs::msg::builder::Init_RobotState_status();
}

}  // namespace nuri_msgs

#endif  // NURI_MSGS__MSG__DETAIL__ROBOT_STATE__BUILDER_HPP_
