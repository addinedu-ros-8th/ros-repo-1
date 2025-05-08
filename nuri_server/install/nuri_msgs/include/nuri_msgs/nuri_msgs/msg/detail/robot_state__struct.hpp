// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from nuri_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "nuri_msgs/msg/robot_state.hpp"


#ifndef NURI_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
#define NURI_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__nuri_msgs__msg__RobotState __attribute__((deprecated))
#else
# define DEPRECATED__nuri_msgs__msg__RobotState __declspec(deprecated)
#endif

namespace nuri_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotState_
{
  using Type = RobotState_<ContainerAllocator>;

  explicit RobotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
      this->battery = 0l;
    }
  }

  explicit RobotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
      this->battery = 0l;
    }
  }

  // field types and members
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _battery_type =
    int32_t;
  _battery_type battery;

  // setters for named parameter idiom
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__battery(
    const int32_t & _arg)
  {
    this->battery = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    nuri_msgs::msg::RobotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const nuri_msgs::msg::RobotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<nuri_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<nuri_msgs::msg::RobotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      nuri_msgs::msg::RobotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<nuri_msgs::msg::RobotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      nuri_msgs::msg::RobotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<nuri_msgs::msg::RobotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<nuri_msgs::msg::RobotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<nuri_msgs::msg::RobotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__nuri_msgs__msg__RobotState
    std::shared_ptr<nuri_msgs::msg::RobotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__nuri_msgs__msg__RobotState
    std::shared_ptr<nuri_msgs::msg::RobotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotState_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->battery != other.battery) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotState_

// alias to use template instance with default allocator
using RobotState =
  nuri_msgs::msg::RobotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace nuri_msgs

#endif  // NURI_MSGS__MSG__DETAIL__ROBOT_STATE__STRUCT_HPP_
