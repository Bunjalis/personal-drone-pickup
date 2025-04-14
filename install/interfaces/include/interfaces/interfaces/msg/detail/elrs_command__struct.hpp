// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/ELRSCommand.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ELRS_COMMAND__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__ELRS_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__ELRSCommand __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__ELRSCommand __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ELRSCommand_
{
  using Type = ELRSCommand_<ContainerAllocator>;

  explicit ELRSCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->armed = false;
      this->channel_0 = 0.0f;
      this->channel_1 = 0.0f;
      this->channel_2 = 0.0f;
      this->channel_3 = 0.0f;
      this->channel_4 = 0.0f;
      this->channel_5 = 0.0f;
      this->channel_6 = 0.0f;
      this->channel_7 = 0.0f;
      this->channel_8 = 0.0f;
      this->channel_9 = 0.0f;
      this->channel_10 = 0.0f;
    }
  }

  explicit ELRSCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->armed = false;
      this->channel_0 = 0.0f;
      this->channel_1 = 0.0f;
      this->channel_2 = 0.0f;
      this->channel_3 = 0.0f;
      this->channel_4 = 0.0f;
      this->channel_5 = 0.0f;
      this->channel_6 = 0.0f;
      this->channel_7 = 0.0f;
      this->channel_8 = 0.0f;
      this->channel_9 = 0.0f;
      this->channel_10 = 0.0f;
    }
  }

  // field types and members
  using _armed_type =
    bool;
  _armed_type armed;
  using _channel_0_type =
    float;
  _channel_0_type channel_0;
  using _channel_1_type =
    float;
  _channel_1_type channel_1;
  using _channel_2_type =
    float;
  _channel_2_type channel_2;
  using _channel_3_type =
    float;
  _channel_3_type channel_3;
  using _channel_4_type =
    float;
  _channel_4_type channel_4;
  using _channel_5_type =
    float;
  _channel_5_type channel_5;
  using _channel_6_type =
    float;
  _channel_6_type channel_6;
  using _channel_7_type =
    float;
  _channel_7_type channel_7;
  using _channel_8_type =
    float;
  _channel_8_type channel_8;
  using _channel_9_type =
    float;
  _channel_9_type channel_9;
  using _channel_10_type =
    float;
  _channel_10_type channel_10;

  // setters for named parameter idiom
  Type & set__armed(
    const bool & _arg)
  {
    this->armed = _arg;
    return *this;
  }
  Type & set__channel_0(
    const float & _arg)
  {
    this->channel_0 = _arg;
    return *this;
  }
  Type & set__channel_1(
    const float & _arg)
  {
    this->channel_1 = _arg;
    return *this;
  }
  Type & set__channel_2(
    const float & _arg)
  {
    this->channel_2 = _arg;
    return *this;
  }
  Type & set__channel_3(
    const float & _arg)
  {
    this->channel_3 = _arg;
    return *this;
  }
  Type & set__channel_4(
    const float & _arg)
  {
    this->channel_4 = _arg;
    return *this;
  }
  Type & set__channel_5(
    const float & _arg)
  {
    this->channel_5 = _arg;
    return *this;
  }
  Type & set__channel_6(
    const float & _arg)
  {
    this->channel_6 = _arg;
    return *this;
  }
  Type & set__channel_7(
    const float & _arg)
  {
    this->channel_7 = _arg;
    return *this;
  }
  Type & set__channel_8(
    const float & _arg)
  {
    this->channel_8 = _arg;
    return *this;
  }
  Type & set__channel_9(
    const float & _arg)
  {
    this->channel_9 = _arg;
    return *this;
  }
  Type & set__channel_10(
    const float & _arg)
  {
    this->channel_10 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::ELRSCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::ELRSCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::ELRSCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::ELRSCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::ELRSCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::ELRSCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::ELRSCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::ELRSCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::ELRSCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::ELRSCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__ELRSCommand
    std::shared_ptr<interfaces::msg::ELRSCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__ELRSCommand
    std::shared_ptr<interfaces::msg::ELRSCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ELRSCommand_ & other) const
  {
    if (this->armed != other.armed) {
      return false;
    }
    if (this->channel_0 != other.channel_0) {
      return false;
    }
    if (this->channel_1 != other.channel_1) {
      return false;
    }
    if (this->channel_2 != other.channel_2) {
      return false;
    }
    if (this->channel_3 != other.channel_3) {
      return false;
    }
    if (this->channel_4 != other.channel_4) {
      return false;
    }
    if (this->channel_5 != other.channel_5) {
      return false;
    }
    if (this->channel_6 != other.channel_6) {
      return false;
    }
    if (this->channel_7 != other.channel_7) {
      return false;
    }
    if (this->channel_8 != other.channel_8) {
      return false;
    }
    if (this->channel_9 != other.channel_9) {
      return false;
    }
    if (this->channel_10 != other.channel_10) {
      return false;
    }
    return true;
  }
  bool operator!=(const ELRSCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ELRSCommand_

// alias to use template instance with default allocator
using ELRSCommand =
  interfaces::msg::ELRSCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ELRS_COMMAND__STRUCT_HPP_
