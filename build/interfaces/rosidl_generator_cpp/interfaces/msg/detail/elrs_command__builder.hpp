// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/ELRSCommand.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ELRS_COMMAND__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__ELRS_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/elrs_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_ELRSCommand_channel_10
{
public:
  explicit Init_ELRSCommand_channel_10(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::ELRSCommand channel_10(::interfaces::msg::ELRSCommand::_channel_10_type arg)
  {
    msg_.channel_10 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_9
{
public:
  explicit Init_ELRSCommand_channel_9(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_10 channel_9(::interfaces::msg::ELRSCommand::_channel_9_type arg)
  {
    msg_.channel_9 = std::move(arg);
    return Init_ELRSCommand_channel_10(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_8
{
public:
  explicit Init_ELRSCommand_channel_8(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_9 channel_8(::interfaces::msg::ELRSCommand::_channel_8_type arg)
  {
    msg_.channel_8 = std::move(arg);
    return Init_ELRSCommand_channel_9(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_7
{
public:
  explicit Init_ELRSCommand_channel_7(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_8 channel_7(::interfaces::msg::ELRSCommand::_channel_7_type arg)
  {
    msg_.channel_7 = std::move(arg);
    return Init_ELRSCommand_channel_8(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_6
{
public:
  explicit Init_ELRSCommand_channel_6(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_7 channel_6(::interfaces::msg::ELRSCommand::_channel_6_type arg)
  {
    msg_.channel_6 = std::move(arg);
    return Init_ELRSCommand_channel_7(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_5
{
public:
  explicit Init_ELRSCommand_channel_5(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_6 channel_5(::interfaces::msg::ELRSCommand::_channel_5_type arg)
  {
    msg_.channel_5 = std::move(arg);
    return Init_ELRSCommand_channel_6(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_4
{
public:
  explicit Init_ELRSCommand_channel_4(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_5 channel_4(::interfaces::msg::ELRSCommand::_channel_4_type arg)
  {
    msg_.channel_4 = std::move(arg);
    return Init_ELRSCommand_channel_5(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_3
{
public:
  explicit Init_ELRSCommand_channel_3(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_4 channel_3(::interfaces::msg::ELRSCommand::_channel_3_type arg)
  {
    msg_.channel_3 = std::move(arg);
    return Init_ELRSCommand_channel_4(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_2
{
public:
  explicit Init_ELRSCommand_channel_2(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_3 channel_2(::interfaces::msg::ELRSCommand::_channel_2_type arg)
  {
    msg_.channel_2 = std::move(arg);
    return Init_ELRSCommand_channel_3(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_1
{
public:
  explicit Init_ELRSCommand_channel_1(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_2 channel_1(::interfaces::msg::ELRSCommand::_channel_1_type arg)
  {
    msg_.channel_1 = std::move(arg);
    return Init_ELRSCommand_channel_2(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_channel_0
{
public:
  explicit Init_ELRSCommand_channel_0(::interfaces::msg::ELRSCommand & msg)
  : msg_(msg)
  {}
  Init_ELRSCommand_channel_1 channel_0(::interfaces::msg::ELRSCommand::_channel_0_type arg)
  {
    msg_.channel_0 = std::move(arg);
    return Init_ELRSCommand_channel_1(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

class Init_ELRSCommand_armed
{
public:
  Init_ELRSCommand_armed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ELRSCommand_channel_0 armed(::interfaces::msg::ELRSCommand::_armed_type arg)
  {
    msg_.armed = std::move(arg);
    return Init_ELRSCommand_channel_0(msg_);
  }

private:
  ::interfaces::msg::ELRSCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::ELRSCommand>()
{
  return interfaces::msg::builder::Init_ELRSCommand_armed();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ELRS_COMMAND__BUILDER_HPP_
