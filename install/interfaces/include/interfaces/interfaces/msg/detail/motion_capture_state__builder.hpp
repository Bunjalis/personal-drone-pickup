// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/MotionCaptureState.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MOTION_CAPTURE_STATE__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__MOTION_CAPTURE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/motion_capture_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_MotionCaptureState_twist
{
public:
  explicit Init_MotionCaptureState_twist(::interfaces::msg::MotionCaptureState & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::MotionCaptureState twist(::interfaces::msg::MotionCaptureState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::MotionCaptureState msg_;
};

class Init_MotionCaptureState_pose
{
public:
  explicit Init_MotionCaptureState_pose(::interfaces::msg::MotionCaptureState & msg)
  : msg_(msg)
  {}
  Init_MotionCaptureState_twist pose(::interfaces::msg::MotionCaptureState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_MotionCaptureState_twist(msg_);
  }

private:
  ::interfaces::msg::MotionCaptureState msg_;
};

class Init_MotionCaptureState_child_frame_id
{
public:
  explicit Init_MotionCaptureState_child_frame_id(::interfaces::msg::MotionCaptureState & msg)
  : msg_(msg)
  {}
  Init_MotionCaptureState_pose child_frame_id(::interfaces::msg::MotionCaptureState::_child_frame_id_type arg)
  {
    msg_.child_frame_id = std::move(arg);
    return Init_MotionCaptureState_pose(msg_);
  }

private:
  ::interfaces::msg::MotionCaptureState msg_;
};

class Init_MotionCaptureState_header
{
public:
  Init_MotionCaptureState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotionCaptureState_child_frame_id header(::interfaces::msg::MotionCaptureState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotionCaptureState_child_frame_id(msg_);
  }

private:
  ::interfaces::msg::MotionCaptureState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::MotionCaptureState>()
{
  return interfaces::msg::builder::Init_MotionCaptureState_header();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__MOTION_CAPTURE_STATE__BUILDER_HPP_
