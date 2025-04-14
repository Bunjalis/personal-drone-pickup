// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/ELRSCommand.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ELRS_COMMAND__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__ELRS_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/elrs_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ELRSCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: armed
  {
    out << "armed: ";
    rosidl_generator_traits::value_to_yaml(msg.armed, out);
    out << ", ";
  }

  // member: channel_0
  {
    out << "channel_0: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_0, out);
    out << ", ";
  }

  // member: channel_1
  {
    out << "channel_1: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_1, out);
    out << ", ";
  }

  // member: channel_2
  {
    out << "channel_2: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_2, out);
    out << ", ";
  }

  // member: channel_3
  {
    out << "channel_3: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_3, out);
    out << ", ";
  }

  // member: channel_4
  {
    out << "channel_4: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_4, out);
    out << ", ";
  }

  // member: channel_5
  {
    out << "channel_5: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_5, out);
    out << ", ";
  }

  // member: channel_6
  {
    out << "channel_6: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_6, out);
    out << ", ";
  }

  // member: channel_7
  {
    out << "channel_7: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_7, out);
    out << ", ";
  }

  // member: channel_8
  {
    out << "channel_8: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_8, out);
    out << ", ";
  }

  // member: channel_9
  {
    out << "channel_9: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_9, out);
    out << ", ";
  }

  // member: channel_10
  {
    out << "channel_10: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_10, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ELRSCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: armed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "armed: ";
    rosidl_generator_traits::value_to_yaml(msg.armed, out);
    out << "\n";
  }

  // member: channel_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_0: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_0, out);
    out << "\n";
  }

  // member: channel_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_1: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_1, out);
    out << "\n";
  }

  // member: channel_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_2: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_2, out);
    out << "\n";
  }

  // member: channel_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_3: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_3, out);
    out << "\n";
  }

  // member: channel_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_4: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_4, out);
    out << "\n";
  }

  // member: channel_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_5: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_5, out);
    out << "\n";
  }

  // member: channel_6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_6: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_6, out);
    out << "\n";
  }

  // member: channel_7
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_7: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_7, out);
    out << "\n";
  }

  // member: channel_8
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_8: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_8, out);
    out << "\n";
  }

  // member: channel_9
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_9: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_9, out);
    out << "\n";
  }

  // member: channel_10
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "channel_10: ";
    rosidl_generator_traits::value_to_yaml(msg.channel_10, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ELRSCommand & msg, bool use_flow_style = false)
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

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::ELRSCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::ELRSCommand & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::ELRSCommand>()
{
  return "interfaces::msg::ELRSCommand";
}

template<>
inline const char * name<interfaces::msg::ELRSCommand>()
{
  return "interfaces/msg/ELRSCommand";
}

template<>
struct has_fixed_size<interfaces::msg::ELRSCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::msg::ELRSCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::msg::ELRSCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__ELRS_COMMAND__TRAITS_HPP_
