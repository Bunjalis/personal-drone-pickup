// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from interfaces:msg/ELRSCommand.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/elrs_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interfaces/msg/detail/elrs_command__struct.h"
#include "interfaces/msg/detail/elrs_command__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ELRSCommand__ros_msg_type = interfaces__msg__ELRSCommand;

static bool _ELRSCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ELRSCommand__ros_msg_type * ros_message = static_cast<const _ELRSCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: armed
  {
    cdr << (ros_message->armed ? true : false);
  }

  // Field name: channel_0
  {
    cdr << ros_message->channel_0;
  }

  // Field name: channel_1
  {
    cdr << ros_message->channel_1;
  }

  // Field name: channel_2
  {
    cdr << ros_message->channel_2;
  }

  // Field name: channel_3
  {
    cdr << ros_message->channel_3;
  }

  // Field name: channel_4
  {
    cdr << ros_message->channel_4;
  }

  // Field name: channel_5
  {
    cdr << ros_message->channel_5;
  }

  // Field name: channel_6
  {
    cdr << ros_message->channel_6;
  }

  // Field name: channel_7
  {
    cdr << ros_message->channel_7;
  }

  // Field name: channel_8
  {
    cdr << ros_message->channel_8;
  }

  // Field name: channel_9
  {
    cdr << ros_message->channel_9;
  }

  // Field name: channel_10
  {
    cdr << ros_message->channel_10;
  }

  return true;
}

static bool _ELRSCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ELRSCommand__ros_msg_type * ros_message = static_cast<_ELRSCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: armed
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->armed = tmp ? true : false;
  }

  // Field name: channel_0
  {
    cdr >> ros_message->channel_0;
  }

  // Field name: channel_1
  {
    cdr >> ros_message->channel_1;
  }

  // Field name: channel_2
  {
    cdr >> ros_message->channel_2;
  }

  // Field name: channel_3
  {
    cdr >> ros_message->channel_3;
  }

  // Field name: channel_4
  {
    cdr >> ros_message->channel_4;
  }

  // Field name: channel_5
  {
    cdr >> ros_message->channel_5;
  }

  // Field name: channel_6
  {
    cdr >> ros_message->channel_6;
  }

  // Field name: channel_7
  {
    cdr >> ros_message->channel_7;
  }

  // Field name: channel_8
  {
    cdr >> ros_message->channel_8;
  }

  // Field name: channel_9
  {
    cdr >> ros_message->channel_9;
  }

  // Field name: channel_10
  {
    cdr >> ros_message->channel_10;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interfaces
size_t get_serialized_size_interfaces__msg__ELRSCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ELRSCommand__ros_msg_type * ros_message = static_cast<const _ELRSCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name armed
  {
    size_t item_size = sizeof(ros_message->armed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_0
  {
    size_t item_size = sizeof(ros_message->channel_0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_1
  {
    size_t item_size = sizeof(ros_message->channel_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_2
  {
    size_t item_size = sizeof(ros_message->channel_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_3
  {
    size_t item_size = sizeof(ros_message->channel_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_4
  {
    size_t item_size = sizeof(ros_message->channel_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_5
  {
    size_t item_size = sizeof(ros_message->channel_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_6
  {
    size_t item_size = sizeof(ros_message->channel_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_7
  {
    size_t item_size = sizeof(ros_message->channel_7);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_8
  {
    size_t item_size = sizeof(ros_message->channel_8);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_9
  {
    size_t item_size = sizeof(ros_message->channel_9);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name channel_10
  {
    size_t item_size = sizeof(ros_message->channel_10);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ELRSCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interfaces__msg__ELRSCommand(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interfaces
size_t max_serialized_size_interfaces__msg__ELRSCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: armed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: channel_0
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_7
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_8
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_9
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: channel_10
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = interfaces__msg__ELRSCommand;
    is_plain =
      (
      offsetof(DataType, channel_10) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ELRSCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_interfaces__msg__ELRSCommand(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ELRSCommand = {
  "interfaces::msg",
  "ELRSCommand",
  _ELRSCommand__cdr_serialize,
  _ELRSCommand__cdr_deserialize,
  _ELRSCommand__get_serialized_size,
  _ELRSCommand__max_serialized_size
};

static rosidl_message_type_support_t _ELRSCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ELRSCommand,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interfaces, msg, ELRSCommand)() {
  return &_ELRSCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
