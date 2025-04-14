// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/ELRSCommand.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ELRS_COMMAND__STRUCT_H_
#define INTERFACES__MSG__DETAIL__ELRS_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ELRSCommand in the package interfaces.
typedef struct interfaces__msg__ELRSCommand
{
  bool armed;
  float channel_0;
  float channel_1;
  float channel_2;
  float channel_3;
  float channel_4;
  float channel_5;
  float channel_6;
  float channel_7;
  float channel_8;
  float channel_9;
  float channel_10;
} interfaces__msg__ELRSCommand;

// Struct for a sequence of interfaces__msg__ELRSCommand.
typedef struct interfaces__msg__ELRSCommand__Sequence
{
  interfaces__msg__ELRSCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__ELRSCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__ELRS_COMMAND__STRUCT_H_
