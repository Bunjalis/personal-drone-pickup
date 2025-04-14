// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/ELRSCommand.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/elrs_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
interfaces__msg__ELRSCommand__init(interfaces__msg__ELRSCommand * msg)
{
  if (!msg) {
    return false;
  }
  // armed
  // channel_0
  // channel_1
  // channel_2
  // channel_3
  // channel_4
  // channel_5
  // channel_6
  // channel_7
  // channel_8
  // channel_9
  // channel_10
  return true;
}

void
interfaces__msg__ELRSCommand__fini(interfaces__msg__ELRSCommand * msg)
{
  if (!msg) {
    return;
  }
  // armed
  // channel_0
  // channel_1
  // channel_2
  // channel_3
  // channel_4
  // channel_5
  // channel_6
  // channel_7
  // channel_8
  // channel_9
  // channel_10
}

bool
interfaces__msg__ELRSCommand__are_equal(const interfaces__msg__ELRSCommand * lhs, const interfaces__msg__ELRSCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // armed
  if (lhs->armed != rhs->armed) {
    return false;
  }
  // channel_0
  if (lhs->channel_0 != rhs->channel_0) {
    return false;
  }
  // channel_1
  if (lhs->channel_1 != rhs->channel_1) {
    return false;
  }
  // channel_2
  if (lhs->channel_2 != rhs->channel_2) {
    return false;
  }
  // channel_3
  if (lhs->channel_3 != rhs->channel_3) {
    return false;
  }
  // channel_4
  if (lhs->channel_4 != rhs->channel_4) {
    return false;
  }
  // channel_5
  if (lhs->channel_5 != rhs->channel_5) {
    return false;
  }
  // channel_6
  if (lhs->channel_6 != rhs->channel_6) {
    return false;
  }
  // channel_7
  if (lhs->channel_7 != rhs->channel_7) {
    return false;
  }
  // channel_8
  if (lhs->channel_8 != rhs->channel_8) {
    return false;
  }
  // channel_9
  if (lhs->channel_9 != rhs->channel_9) {
    return false;
  }
  // channel_10
  if (lhs->channel_10 != rhs->channel_10) {
    return false;
  }
  return true;
}

bool
interfaces__msg__ELRSCommand__copy(
  const interfaces__msg__ELRSCommand * input,
  interfaces__msg__ELRSCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // armed
  output->armed = input->armed;
  // channel_0
  output->channel_0 = input->channel_0;
  // channel_1
  output->channel_1 = input->channel_1;
  // channel_2
  output->channel_2 = input->channel_2;
  // channel_3
  output->channel_3 = input->channel_3;
  // channel_4
  output->channel_4 = input->channel_4;
  // channel_5
  output->channel_5 = input->channel_5;
  // channel_6
  output->channel_6 = input->channel_6;
  // channel_7
  output->channel_7 = input->channel_7;
  // channel_8
  output->channel_8 = input->channel_8;
  // channel_9
  output->channel_9 = input->channel_9;
  // channel_10
  output->channel_10 = input->channel_10;
  return true;
}

interfaces__msg__ELRSCommand *
interfaces__msg__ELRSCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__ELRSCommand * msg = (interfaces__msg__ELRSCommand *)allocator.allocate(sizeof(interfaces__msg__ELRSCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__ELRSCommand));
  bool success = interfaces__msg__ELRSCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__ELRSCommand__destroy(interfaces__msg__ELRSCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__ELRSCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__ELRSCommand__Sequence__init(interfaces__msg__ELRSCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__ELRSCommand * data = NULL;

  if (size) {
    data = (interfaces__msg__ELRSCommand *)allocator.zero_allocate(size, sizeof(interfaces__msg__ELRSCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__ELRSCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__ELRSCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
interfaces__msg__ELRSCommand__Sequence__fini(interfaces__msg__ELRSCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      interfaces__msg__ELRSCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

interfaces__msg__ELRSCommand__Sequence *
interfaces__msg__ELRSCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__ELRSCommand__Sequence * array = (interfaces__msg__ELRSCommand__Sequence *)allocator.allocate(sizeof(interfaces__msg__ELRSCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__ELRSCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__ELRSCommand__Sequence__destroy(interfaces__msg__ELRSCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__ELRSCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__ELRSCommand__Sequence__are_equal(const interfaces__msg__ELRSCommand__Sequence * lhs, const interfaces__msg__ELRSCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__ELRSCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__ELRSCommand__Sequence__copy(
  const interfaces__msg__ELRSCommand__Sequence * input,
  interfaces__msg__ELRSCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__ELRSCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__ELRSCommand * data =
      (interfaces__msg__ELRSCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__ELRSCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__ELRSCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__ELRSCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
