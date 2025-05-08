// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from nuri_msgs:msg/RobotState.idl
// generated code does not contain a copyright notice
#include "nuri_msgs/msg/detail/robot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

bool
nuri_msgs__msg__RobotState__init(nuri_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    nuri_msgs__msg__RobotState__fini(msg);
    return false;
  }
  // battery
  return true;
}

void
nuri_msgs__msg__RobotState__fini(nuri_msgs__msg__RobotState * msg)
{
  if (!msg) {
    return;
  }
  // status
  rosidl_runtime_c__String__fini(&msg->status);
  // battery
}

bool
nuri_msgs__msg__RobotState__are_equal(const nuri_msgs__msg__RobotState * lhs, const nuri_msgs__msg__RobotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  // battery
  if (lhs->battery != rhs->battery) {
    return false;
  }
  return true;
}

bool
nuri_msgs__msg__RobotState__copy(
  const nuri_msgs__msg__RobotState * input,
  nuri_msgs__msg__RobotState * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  // battery
  output->battery = input->battery;
  return true;
}

nuri_msgs__msg__RobotState *
nuri_msgs__msg__RobotState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuri_msgs__msg__RobotState * msg = (nuri_msgs__msg__RobotState *)allocator.allocate(sizeof(nuri_msgs__msg__RobotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(nuri_msgs__msg__RobotState));
  bool success = nuri_msgs__msg__RobotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
nuri_msgs__msg__RobotState__destroy(nuri_msgs__msg__RobotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    nuri_msgs__msg__RobotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
nuri_msgs__msg__RobotState__Sequence__init(nuri_msgs__msg__RobotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuri_msgs__msg__RobotState * data = NULL;

  if (size) {
    data = (nuri_msgs__msg__RobotState *)allocator.zero_allocate(size, sizeof(nuri_msgs__msg__RobotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = nuri_msgs__msg__RobotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        nuri_msgs__msg__RobotState__fini(&data[i - 1]);
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
nuri_msgs__msg__RobotState__Sequence__fini(nuri_msgs__msg__RobotState__Sequence * array)
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
      nuri_msgs__msg__RobotState__fini(&array->data[i]);
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

nuri_msgs__msg__RobotState__Sequence *
nuri_msgs__msg__RobotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  nuri_msgs__msg__RobotState__Sequence * array = (nuri_msgs__msg__RobotState__Sequence *)allocator.allocate(sizeof(nuri_msgs__msg__RobotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = nuri_msgs__msg__RobotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
nuri_msgs__msg__RobotState__Sequence__destroy(nuri_msgs__msg__RobotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    nuri_msgs__msg__RobotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
nuri_msgs__msg__RobotState__Sequence__are_equal(const nuri_msgs__msg__RobotState__Sequence * lhs, const nuri_msgs__msg__RobotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!nuri_msgs__msg__RobotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
nuri_msgs__msg__RobotState__Sequence__copy(
  const nuri_msgs__msg__RobotState__Sequence * input,
  nuri_msgs__msg__RobotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(nuri_msgs__msg__RobotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    nuri_msgs__msg__RobotState * data =
      (nuri_msgs__msg__RobotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!nuri_msgs__msg__RobotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          nuri_msgs__msg__RobotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!nuri_msgs__msg__RobotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
