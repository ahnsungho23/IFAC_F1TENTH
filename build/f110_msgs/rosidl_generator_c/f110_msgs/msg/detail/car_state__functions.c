// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from f110_msgs:msg/CarState.idl
// generated code does not contain a copyright notice
#include "f110_msgs/msg/detail/car_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `slam_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `ekf_twist`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
f110_msgs__msg__CarState__init(f110_msgs__msg__CarState * msg)
{
  if (!msg) {
    return false;
  }
  // slam_pose
  if (!geometry_msgs__msg__Pose__init(&msg->slam_pose)) {
    f110_msgs__msg__CarState__fini(msg);
    return false;
  }
  // ekf_twist
  if (!geometry_msgs__msg__Twist__init(&msg->ekf_twist)) {
    f110_msgs__msg__CarState__fini(msg);
    return false;
  }
  return true;
}

void
f110_msgs__msg__CarState__fini(f110_msgs__msg__CarState * msg)
{
  if (!msg) {
    return;
  }
  // slam_pose
  geometry_msgs__msg__Pose__fini(&msg->slam_pose);
  // ekf_twist
  geometry_msgs__msg__Twist__fini(&msg->ekf_twist);
}

bool
f110_msgs__msg__CarState__are_equal(const f110_msgs__msg__CarState * lhs, const f110_msgs__msg__CarState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // slam_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->slam_pose), &(rhs->slam_pose)))
  {
    return false;
  }
  // ekf_twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->ekf_twist), &(rhs->ekf_twist)))
  {
    return false;
  }
  return true;
}

bool
f110_msgs__msg__CarState__copy(
  const f110_msgs__msg__CarState * input,
  f110_msgs__msg__CarState * output)
{
  if (!input || !output) {
    return false;
  }
  // slam_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->slam_pose), &(output->slam_pose)))
  {
    return false;
  }
  // ekf_twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->ekf_twist), &(output->ekf_twist)))
  {
    return false;
  }
  return true;
}

f110_msgs__msg__CarState *
f110_msgs__msg__CarState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__CarState * msg = (f110_msgs__msg__CarState *)allocator.allocate(sizeof(f110_msgs__msg__CarState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(f110_msgs__msg__CarState));
  bool success = f110_msgs__msg__CarState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
f110_msgs__msg__CarState__destroy(f110_msgs__msg__CarState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    f110_msgs__msg__CarState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
f110_msgs__msg__CarState__Sequence__init(f110_msgs__msg__CarState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__CarState * data = NULL;

  if (size) {
    data = (f110_msgs__msg__CarState *)allocator.zero_allocate(size, sizeof(f110_msgs__msg__CarState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = f110_msgs__msg__CarState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        f110_msgs__msg__CarState__fini(&data[i - 1]);
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
f110_msgs__msg__CarState__Sequence__fini(f110_msgs__msg__CarState__Sequence * array)
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
      f110_msgs__msg__CarState__fini(&array->data[i]);
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

f110_msgs__msg__CarState__Sequence *
f110_msgs__msg__CarState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__CarState__Sequence * array = (f110_msgs__msg__CarState__Sequence *)allocator.allocate(sizeof(f110_msgs__msg__CarState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = f110_msgs__msg__CarState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
f110_msgs__msg__CarState__Sequence__destroy(f110_msgs__msg__CarState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    f110_msgs__msg__CarState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
f110_msgs__msg__CarState__Sequence__are_equal(const f110_msgs__msg__CarState__Sequence * lhs, const f110_msgs__msg__CarState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!f110_msgs__msg__CarState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
f110_msgs__msg__CarState__Sequence__copy(
  const f110_msgs__msg__CarState__Sequence * input,
  f110_msgs__msg__CarState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(f110_msgs__msg__CarState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    f110_msgs__msg__CarState * data =
      (f110_msgs__msg__CarState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!f110_msgs__msg__CarState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          f110_msgs__msg__CarState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!f110_msgs__msg__CarState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
