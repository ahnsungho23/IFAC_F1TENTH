// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from f110_msgs:msg/LapData.idl
// generated code does not contain a copyright notice
#include "f110_msgs/msg/detail/lap_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
f110_msgs__msg__LapData__init(f110_msgs__msg__LapData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    f110_msgs__msg__LapData__fini(msg);
    return false;
  }
  // lap_count
  // lap_time
  // average_lateral_error_to_global_waypoints
  // max_lateral_error_to_global_waypoints
  return true;
}

void
f110_msgs__msg__LapData__fini(f110_msgs__msg__LapData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // lap_count
  // lap_time
  // average_lateral_error_to_global_waypoints
  // max_lateral_error_to_global_waypoints
}

bool
f110_msgs__msg__LapData__are_equal(const f110_msgs__msg__LapData * lhs, const f110_msgs__msg__LapData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // lap_count
  if (lhs->lap_count != rhs->lap_count) {
    return false;
  }
  // lap_time
  if (lhs->lap_time != rhs->lap_time) {
    return false;
  }
  // average_lateral_error_to_global_waypoints
  if (lhs->average_lateral_error_to_global_waypoints != rhs->average_lateral_error_to_global_waypoints) {
    return false;
  }
  // max_lateral_error_to_global_waypoints
  if (lhs->max_lateral_error_to_global_waypoints != rhs->max_lateral_error_to_global_waypoints) {
    return false;
  }
  return true;
}

bool
f110_msgs__msg__LapData__copy(
  const f110_msgs__msg__LapData * input,
  f110_msgs__msg__LapData * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // lap_count
  output->lap_count = input->lap_count;
  // lap_time
  output->lap_time = input->lap_time;
  // average_lateral_error_to_global_waypoints
  output->average_lateral_error_to_global_waypoints = input->average_lateral_error_to_global_waypoints;
  // max_lateral_error_to_global_waypoints
  output->max_lateral_error_to_global_waypoints = input->max_lateral_error_to_global_waypoints;
  return true;
}

f110_msgs__msg__LapData *
f110_msgs__msg__LapData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__LapData * msg = (f110_msgs__msg__LapData *)allocator.allocate(sizeof(f110_msgs__msg__LapData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(f110_msgs__msg__LapData));
  bool success = f110_msgs__msg__LapData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
f110_msgs__msg__LapData__destroy(f110_msgs__msg__LapData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    f110_msgs__msg__LapData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
f110_msgs__msg__LapData__Sequence__init(f110_msgs__msg__LapData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__LapData * data = NULL;

  if (size) {
    data = (f110_msgs__msg__LapData *)allocator.zero_allocate(size, sizeof(f110_msgs__msg__LapData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = f110_msgs__msg__LapData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        f110_msgs__msg__LapData__fini(&data[i - 1]);
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
f110_msgs__msg__LapData__Sequence__fini(f110_msgs__msg__LapData__Sequence * array)
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
      f110_msgs__msg__LapData__fini(&array->data[i]);
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

f110_msgs__msg__LapData__Sequence *
f110_msgs__msg__LapData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__LapData__Sequence * array = (f110_msgs__msg__LapData__Sequence *)allocator.allocate(sizeof(f110_msgs__msg__LapData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = f110_msgs__msg__LapData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
f110_msgs__msg__LapData__Sequence__destroy(f110_msgs__msg__LapData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    f110_msgs__msg__LapData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
f110_msgs__msg__LapData__Sequence__are_equal(const f110_msgs__msg__LapData__Sequence * lhs, const f110_msgs__msg__LapData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!f110_msgs__msg__LapData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
f110_msgs__msg__LapData__Sequence__copy(
  const f110_msgs__msg__LapData__Sequence * input,
  f110_msgs__msg__LapData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(f110_msgs__msg__LapData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    f110_msgs__msg__LapData * data =
      (f110_msgs__msg__LapData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!f110_msgs__msg__LapData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          f110_msgs__msg__LapData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!f110_msgs__msg__LapData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
