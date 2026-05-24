// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from f110_msgs:msg/ProjOppTraj.idl
// generated code does not contain a copyright notice
#include "f110_msgs/msg/detail/proj_opp_traj__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `detections`
#include "f110_msgs/msg/detail/proj_opp_point__functions.h"

bool
f110_msgs__msg__ProjOppTraj__init(f110_msgs__msg__ProjOppTraj * msg)
{
  if (!msg) {
    return false;
  }
  // lapcount
  // nrofpoints
  // opp_is_on_trajectory
  // detections
  if (!f110_msgs__msg__ProjOppPoint__Sequence__init(&msg->detections, 0)) {
    f110_msgs__msg__ProjOppTraj__fini(msg);
    return false;
  }
  return true;
}

void
f110_msgs__msg__ProjOppTraj__fini(f110_msgs__msg__ProjOppTraj * msg)
{
  if (!msg) {
    return;
  }
  // lapcount
  // nrofpoints
  // opp_is_on_trajectory
  // detections
  f110_msgs__msg__ProjOppPoint__Sequence__fini(&msg->detections);
}

bool
f110_msgs__msg__ProjOppTraj__are_equal(const f110_msgs__msg__ProjOppTraj * lhs, const f110_msgs__msg__ProjOppTraj * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // lapcount
  if (lhs->lapcount != rhs->lapcount) {
    return false;
  }
  // nrofpoints
  if (lhs->nrofpoints != rhs->nrofpoints) {
    return false;
  }
  // opp_is_on_trajectory
  if (lhs->opp_is_on_trajectory != rhs->opp_is_on_trajectory) {
    return false;
  }
  // detections
  if (!f110_msgs__msg__ProjOppPoint__Sequence__are_equal(
      &(lhs->detections), &(rhs->detections)))
  {
    return false;
  }
  return true;
}

bool
f110_msgs__msg__ProjOppTraj__copy(
  const f110_msgs__msg__ProjOppTraj * input,
  f110_msgs__msg__ProjOppTraj * output)
{
  if (!input || !output) {
    return false;
  }
  // lapcount
  output->lapcount = input->lapcount;
  // nrofpoints
  output->nrofpoints = input->nrofpoints;
  // opp_is_on_trajectory
  output->opp_is_on_trajectory = input->opp_is_on_trajectory;
  // detections
  if (!f110_msgs__msg__ProjOppPoint__Sequence__copy(
      &(input->detections), &(output->detections)))
  {
    return false;
  }
  return true;
}

f110_msgs__msg__ProjOppTraj *
f110_msgs__msg__ProjOppTraj__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__ProjOppTraj * msg = (f110_msgs__msg__ProjOppTraj *)allocator.allocate(sizeof(f110_msgs__msg__ProjOppTraj), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(f110_msgs__msg__ProjOppTraj));
  bool success = f110_msgs__msg__ProjOppTraj__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
f110_msgs__msg__ProjOppTraj__destroy(f110_msgs__msg__ProjOppTraj * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    f110_msgs__msg__ProjOppTraj__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
f110_msgs__msg__ProjOppTraj__Sequence__init(f110_msgs__msg__ProjOppTraj__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__ProjOppTraj * data = NULL;

  if (size) {
    data = (f110_msgs__msg__ProjOppTraj *)allocator.zero_allocate(size, sizeof(f110_msgs__msg__ProjOppTraj), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = f110_msgs__msg__ProjOppTraj__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        f110_msgs__msg__ProjOppTraj__fini(&data[i - 1]);
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
f110_msgs__msg__ProjOppTraj__Sequence__fini(f110_msgs__msg__ProjOppTraj__Sequence * array)
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
      f110_msgs__msg__ProjOppTraj__fini(&array->data[i]);
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

f110_msgs__msg__ProjOppTraj__Sequence *
f110_msgs__msg__ProjOppTraj__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__ProjOppTraj__Sequence * array = (f110_msgs__msg__ProjOppTraj__Sequence *)allocator.allocate(sizeof(f110_msgs__msg__ProjOppTraj__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = f110_msgs__msg__ProjOppTraj__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
f110_msgs__msg__ProjOppTraj__Sequence__destroy(f110_msgs__msg__ProjOppTraj__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    f110_msgs__msg__ProjOppTraj__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
f110_msgs__msg__ProjOppTraj__Sequence__are_equal(const f110_msgs__msg__ProjOppTraj__Sequence * lhs, const f110_msgs__msg__ProjOppTraj__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!f110_msgs__msg__ProjOppTraj__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
f110_msgs__msg__ProjOppTraj__Sequence__copy(
  const f110_msgs__msg__ProjOppTraj__Sequence * input,
  f110_msgs__msg__ProjOppTraj__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(f110_msgs__msg__ProjOppTraj);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    f110_msgs__msg__ProjOppTraj * data =
      (f110_msgs__msg__ProjOppTraj *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!f110_msgs__msg__ProjOppTraj__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          f110_msgs__msg__ProjOppTraj__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!f110_msgs__msg__ProjOppTraj__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
