// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from f110_msgs:msg/Obstacle.idl
// generated code does not contain a copyright notice
#include "f110_msgs/msg/detail/obstacle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
f110_msgs__msg__Obstacle__init(f110_msgs__msg__Obstacle * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // s_start
  // s_end
  // d_right
  // d_left
  // is_actually_a_gap
  // s_center
  // d_center
  // size
  // vs
  // vd
  // s_var
  // d_var
  // vs_var
  // vd_var
  // is_static
  // is_visible
  return true;
}

void
f110_msgs__msg__Obstacle__fini(f110_msgs__msg__Obstacle * msg)
{
  if (!msg) {
    return;
  }
  // id
  // s_start
  // s_end
  // d_right
  // d_left
  // is_actually_a_gap
  // s_center
  // d_center
  // size
  // vs
  // vd
  // s_var
  // d_var
  // vs_var
  // vd_var
  // is_static
  // is_visible
}

bool
f110_msgs__msg__Obstacle__are_equal(const f110_msgs__msg__Obstacle * lhs, const f110_msgs__msg__Obstacle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // s_start
  if (lhs->s_start != rhs->s_start) {
    return false;
  }
  // s_end
  if (lhs->s_end != rhs->s_end) {
    return false;
  }
  // d_right
  if (lhs->d_right != rhs->d_right) {
    return false;
  }
  // d_left
  if (lhs->d_left != rhs->d_left) {
    return false;
  }
  // is_actually_a_gap
  if (lhs->is_actually_a_gap != rhs->is_actually_a_gap) {
    return false;
  }
  // s_center
  if (lhs->s_center != rhs->s_center) {
    return false;
  }
  // d_center
  if (lhs->d_center != rhs->d_center) {
    return false;
  }
  // size
  if (lhs->size != rhs->size) {
    return false;
  }
  // vs
  if (lhs->vs != rhs->vs) {
    return false;
  }
  // vd
  if (lhs->vd != rhs->vd) {
    return false;
  }
  // s_var
  if (lhs->s_var != rhs->s_var) {
    return false;
  }
  // d_var
  if (lhs->d_var != rhs->d_var) {
    return false;
  }
  // vs_var
  if (lhs->vs_var != rhs->vs_var) {
    return false;
  }
  // vd_var
  if (lhs->vd_var != rhs->vd_var) {
    return false;
  }
  // is_static
  if (lhs->is_static != rhs->is_static) {
    return false;
  }
  // is_visible
  if (lhs->is_visible != rhs->is_visible) {
    return false;
  }
  return true;
}

bool
f110_msgs__msg__Obstacle__copy(
  const f110_msgs__msg__Obstacle * input,
  f110_msgs__msg__Obstacle * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // s_start
  output->s_start = input->s_start;
  // s_end
  output->s_end = input->s_end;
  // d_right
  output->d_right = input->d_right;
  // d_left
  output->d_left = input->d_left;
  // is_actually_a_gap
  output->is_actually_a_gap = input->is_actually_a_gap;
  // s_center
  output->s_center = input->s_center;
  // d_center
  output->d_center = input->d_center;
  // size
  output->size = input->size;
  // vs
  output->vs = input->vs;
  // vd
  output->vd = input->vd;
  // s_var
  output->s_var = input->s_var;
  // d_var
  output->d_var = input->d_var;
  // vs_var
  output->vs_var = input->vs_var;
  // vd_var
  output->vd_var = input->vd_var;
  // is_static
  output->is_static = input->is_static;
  // is_visible
  output->is_visible = input->is_visible;
  return true;
}

f110_msgs__msg__Obstacle *
f110_msgs__msg__Obstacle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__Obstacle * msg = (f110_msgs__msg__Obstacle *)allocator.allocate(sizeof(f110_msgs__msg__Obstacle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(f110_msgs__msg__Obstacle));
  bool success = f110_msgs__msg__Obstacle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
f110_msgs__msg__Obstacle__destroy(f110_msgs__msg__Obstacle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    f110_msgs__msg__Obstacle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
f110_msgs__msg__Obstacle__Sequence__init(f110_msgs__msg__Obstacle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__Obstacle * data = NULL;

  if (size) {
    data = (f110_msgs__msg__Obstacle *)allocator.zero_allocate(size, sizeof(f110_msgs__msg__Obstacle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = f110_msgs__msg__Obstacle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        f110_msgs__msg__Obstacle__fini(&data[i - 1]);
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
f110_msgs__msg__Obstacle__Sequence__fini(f110_msgs__msg__Obstacle__Sequence * array)
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
      f110_msgs__msg__Obstacle__fini(&array->data[i]);
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

f110_msgs__msg__Obstacle__Sequence *
f110_msgs__msg__Obstacle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__Obstacle__Sequence * array = (f110_msgs__msg__Obstacle__Sequence *)allocator.allocate(sizeof(f110_msgs__msg__Obstacle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = f110_msgs__msg__Obstacle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
f110_msgs__msg__Obstacle__Sequence__destroy(f110_msgs__msg__Obstacle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    f110_msgs__msg__Obstacle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
f110_msgs__msg__Obstacle__Sequence__are_equal(const f110_msgs__msg__Obstacle__Sequence * lhs, const f110_msgs__msg__Obstacle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!f110_msgs__msg__Obstacle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
f110_msgs__msg__Obstacle__Sequence__copy(
  const f110_msgs__msg__Obstacle__Sequence * input,
  f110_msgs__msg__Obstacle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(f110_msgs__msg__Obstacle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    f110_msgs__msg__Obstacle * data =
      (f110_msgs__msg__Obstacle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!f110_msgs__msg__Obstacle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          f110_msgs__msg__Obstacle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!f110_msgs__msg__Obstacle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
