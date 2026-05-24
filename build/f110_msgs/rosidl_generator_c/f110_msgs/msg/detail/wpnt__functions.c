// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from f110_msgs:msg/Wpnt.idl
// generated code does not contain a copyright notice
#include "f110_msgs/msg/detail/wpnt__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
f110_msgs__msg__Wpnt__init(f110_msgs__msg__Wpnt * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // s_m
  // d_m
  // x_m
  // y_m
  // d_right
  // d_left
  // psi_rad
  // kappa_radpm
  // vx_mps
  // ax_mps2
  return true;
}

void
f110_msgs__msg__Wpnt__fini(f110_msgs__msg__Wpnt * msg)
{
  if (!msg) {
    return;
  }
  // id
  // s_m
  // d_m
  // x_m
  // y_m
  // d_right
  // d_left
  // psi_rad
  // kappa_radpm
  // vx_mps
  // ax_mps2
}

bool
f110_msgs__msg__Wpnt__are_equal(const f110_msgs__msg__Wpnt * lhs, const f110_msgs__msg__Wpnt * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // s_m
  if (lhs->s_m != rhs->s_m) {
    return false;
  }
  // d_m
  if (lhs->d_m != rhs->d_m) {
    return false;
  }
  // x_m
  if (lhs->x_m != rhs->x_m) {
    return false;
  }
  // y_m
  if (lhs->y_m != rhs->y_m) {
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
  // psi_rad
  if (lhs->psi_rad != rhs->psi_rad) {
    return false;
  }
  // kappa_radpm
  if (lhs->kappa_radpm != rhs->kappa_radpm) {
    return false;
  }
  // vx_mps
  if (lhs->vx_mps != rhs->vx_mps) {
    return false;
  }
  // ax_mps2
  if (lhs->ax_mps2 != rhs->ax_mps2) {
    return false;
  }
  return true;
}

bool
f110_msgs__msg__Wpnt__copy(
  const f110_msgs__msg__Wpnt * input,
  f110_msgs__msg__Wpnt * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // s_m
  output->s_m = input->s_m;
  // d_m
  output->d_m = input->d_m;
  // x_m
  output->x_m = input->x_m;
  // y_m
  output->y_m = input->y_m;
  // d_right
  output->d_right = input->d_right;
  // d_left
  output->d_left = input->d_left;
  // psi_rad
  output->psi_rad = input->psi_rad;
  // kappa_radpm
  output->kappa_radpm = input->kappa_radpm;
  // vx_mps
  output->vx_mps = input->vx_mps;
  // ax_mps2
  output->ax_mps2 = input->ax_mps2;
  return true;
}

f110_msgs__msg__Wpnt *
f110_msgs__msg__Wpnt__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__Wpnt * msg = (f110_msgs__msg__Wpnt *)allocator.allocate(sizeof(f110_msgs__msg__Wpnt), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(f110_msgs__msg__Wpnt));
  bool success = f110_msgs__msg__Wpnt__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
f110_msgs__msg__Wpnt__destroy(f110_msgs__msg__Wpnt * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    f110_msgs__msg__Wpnt__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
f110_msgs__msg__Wpnt__Sequence__init(f110_msgs__msg__Wpnt__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__Wpnt * data = NULL;

  if (size) {
    data = (f110_msgs__msg__Wpnt *)allocator.zero_allocate(size, sizeof(f110_msgs__msg__Wpnt), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = f110_msgs__msg__Wpnt__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        f110_msgs__msg__Wpnt__fini(&data[i - 1]);
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
f110_msgs__msg__Wpnt__Sequence__fini(f110_msgs__msg__Wpnt__Sequence * array)
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
      f110_msgs__msg__Wpnt__fini(&array->data[i]);
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

f110_msgs__msg__Wpnt__Sequence *
f110_msgs__msg__Wpnt__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  f110_msgs__msg__Wpnt__Sequence * array = (f110_msgs__msg__Wpnt__Sequence *)allocator.allocate(sizeof(f110_msgs__msg__Wpnt__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = f110_msgs__msg__Wpnt__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
f110_msgs__msg__Wpnt__Sequence__destroy(f110_msgs__msg__Wpnt__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    f110_msgs__msg__Wpnt__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
f110_msgs__msg__Wpnt__Sequence__are_equal(const f110_msgs__msg__Wpnt__Sequence * lhs, const f110_msgs__msg__Wpnt__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!f110_msgs__msg__Wpnt__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
f110_msgs__msg__Wpnt__Sequence__copy(
  const f110_msgs__msg__Wpnt__Sequence * input,
  f110_msgs__msg__Wpnt__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(f110_msgs__msg__Wpnt);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    f110_msgs__msg__Wpnt * data =
      (f110_msgs__msg__Wpnt *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!f110_msgs__msg__Wpnt__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          f110_msgs__msg__Wpnt__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!f110_msgs__msg__Wpnt__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
