// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/CarState.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__CAR_STATE__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__CAR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'slam_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'ekf_twist'
#include "geometry_msgs/msg/detail/twist__struct.h"

/// Struct defined in msg/CarState in the package f110_msgs.
typedef struct f110_msgs__msg__CarState
{
  geometry_msgs__msg__Pose slam_pose;
  geometry_msgs__msg__Twist ekf_twist;
} f110_msgs__msg__CarState;

// Struct for a sequence of f110_msgs__msg__CarState.
typedef struct f110_msgs__msg__CarState__Sequence
{
  f110_msgs__msg__CarState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__CarState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__CAR_STATE__STRUCT_H_
