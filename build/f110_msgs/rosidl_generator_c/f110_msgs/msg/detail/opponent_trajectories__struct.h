// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/OpponentTrajectories.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORIES__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORIES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'trajectories'
#include "f110_msgs/msg/detail/opponent_trajectory__struct.h"

/// Struct defined in msg/OpponentTrajectories in the package f110_msgs.
typedef struct f110_msgs__msg__OpponentTrajectories
{
  std_msgs__msg__Header header;
  f110_msgs__msg__OpponentTrajectory__Sequence trajectories;
} f110_msgs__msg__OpponentTrajectories;

// Struct for a sequence of f110_msgs__msg__OpponentTrajectories.
typedef struct f110_msgs__msg__OpponentTrajectories__Sequence
{
  f110_msgs__msg__OpponentTrajectories * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__OpponentTrajectories__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORIES__STRUCT_H_
