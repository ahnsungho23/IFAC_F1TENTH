// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/OpponentTrajectory.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__STRUCT_H_

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
// Member 'oppwpnts'
#include "f110_msgs/msg/detail/opp_wpnt__struct.h"

/// Struct defined in msg/OpponentTrajectory in the package f110_msgs.
typedef struct f110_msgs__msg__OpponentTrajectory
{
  std_msgs__msg__Header header;
  int32_t id;
  double lap_count;
  bool opp_is_on_trajectory;
  f110_msgs__msg__OppWpnt__Sequence oppwpnts;
} f110_msgs__msg__OpponentTrajectory;

// Struct for a sequence of f110_msgs__msg__OpponentTrajectory.
typedef struct f110_msgs__msg__OpponentTrajectory__Sequence
{
  f110_msgs__msg__OpponentTrajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__OpponentTrajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__OPPONENT_TRAJECTORY__STRUCT_H_
