// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/ProjOppTraj.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'detections'
#include "f110_msgs/msg/detail/proj_opp_point__struct.h"

/// Struct defined in msg/ProjOppTraj in the package f110_msgs.
typedef struct f110_msgs__msg__ProjOppTraj
{
  double lapcount;
  double nrofpoints;
  bool opp_is_on_trajectory;
  f110_msgs__msg__ProjOppPoint__Sequence detections;
} f110_msgs__msg__ProjOppTraj;

// Struct for a sequence of f110_msgs__msg__ProjOppTraj.
typedef struct f110_msgs__msg__ProjOppTraj__Sequence
{
  f110_msgs__msg__ProjOppTraj * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__ProjOppTraj__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__PROJ_OPP_TRAJ__STRUCT_H_
