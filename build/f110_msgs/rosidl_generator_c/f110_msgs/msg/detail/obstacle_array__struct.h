// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from f110_msgs:msg/ObstacleArray.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_
#define F110_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_

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
// Member 'obstacles'
#include "f110_msgs/msg/detail/obstacle__struct.h"

/// Struct defined in msg/ObstacleArray in the package f110_msgs.
typedef struct f110_msgs__msg__ObstacleArray
{
  std_msgs__msg__Header header;
  f110_msgs__msg__Obstacle__Sequence obstacles;
} f110_msgs__msg__ObstacleArray;

// Struct for a sequence of f110_msgs__msg__ObstacleArray.
typedef struct f110_msgs__msg__ObstacleArray__Sequence
{
  f110_msgs__msg__ObstacleArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} f110_msgs__msg__ObstacleArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__OBSTACLE_ARRAY__STRUCT_H_
