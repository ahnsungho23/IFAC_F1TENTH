// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from f110_msgs:msg/PidData.idl
// generated code does not contain a copyright notice

#ifndef F110_MSGS__MSG__DETAIL__PID_DATA__FUNCTIONS_H_
#define F110_MSGS__MSG__DETAIL__PID_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "f110_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "f110_msgs/msg/detail/pid_data__struct.h"

/// Initialize msg/PidData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * f110_msgs__msg__PidData
 * )) before or use
 * f110_msgs__msg__PidData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
bool
f110_msgs__msg__PidData__init(f110_msgs__msg__PidData * msg);

/// Finalize msg/PidData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
void
f110_msgs__msg__PidData__fini(f110_msgs__msg__PidData * msg);

/// Create msg/PidData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * f110_msgs__msg__PidData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
f110_msgs__msg__PidData *
f110_msgs__msg__PidData__create();

/// Destroy msg/PidData message.
/**
 * It calls
 * f110_msgs__msg__PidData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
void
f110_msgs__msg__PidData__destroy(f110_msgs__msg__PidData * msg);

/// Check for msg/PidData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
bool
f110_msgs__msg__PidData__are_equal(const f110_msgs__msg__PidData * lhs, const f110_msgs__msg__PidData * rhs);

/// Copy a msg/PidData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
bool
f110_msgs__msg__PidData__copy(
  const f110_msgs__msg__PidData * input,
  f110_msgs__msg__PidData * output);

/// Initialize array of msg/PidData messages.
/**
 * It allocates the memory for the number of elements and calls
 * f110_msgs__msg__PidData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
bool
f110_msgs__msg__PidData__Sequence__init(f110_msgs__msg__PidData__Sequence * array, size_t size);

/// Finalize array of msg/PidData messages.
/**
 * It calls
 * f110_msgs__msg__PidData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
void
f110_msgs__msg__PidData__Sequence__fini(f110_msgs__msg__PidData__Sequence * array);

/// Create array of msg/PidData messages.
/**
 * It allocates the memory for the array and calls
 * f110_msgs__msg__PidData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
f110_msgs__msg__PidData__Sequence *
f110_msgs__msg__PidData__Sequence__create(size_t size);

/// Destroy array of msg/PidData messages.
/**
 * It calls
 * f110_msgs__msg__PidData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
void
f110_msgs__msg__PidData__Sequence__destroy(f110_msgs__msg__PidData__Sequence * array);

/// Check for msg/PidData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
bool
f110_msgs__msg__PidData__Sequence__are_equal(const f110_msgs__msg__PidData__Sequence * lhs, const f110_msgs__msg__PidData__Sequence * rhs);

/// Copy an array of msg/PidData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_f110_msgs
bool
f110_msgs__msg__PidData__Sequence__copy(
  const f110_msgs__msg__PidData__Sequence * input,
  f110_msgs__msg__PidData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // F110_MSGS__MSG__DETAIL__PID_DATA__FUNCTIONS_H_
