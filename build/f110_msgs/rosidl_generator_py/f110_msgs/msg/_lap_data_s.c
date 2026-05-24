// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from f110_msgs:msg/LapData.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "f110_msgs/msg/detail/lap_data__struct.h"
#include "f110_msgs/msg/detail/lap_data__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool f110_msgs__msg__lap_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[32];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("f110_msgs.msg._lap_data.LapData", full_classname_dest, 31) == 0);
  }
  f110_msgs__msg__LapData * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // lap_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "lap_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->lap_count = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // lap_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "lap_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lap_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // average_lateral_error_to_global_waypoints
    PyObject * field = PyObject_GetAttrString(_pymsg, "average_lateral_error_to_global_waypoints");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->average_lateral_error_to_global_waypoints = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_lateral_error_to_global_waypoints
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_lateral_error_to_global_waypoints");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_lateral_error_to_global_waypoints = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * f110_msgs__msg__lap_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LapData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("f110_msgs.msg._lap_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LapData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  f110_msgs__msg__LapData * ros_message = (f110_msgs__msg__LapData *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lap_count
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->lap_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lap_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lap_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lap_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lap_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // average_lateral_error_to_global_waypoints
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->average_lateral_error_to_global_waypoints);
    {
      int rc = PyObject_SetAttrString(_pymessage, "average_lateral_error_to_global_waypoints", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_lateral_error_to_global_waypoints
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_lateral_error_to_global_waypoints);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_lateral_error_to_global_waypoints", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
