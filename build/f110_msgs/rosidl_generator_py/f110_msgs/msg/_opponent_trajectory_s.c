// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from f110_msgs:msg/OpponentTrajectory.idl
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
#include "f110_msgs/msg/detail/opponent_trajectory__struct.h"
#include "f110_msgs/msg/detail/opponent_trajectory__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "f110_msgs/msg/detail/opp_wpnt__functions.h"
// end nested array functions include
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool f110_msgs__msg__opp_wpnt__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * f110_msgs__msg__opp_wpnt__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool f110_msgs__msg__opponent_trajectory__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
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
    assert(strncmp("f110_msgs.msg._opponent_trajectory.OpponentTrajectory", full_classname_dest, 53) == 0);
  }
  f110_msgs__msg__OpponentTrajectory * ros_message = _ros_message;
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
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // lap_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "lap_count");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->lap_count = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // opp_is_on_trajectory
    PyObject * field = PyObject_GetAttrString(_pymsg, "opp_is_on_trajectory");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->opp_is_on_trajectory = (Py_True == field);
    Py_DECREF(field);
  }
  {  // oppwpnts
    PyObject * field = PyObject_GetAttrString(_pymsg, "oppwpnts");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'oppwpnts'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!f110_msgs__msg__OppWpnt__Sequence__init(&(ros_message->oppwpnts), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create f110_msgs__msg__OppWpnt__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    f110_msgs__msg__OppWpnt * dest = ros_message->oppwpnts.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!f110_msgs__msg__opp_wpnt__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * f110_msgs__msg__opponent_trajectory__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OpponentTrajectory */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("f110_msgs.msg._opponent_trajectory");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OpponentTrajectory");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  f110_msgs__msg__OpponentTrajectory * ros_message = (f110_msgs__msg__OpponentTrajectory *)raw_ros_message;
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
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lap_count
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->lap_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "lap_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // opp_is_on_trajectory
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->opp_is_on_trajectory ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "opp_is_on_trajectory", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // oppwpnts
    PyObject * field = NULL;
    size_t size = ros_message->oppwpnts.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    f110_msgs__msg__OppWpnt * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->oppwpnts.data[i]);
      PyObject * pyitem = f110_msgs__msg__opp_wpnt__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "oppwpnts", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
