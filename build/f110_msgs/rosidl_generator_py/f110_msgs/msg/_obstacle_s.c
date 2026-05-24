// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from f110_msgs:msg/Obstacle.idl
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
#include "f110_msgs/msg/detail/obstacle__struct.h"
#include "f110_msgs/msg/detail/obstacle__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool f110_msgs__msg__obstacle__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
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
    assert(strncmp("f110_msgs.msg._obstacle.Obstacle", full_classname_dest, 32) == 0);
  }
  f110_msgs__msg__Obstacle * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // s_start
    PyObject * field = PyObject_GetAttrString(_pymsg, "s_start");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->s_start = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // s_end
    PyObject * field = PyObject_GetAttrString(_pymsg, "s_end");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->s_end = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "d_right");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d_right = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "d_left");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d_left = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // is_actually_a_gap
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_actually_a_gap");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_actually_a_gap = (Py_True == field);
    Py_DECREF(field);
  }
  {  // s_center
    PyObject * field = PyObject_GetAttrString(_pymsg, "s_center");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->s_center = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d_center
    PyObject * field = PyObject_GetAttrString(_pymsg, "d_center");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d_center = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // size
    PyObject * field = PyObject_GetAttrString(_pymsg, "size");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->size = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vs
    PyObject * field = PyObject_GetAttrString(_pymsg, "vs");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vs = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vd
    PyObject * field = PyObject_GetAttrString(_pymsg, "vd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vd = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // s_var
    PyObject * field = PyObject_GetAttrString(_pymsg, "s_var");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->s_var = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d_var
    PyObject * field = PyObject_GetAttrString(_pymsg, "d_var");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d_var = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vs_var
    PyObject * field = PyObject_GetAttrString(_pymsg, "vs_var");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vs_var = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vd_var
    PyObject * field = PyObject_GetAttrString(_pymsg, "vd_var");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vd_var = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // is_static
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_static");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_static = (Py_True == field);
    Py_DECREF(field);
  }
  {  // is_visible
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_visible");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_visible = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * f110_msgs__msg__obstacle__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Obstacle */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("f110_msgs.msg._obstacle");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Obstacle");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  f110_msgs__msg__Obstacle * ros_message = (f110_msgs__msg__Obstacle *)raw_ros_message;
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
  {  // s_start
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->s_start);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s_start", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // s_end
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->s_end);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s_end", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d_right
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d_right);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d_left
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_actually_a_gap
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_actually_a_gap ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_actually_a_gap", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // s_center
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->s_center);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s_center", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d_center
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d_center);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d_center", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // size
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->size);
    {
      int rc = PyObject_SetAttrString(_pymessage, "size", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vs
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vs);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vs", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // s_var
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->s_var);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s_var", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d_var
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d_var);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d_var", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vs_var
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vs_var);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vs_var", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vd_var
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vd_var);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vd_var", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_static
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_static ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_static", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_visible
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_visible ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_visible", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
