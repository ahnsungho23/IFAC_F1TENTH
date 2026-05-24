// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from f110_msgs:msg/OppWpnt.idl
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
#include "f110_msgs/msg/detail/opp_wpnt__struct.h"
#include "f110_msgs/msg/detail/opp_wpnt__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool f110_msgs__msg__opp_wpnt__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("f110_msgs.msg._opp_wpnt.OppWpnt", full_classname_dest, 31) == 0);
  }
  f110_msgs__msg__OppWpnt * ros_message = _ros_message;
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // s_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "s_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->s_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // d_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "d_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->d_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // proj_vs_mps
    PyObject * field = PyObject_GetAttrString(_pymsg, "proj_vs_mps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->proj_vs_mps = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vd_mps
    PyObject * field = PyObject_GetAttrString(_pymsg, "vd_mps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vd_mps = PyFloat_AS_DOUBLE(field);
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

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * f110_msgs__msg__opp_wpnt__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OppWpnt */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("f110_msgs.msg._opp_wpnt");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OppWpnt");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  f110_msgs__msg__OppWpnt * ros_message = (f110_msgs__msg__OppWpnt *)raw_ros_message;
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
  {  // s_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->s_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // d_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->d_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "d_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // proj_vs_mps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->proj_vs_mps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "proj_vs_mps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vd_mps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vd_mps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vd_mps", field);
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

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
