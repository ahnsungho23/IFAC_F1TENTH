// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from f110_msgs:msg/Wpnt.idl
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
#include "f110_msgs/msg/detail/wpnt__struct.h"
#include "f110_msgs/msg/detail/wpnt__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool f110_msgs__msg__wpnt__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[25];
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
    assert(strncmp("f110_msgs.msg._wpnt.Wpnt", full_classname_dest, 24) == 0);
  }
  f110_msgs__msg__Wpnt * ros_message = _ros_message;
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
  {  // psi_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "psi_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->psi_rad = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kappa_radpm
    PyObject * field = PyObject_GetAttrString(_pymsg, "kappa_radpm");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kappa_radpm = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vx_mps
    PyObject * field = PyObject_GetAttrString(_pymsg, "vx_mps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vx_mps = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ax_mps2
    PyObject * field = PyObject_GetAttrString(_pymsg, "ax_mps2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ax_mps2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * f110_msgs__msg__wpnt__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Wpnt */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("f110_msgs.msg._wpnt");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Wpnt");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  f110_msgs__msg__Wpnt * ros_message = (f110_msgs__msg__Wpnt *)raw_ros_message;
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
  {  // psi_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->psi_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "psi_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kappa_radpm
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kappa_radpm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kappa_radpm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vx_mps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vx_mps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vx_mps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ax_mps2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ax_mps2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ax_mps2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
