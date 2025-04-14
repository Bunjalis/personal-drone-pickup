// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from interfaces:msg/ELRSCommand.idl
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
#include "interfaces/msg/detail/elrs_command__struct.h"
#include "interfaces/msg/detail/elrs_command__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool interfaces__msg__elrs_command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
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
    assert(strncmp("interfaces.msg._elrs_command.ELRSCommand", full_classname_dest, 40) == 0);
  }
  interfaces__msg__ELRSCommand * ros_message = _ros_message;
  {  // armed
    PyObject * field = PyObject_GetAttrString(_pymsg, "armed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->armed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // channel_0
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_0");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_0 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_1
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_2
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_3
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_3 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_4
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_4 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_5
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_5 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_6
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_6");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_6 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_7
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_7");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_7 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_8
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_8");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_8 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_9
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_9");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_9 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // channel_10
    PyObject * field = PyObject_GetAttrString(_pymsg, "channel_10");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->channel_10 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * interfaces__msg__elrs_command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ELRSCommand */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("interfaces.msg._elrs_command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ELRSCommand");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  interfaces__msg__ELRSCommand * ros_message = (interfaces__msg__ELRSCommand *)raw_ros_message;
  {  // armed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->armed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "armed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_0
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_0", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_6
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_6);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_6", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_7
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_7);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_7", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_8
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_8);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_8", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_9
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_9);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_9", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // channel_10
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->channel_10);
    {
      int rc = PyObject_SetAttrString(_pymessage, "channel_10", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
