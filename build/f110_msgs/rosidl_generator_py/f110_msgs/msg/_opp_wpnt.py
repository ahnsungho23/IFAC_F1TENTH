# generated from rosidl_generator_py/resource/_idl.py.em
# with input from f110_msgs:msg/OppWpnt.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OppWpnt(type):
    """Metaclass of message 'OppWpnt'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('f110_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'f110_msgs.msg.OppWpnt')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__opp_wpnt
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__opp_wpnt
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__opp_wpnt
            cls._TYPE_SUPPORT = module.type_support_msg__msg__opp_wpnt
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__opp_wpnt

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OppWpnt(metaclass=Metaclass_OppWpnt):
    """Message class 'OppWpnt'."""

    __slots__ = [
        '_id',
        '_s_m',
        '_d_m',
        '_x_m',
        '_y_m',
        '_proj_vs_mps',
        '_vd_mps',
        '_d_var',
        '_vs_var',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        's_m': 'double',
        'd_m': 'double',
        'x_m': 'double',
        'y_m': 'double',
        'proj_vs_mps': 'double',
        'vd_mps': 'double',
        'd_var': 'double',
        'vs_var': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.s_m = kwargs.get('s_m', float())
        self.d_m = kwargs.get('d_m', float())
        self.x_m = kwargs.get('x_m', float())
        self.y_m = kwargs.get('y_m', float())
        self.proj_vs_mps = kwargs.get('proj_vs_mps', float())
        self.vd_mps = kwargs.get('vd_mps', float())
        self.d_var = kwargs.get('d_var', float())
        self.vs_var = kwargs.get('vs_var', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.id != other.id:
            return False
        if self.s_m != other.s_m:
            return False
        if self.d_m != other.d_m:
            return False
        if self.x_m != other.x_m:
            return False
        if self.y_m != other.y_m:
            return False
        if self.proj_vs_mps != other.proj_vs_mps:
            return False
        if self.vd_mps != other.vd_mps:
            return False
        if self.d_var != other.d_var:
            return False
        if self.vs_var != other.vs_var:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'id' field must be an integer in [-2147483648, 2147483647]"
        self._id = value

    @builtins.property
    def s_m(self):
        """Message field 's_m'."""
        return self._s_m

    @s_m.setter
    def s_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 's_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._s_m = value

    @builtins.property
    def d_m(self):
        """Message field 'd_m'."""
        return self._d_m

    @d_m.setter
    def d_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d_m = value

    @builtins.property
    def x_m(self):
        """Message field 'x_m'."""
        return self._x_m

    @x_m.setter
    def x_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'x_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._x_m = value

    @builtins.property
    def y_m(self):
        """Message field 'y_m'."""
        return self._y_m

    @y_m.setter
    def y_m(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_m' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'y_m' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._y_m = value

    @builtins.property
    def proj_vs_mps(self):
        """Message field 'proj_vs_mps'."""
        return self._proj_vs_mps

    @proj_vs_mps.setter
    def proj_vs_mps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'proj_vs_mps' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'proj_vs_mps' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._proj_vs_mps = value

    @builtins.property
    def vd_mps(self):
        """Message field 'vd_mps'."""
        return self._vd_mps

    @vd_mps.setter
    def vd_mps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vd_mps' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vd_mps' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vd_mps = value

    @builtins.property
    def d_var(self):
        """Message field 'd_var'."""
        return self._d_var

    @d_var.setter
    def d_var(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd_var' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd_var' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d_var = value

    @builtins.property
    def vs_var(self):
        """Message field 'vs_var'."""
        return self._vs_var

    @vs_var.setter
    def vs_var(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vs_var' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vs_var' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vs_var = value
