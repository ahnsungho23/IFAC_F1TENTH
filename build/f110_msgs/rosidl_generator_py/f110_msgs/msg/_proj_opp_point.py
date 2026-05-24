# generated from rosidl_generator_py/resource/_idl.py.em
# with input from f110_msgs:msg/ProjOppPoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ProjOppPoint(type):
    """Metaclass of message 'ProjOppPoint'."""

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
                'f110_msgs.msg.ProjOppPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__proj_opp_point
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__proj_opp_point
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__proj_opp_point
            cls._TYPE_SUPPORT = module.type_support_msg__msg__proj_opp_point
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__proj_opp_point

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ProjOppPoint(metaclass=Metaclass_ProjOppPoint):
    """Message class 'ProjOppPoint'."""

    __slots__ = [
        '_s',
        '_d',
        '_vs',
        '_vd',
        '_is_static',
        '_is_visible',
        '_time',
        '_s_var',
        '_d_var',
        '_vs_var',
        '_vd_var',
    ]

    _fields_and_field_types = {
        's': 'double',
        'd': 'double',
        'vs': 'double',
        'vd': 'double',
        'is_static': 'boolean',
        'is_visible': 'boolean',
        'time': 'double',
        's_var': 'double',
        'd_var': 'double',
        'vs_var': 'double',
        'vd_var': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
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
        self.s = kwargs.get('s', float())
        self.d = kwargs.get('d', float())
        self.vs = kwargs.get('vs', float())
        self.vd = kwargs.get('vd', float())
        self.is_static = kwargs.get('is_static', bool())
        self.is_visible = kwargs.get('is_visible', bool())
        self.time = kwargs.get('time', float())
        self.s_var = kwargs.get('s_var', float())
        self.d_var = kwargs.get('d_var', float())
        self.vs_var = kwargs.get('vs_var', float())
        self.vd_var = kwargs.get('vd_var', float())

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
        if self.s != other.s:
            return False
        if self.d != other.d:
            return False
        if self.vs != other.vs:
            return False
        if self.vd != other.vd:
            return False
        if self.is_static != other.is_static:
            return False
        if self.is_visible != other.is_visible:
            return False
        if self.time != other.time:
            return False
        if self.s_var != other.s_var:
            return False
        if self.d_var != other.d_var:
            return False
        if self.vs_var != other.vs_var:
            return False
        if self.vd_var != other.vd_var:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def s(self):
        """Message field 's'."""
        return self._s

    @s.setter
    def s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 's' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._s = value

    @builtins.property
    def d(self):
        """Message field 'd'."""
        return self._d

    @d.setter
    def d(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d = value

    @builtins.property
    def vs(self):
        """Message field 'vs'."""
        return self._vs

    @vs.setter
    def vs(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vs' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vs' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vs = value

    @builtins.property
    def vd(self):
        """Message field 'vd'."""
        return self._vd

    @vd.setter
    def vd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vd' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vd' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vd = value

    @builtins.property
    def is_static(self):
        """Message field 'is_static'."""
        return self._is_static

    @is_static.setter
    def is_static(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_static' field must be of type 'bool'"
        self._is_static = value

    @builtins.property
    def is_visible(self):
        """Message field 'is_visible'."""
        return self._is_visible

    @is_visible.setter
    def is_visible(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_visible' field must be of type 'bool'"
        self._is_visible = value

    @builtins.property
    def time(self):
        """Message field 'time'."""
        return self._time

    @time.setter
    def time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._time = value

    @builtins.property
    def s_var(self):
        """Message field 's_var'."""
        return self._s_var

    @s_var.setter
    def s_var(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's_var' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 's_var' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._s_var = value

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

    @builtins.property
    def vd_var(self):
        """Message field 'vd_var'."""
        return self._vd_var

    @vd_var.setter
    def vd_var(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vd_var' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vd_var' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vd_var = value
