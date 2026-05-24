# generated from rosidl_generator_py/resource/_idl.py.em
# with input from f110_msgs:msg/PidData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PidData(type):
    """Metaclass of message 'PidData'."""

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
                'f110_msgs.msg.PidData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pid_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pid_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pid_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pid_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pid_data

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PidData(metaclass=Metaclass_PidData):
    """Message class 'PidData'."""

    __slots__ = [
        '_header',
        '_should',
        '_actual',
        '_error',
        '_d_value',
        '_i_value',
        '_input',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'should': 'double',
        'actual': 'double',
        'error': 'double',
        'd_value': 'double',
        'i_value': 'double',
        'input': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
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
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.should = kwargs.get('should', float())
        self.actual = kwargs.get('actual', float())
        self.error = kwargs.get('error', float())
        self.d_value = kwargs.get('d_value', float())
        self.i_value = kwargs.get('i_value', float())
        self.input = kwargs.get('input', float())

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
        if self.header != other.header:
            return False
        if self.should != other.should:
            return False
        if self.actual != other.actual:
            return False
        if self.error != other.error:
            return False
        if self.d_value != other.d_value:
            return False
        if self.i_value != other.i_value:
            return False
        if self.input != other.input:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def should(self):
        """Message field 'should'."""
        return self._should

    @should.setter
    def should(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'should' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'should' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._should = value

    @builtins.property
    def actual(self):
        """Message field 'actual'."""
        return self._actual

    @actual.setter
    def actual(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'actual' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'actual' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._actual = value

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'error' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'error' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._error = value

    @builtins.property
    def d_value(self):
        """Message field 'd_value'."""
        return self._d_value

    @d_value.setter
    def d_value(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd_value' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd_value' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d_value = value

    @builtins.property
    def i_value(self):
        """Message field 'i_value'."""
        return self._i_value

    @i_value.setter
    def i_value(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'i_value' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'i_value' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._i_value = value

    @builtins.property  # noqa: A003
    def input(self):  # noqa: A003
        """Message field 'input'."""
        return self._input

    @input.setter  # noqa: A003
    def input(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'input' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'input' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._input = value
