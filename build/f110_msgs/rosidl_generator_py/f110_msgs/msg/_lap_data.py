# generated from rosidl_generator_py/resource/_idl.py.em
# with input from f110_msgs:msg/LapData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LapData(type):
    """Metaclass of message 'LapData'."""

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
                'f110_msgs.msg.LapData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__lap_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__lap_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__lap_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__lap_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__lap_data

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


class LapData(metaclass=Metaclass_LapData):
    """Message class 'LapData'."""

    __slots__ = [
        '_header',
        '_lap_count',
        '_lap_time',
        '_average_lateral_error_to_global_waypoints',
        '_max_lateral_error_to_global_waypoints',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'lap_count': 'int32',
        'lap_time': 'double',
        'average_lateral_error_to_global_waypoints': 'double',
        'max_lateral_error_to_global_waypoints': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
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
        self.lap_count = kwargs.get('lap_count', int())
        self.lap_time = kwargs.get('lap_time', float())
        self.average_lateral_error_to_global_waypoints = kwargs.get('average_lateral_error_to_global_waypoints', float())
        self.max_lateral_error_to_global_waypoints = kwargs.get('max_lateral_error_to_global_waypoints', float())

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
        if self.lap_count != other.lap_count:
            return False
        if self.lap_time != other.lap_time:
            return False
        if self.average_lateral_error_to_global_waypoints != other.average_lateral_error_to_global_waypoints:
            return False
        if self.max_lateral_error_to_global_waypoints != other.max_lateral_error_to_global_waypoints:
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
    def lap_count(self):
        """Message field 'lap_count'."""
        return self._lap_count

    @lap_count.setter
    def lap_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'lap_count' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'lap_count' field must be an integer in [-2147483648, 2147483647]"
        self._lap_count = value

    @builtins.property
    def lap_time(self):
        """Message field 'lap_time'."""
        return self._lap_time

    @lap_time.setter
    def lap_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lap_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lap_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lap_time = value

    @builtins.property
    def average_lateral_error_to_global_waypoints(self):
        """Message field 'average_lateral_error_to_global_waypoints'."""
        return self._average_lateral_error_to_global_waypoints

    @average_lateral_error_to_global_waypoints.setter
    def average_lateral_error_to_global_waypoints(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'average_lateral_error_to_global_waypoints' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'average_lateral_error_to_global_waypoints' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._average_lateral_error_to_global_waypoints = value

    @builtins.property
    def max_lateral_error_to_global_waypoints(self):
        """Message field 'max_lateral_error_to_global_waypoints'."""
        return self._max_lateral_error_to_global_waypoints

    @max_lateral_error_to_global_waypoints.setter
    def max_lateral_error_to_global_waypoints(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'max_lateral_error_to_global_waypoints' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'max_lateral_error_to_global_waypoints' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._max_lateral_error_to_global_waypoints = value
