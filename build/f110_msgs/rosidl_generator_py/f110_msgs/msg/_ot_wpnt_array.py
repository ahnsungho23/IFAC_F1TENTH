# generated from rosidl_generator_py/resource/_idl.py.em
# with input from f110_msgs:msg/OTWpntArray.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OTWpntArray(type):
    """Metaclass of message 'OTWpntArray'."""

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
                'f110_msgs.msg.OTWpntArray')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ot_wpnt_array
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ot_wpnt_array
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ot_wpnt_array
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ot_wpnt_array
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ot_wpnt_array

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from f110_msgs.msg import Wpnt
            if Wpnt.__class__._TYPE_SUPPORT is None:
                Wpnt.__class__.__import_type_support__()

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


class OTWpntArray(metaclass=Metaclass_OTWpntArray):
    """Message class 'OTWpntArray'."""

    __slots__ = [
        '_header',
        '_last_switch_time',
        '_side_switch',
        '_ot_side',
        '_ot_line',
        '_wpnts',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'last_switch_time': 'builtin_interfaces/Time',
        'side_switch': 'boolean',
        'ot_side': 'string',
        'ot_line': 'string',
        'wpnts': 'sequence<f110_msgs/Wpnt>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['f110_msgs', 'msg'], 'Wpnt')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from builtin_interfaces.msg import Time
        self.last_switch_time = kwargs.get('last_switch_time', Time())
        self.side_switch = kwargs.get('side_switch', bool())
        self.ot_side = kwargs.get('ot_side', str())
        self.ot_line = kwargs.get('ot_line', str())
        self.wpnts = kwargs.get('wpnts', [])

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
        if self.last_switch_time != other.last_switch_time:
            return False
        if self.side_switch != other.side_switch:
            return False
        if self.ot_side != other.ot_side:
            return False
        if self.ot_line != other.ot_line:
            return False
        if self.wpnts != other.wpnts:
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
    def last_switch_time(self):
        """Message field 'last_switch_time'."""
        return self._last_switch_time

    @last_switch_time.setter
    def last_switch_time(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'last_switch_time' field must be a sub message of type 'Time'"
        self._last_switch_time = value

    @builtins.property
    def side_switch(self):
        """Message field 'side_switch'."""
        return self._side_switch

    @side_switch.setter
    def side_switch(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'side_switch' field must be of type 'bool'"
        self._side_switch = value

    @builtins.property
    def ot_side(self):
        """Message field 'ot_side'."""
        return self._ot_side

    @ot_side.setter
    def ot_side(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'ot_side' field must be of type 'str'"
        self._ot_side = value

    @builtins.property
    def ot_line(self):
        """Message field 'ot_line'."""
        return self._ot_line

    @ot_line.setter
    def ot_line(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'ot_line' field must be of type 'str'"
        self._ot_line = value

    @builtins.property
    def wpnts(self):
        """Message field 'wpnts'."""
        return self._wpnts

    @wpnts.setter
    def wpnts(self, value):
        if __debug__:
            from f110_msgs.msg import Wpnt
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Wpnt) for v in value) and
                 True), \
                "The 'wpnts' field must be a set or sequence and each value of type 'Wpnt'"
        self._wpnts = value
