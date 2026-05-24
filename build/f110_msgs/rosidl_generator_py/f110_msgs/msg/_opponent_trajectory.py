# generated from rosidl_generator_py/resource/_idl.py.em
# with input from f110_msgs:msg/OpponentTrajectory.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OpponentTrajectory(type):
    """Metaclass of message 'OpponentTrajectory'."""

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
                'f110_msgs.msg.OpponentTrajectory')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__opponent_trajectory
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__opponent_trajectory
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__opponent_trajectory
            cls._TYPE_SUPPORT = module.type_support_msg__msg__opponent_trajectory
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__opponent_trajectory

            from f110_msgs.msg import OppWpnt
            if OppWpnt.__class__._TYPE_SUPPORT is None:
                OppWpnt.__class__.__import_type_support__()

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


class OpponentTrajectory(metaclass=Metaclass_OpponentTrajectory):
    """Message class 'OpponentTrajectory'."""

    __slots__ = [
        '_header',
        '_id',
        '_lap_count',
        '_opp_is_on_trajectory',
        '_oppwpnts',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'id': 'int32',
        'lap_count': 'double',
        'opp_is_on_trajectory': 'boolean',
        'oppwpnts': 'sequence<f110_msgs/OppWpnt>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['f110_msgs', 'msg'], 'OppWpnt')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.id = kwargs.get('id', int())
        self.lap_count = kwargs.get('lap_count', float())
        self.opp_is_on_trajectory = kwargs.get('opp_is_on_trajectory', bool())
        self.oppwpnts = kwargs.get('oppwpnts', [])

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
        if self.id != other.id:
            return False
        if self.lap_count != other.lap_count:
            return False
        if self.opp_is_on_trajectory != other.opp_is_on_trajectory:
            return False
        if self.oppwpnts != other.oppwpnts:
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
    def lap_count(self):
        """Message field 'lap_count'."""
        return self._lap_count

    @lap_count.setter
    def lap_count(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lap_count' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lap_count' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lap_count = value

    @builtins.property
    def opp_is_on_trajectory(self):
        """Message field 'opp_is_on_trajectory'."""
        return self._opp_is_on_trajectory

    @opp_is_on_trajectory.setter
    def opp_is_on_trajectory(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'opp_is_on_trajectory' field must be of type 'bool'"
        self._opp_is_on_trajectory = value

    @builtins.property
    def oppwpnts(self):
        """Message field 'oppwpnts'."""
        return self._oppwpnts

    @oppwpnts.setter
    def oppwpnts(self, value):
        if __debug__:
            from f110_msgs.msg import OppWpnt
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
                 all(isinstance(v, OppWpnt) for v in value) and
                 True), \
                "The 'oppwpnts' field must be a set or sequence and each value of type 'OppWpnt'"
        self._oppwpnts = value
