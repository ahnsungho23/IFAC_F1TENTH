# generated from rosidl_generator_py/resource/_idl.py.em
# with input from f110_msgs:msg/ProjOppTraj.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ProjOppTraj(type):
    """Metaclass of message 'ProjOppTraj'."""

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
                'f110_msgs.msg.ProjOppTraj')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__proj_opp_traj
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__proj_opp_traj
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__proj_opp_traj
            cls._TYPE_SUPPORT = module.type_support_msg__msg__proj_opp_traj
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__proj_opp_traj

            from f110_msgs.msg import ProjOppPoint
            if ProjOppPoint.__class__._TYPE_SUPPORT is None:
                ProjOppPoint.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ProjOppTraj(metaclass=Metaclass_ProjOppTraj):
    """Message class 'ProjOppTraj'."""

    __slots__ = [
        '_lapcount',
        '_nrofpoints',
        '_opp_is_on_trajectory',
        '_detections',
    ]

    _fields_and_field_types = {
        'lapcount': 'double',
        'nrofpoints': 'double',
        'opp_is_on_trajectory': 'boolean',
        'detections': 'sequence<f110_msgs/ProjOppPoint>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['f110_msgs', 'msg'], 'ProjOppPoint')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.lapcount = kwargs.get('lapcount', float())
        self.nrofpoints = kwargs.get('nrofpoints', float())
        self.opp_is_on_trajectory = kwargs.get('opp_is_on_trajectory', bool())
        self.detections = kwargs.get('detections', [])

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
        if self.lapcount != other.lapcount:
            return False
        if self.nrofpoints != other.nrofpoints:
            return False
        if self.opp_is_on_trajectory != other.opp_is_on_trajectory:
            return False
        if self.detections != other.detections:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def lapcount(self):
        """Message field 'lapcount'."""
        return self._lapcount

    @lapcount.setter
    def lapcount(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'lapcount' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'lapcount' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._lapcount = value

    @builtins.property
    def nrofpoints(self):
        """Message field 'nrofpoints'."""
        return self._nrofpoints

    @nrofpoints.setter
    def nrofpoints(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'nrofpoints' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'nrofpoints' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._nrofpoints = value

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
    def detections(self):
        """Message field 'detections'."""
        return self._detections

    @detections.setter
    def detections(self, value):
        if __debug__:
            from f110_msgs.msg import ProjOppPoint
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
                 all(isinstance(v, ProjOppPoint) for v in value) and
                 True), \
                "The 'detections' field must be a set or sequence and each value of type 'ProjOppPoint'"
        self._detections = value
