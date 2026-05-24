# generated from rosidl_generator_py/resource/_idl.py.em
# with input from f110_msgs:msg/Obstacle.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Obstacle(type):
    """Metaclass of message 'Obstacle'."""

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
                'f110_msgs.msg.Obstacle')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__obstacle
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__obstacle
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__obstacle
            cls._TYPE_SUPPORT = module.type_support_msg__msg__obstacle
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__obstacle

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Obstacle(metaclass=Metaclass_Obstacle):
    """Message class 'Obstacle'."""

    __slots__ = [
        '_id',
        '_s_start',
        '_s_end',
        '_d_right',
        '_d_left',
        '_is_actually_a_gap',
        '_s_center',
        '_d_center',
        '_size',
        '_vs',
        '_vd',
        '_s_var',
        '_d_var',
        '_vs_var',
        '_vd_var',
        '_is_static',
        '_is_visible',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        's_start': 'double',
        's_end': 'double',
        'd_right': 'double',
        'd_left': 'double',
        'is_actually_a_gap': 'boolean',
        's_center': 'double',
        'd_center': 'double',
        'size': 'double',
        'vs': 'double',
        'vd': 'double',
        's_var': 'double',
        'd_var': 'double',
        'vs_var': 'double',
        'vd_var': 'double',
        'is_static': 'boolean',
        'is_visible': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())
        self.s_start = kwargs.get('s_start', float())
        self.s_end = kwargs.get('s_end', float())
        self.d_right = kwargs.get('d_right', float())
        self.d_left = kwargs.get('d_left', float())
        self.is_actually_a_gap = kwargs.get('is_actually_a_gap', bool())
        self.s_center = kwargs.get('s_center', float())
        self.d_center = kwargs.get('d_center', float())
        self.size = kwargs.get('size', float())
        self.vs = kwargs.get('vs', float())
        self.vd = kwargs.get('vd', float())
        self.s_var = kwargs.get('s_var', float())
        self.d_var = kwargs.get('d_var', float())
        self.vs_var = kwargs.get('vs_var', float())
        self.vd_var = kwargs.get('vd_var', float())
        self.is_static = kwargs.get('is_static', bool())
        self.is_visible = kwargs.get('is_visible', bool())

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
        if self.s_start != other.s_start:
            return False
        if self.s_end != other.s_end:
            return False
        if self.d_right != other.d_right:
            return False
        if self.d_left != other.d_left:
            return False
        if self.is_actually_a_gap != other.is_actually_a_gap:
            return False
        if self.s_center != other.s_center:
            return False
        if self.d_center != other.d_center:
            return False
        if self.size != other.size:
            return False
        if self.vs != other.vs:
            return False
        if self.vd != other.vd:
            return False
        if self.s_var != other.s_var:
            return False
        if self.d_var != other.d_var:
            return False
        if self.vs_var != other.vs_var:
            return False
        if self.vd_var != other.vd_var:
            return False
        if self.is_static != other.is_static:
            return False
        if self.is_visible != other.is_visible:
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
    def s_start(self):
        """Message field 's_start'."""
        return self._s_start

    @s_start.setter
    def s_start(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's_start' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 's_start' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._s_start = value

    @builtins.property
    def s_end(self):
        """Message field 's_end'."""
        return self._s_end

    @s_end.setter
    def s_end(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's_end' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 's_end' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._s_end = value

    @builtins.property
    def d_right(self):
        """Message field 'd_right'."""
        return self._d_right

    @d_right.setter
    def d_right(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd_right' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd_right' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d_right = value

    @builtins.property
    def d_left(self):
        """Message field 'd_left'."""
        return self._d_left

    @d_left.setter
    def d_left(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd_left' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd_left' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d_left = value

    @builtins.property
    def is_actually_a_gap(self):
        """Message field 'is_actually_a_gap'."""
        return self._is_actually_a_gap

    @is_actually_a_gap.setter
    def is_actually_a_gap(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_actually_a_gap' field must be of type 'bool'"
        self._is_actually_a_gap = value

    @builtins.property
    def s_center(self):
        """Message field 's_center'."""
        return self._s_center

    @s_center.setter
    def s_center(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's_center' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 's_center' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._s_center = value

    @builtins.property
    def d_center(self):
        """Message field 'd_center'."""
        return self._d_center

    @d_center.setter
    def d_center(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'd_center' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'd_center' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._d_center = value

    @builtins.property
    def size(self):
        """Message field 'size'."""
        return self._size

    @size.setter
    def size(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'size' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'size' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._size = value

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
