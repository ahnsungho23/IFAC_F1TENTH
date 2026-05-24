# generated from rosidl_generator_py/resource/_idl.py.em
# with input from f110_msgs:msg/Wpnt.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Wpnt(type):
    """Metaclass of message 'Wpnt'."""

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
                'f110_msgs.msg.Wpnt')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__wpnt
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__wpnt
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__wpnt
            cls._TYPE_SUPPORT = module.type_support_msg__msg__wpnt
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__wpnt

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Wpnt(metaclass=Metaclass_Wpnt):
    """Message class 'Wpnt'."""

    __slots__ = [
        '_id',
        '_s_m',
        '_d_m',
        '_x_m',
        '_y_m',
        '_d_right',
        '_d_left',
        '_psi_rad',
        '_kappa_radpm',
        '_vx_mps',
        '_ax_mps2',
    ]

    _fields_and_field_types = {
        'id': 'int32',
        's_m': 'double',
        'd_m': 'double',
        'x_m': 'double',
        'y_m': 'double',
        'd_right': 'double',
        'd_left': 'double',
        'psi_rad': 'double',
        'kappa_radpm': 'double',
        'vx_mps': 'double',
        'ax_mps2': 'double',
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
        self.d_right = kwargs.get('d_right', float())
        self.d_left = kwargs.get('d_left', float())
        self.psi_rad = kwargs.get('psi_rad', float())
        self.kappa_radpm = kwargs.get('kappa_radpm', float())
        self.vx_mps = kwargs.get('vx_mps', float())
        self.ax_mps2 = kwargs.get('ax_mps2', float())

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
        if self.d_right != other.d_right:
            return False
        if self.d_left != other.d_left:
            return False
        if self.psi_rad != other.psi_rad:
            return False
        if self.kappa_radpm != other.kappa_radpm:
            return False
        if self.vx_mps != other.vx_mps:
            return False
        if self.ax_mps2 != other.ax_mps2:
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
    def psi_rad(self):
        """Message field 'psi_rad'."""
        return self._psi_rad

    @psi_rad.setter
    def psi_rad(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'psi_rad' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'psi_rad' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._psi_rad = value

    @builtins.property
    def kappa_radpm(self):
        """Message field 'kappa_radpm'."""
        return self._kappa_radpm

    @kappa_radpm.setter
    def kappa_radpm(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kappa_radpm' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'kappa_radpm' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._kappa_radpm = value

    @builtins.property
    def vx_mps(self):
        """Message field 'vx_mps'."""
        return self._vx_mps

    @vx_mps.setter
    def vx_mps(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vx_mps' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'vx_mps' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._vx_mps = value

    @builtins.property
    def ax_mps2(self):
        """Message field 'ax_mps2'."""
        return self._ax_mps2

    @ax_mps2.setter
    def ax_mps2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ax_mps2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ax_mps2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ax_mps2 = value
