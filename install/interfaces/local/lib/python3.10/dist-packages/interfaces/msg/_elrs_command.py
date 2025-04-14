# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:msg/ELRSCommand.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ELRSCommand(type):
    """Metaclass of message 'ELRSCommand'."""

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
            module = import_type_support('interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interfaces.msg.ELRSCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__elrs_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__elrs_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__elrs_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__elrs_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__elrs_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ELRSCommand(metaclass=Metaclass_ELRSCommand):
    """Message class 'ELRSCommand'."""

    __slots__ = [
        '_armed',
        '_channel_0',
        '_channel_1',
        '_channel_2',
        '_channel_3',
        '_channel_4',
        '_channel_5',
        '_channel_6',
        '_channel_7',
        '_channel_8',
        '_channel_9',
        '_channel_10',
    ]

    _fields_and_field_types = {
        'armed': 'boolean',
        'channel_0': 'float',
        'channel_1': 'float',
        'channel_2': 'float',
        'channel_3': 'float',
        'channel_4': 'float',
        'channel_5': 'float',
        'channel_6': 'float',
        'channel_7': 'float',
        'channel_8': 'float',
        'channel_9': 'float',
        'channel_10': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.armed = kwargs.get('armed', bool())
        self.channel_0 = kwargs.get('channel_0', float())
        self.channel_1 = kwargs.get('channel_1', float())
        self.channel_2 = kwargs.get('channel_2', float())
        self.channel_3 = kwargs.get('channel_3', float())
        self.channel_4 = kwargs.get('channel_4', float())
        self.channel_5 = kwargs.get('channel_5', float())
        self.channel_6 = kwargs.get('channel_6', float())
        self.channel_7 = kwargs.get('channel_7', float())
        self.channel_8 = kwargs.get('channel_8', float())
        self.channel_9 = kwargs.get('channel_9', float())
        self.channel_10 = kwargs.get('channel_10', float())

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
        if self.armed != other.armed:
            return False
        if self.channel_0 != other.channel_0:
            return False
        if self.channel_1 != other.channel_1:
            return False
        if self.channel_2 != other.channel_2:
            return False
        if self.channel_3 != other.channel_3:
            return False
        if self.channel_4 != other.channel_4:
            return False
        if self.channel_5 != other.channel_5:
            return False
        if self.channel_6 != other.channel_6:
            return False
        if self.channel_7 != other.channel_7:
            return False
        if self.channel_8 != other.channel_8:
            return False
        if self.channel_9 != other.channel_9:
            return False
        if self.channel_10 != other.channel_10:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def armed(self):
        """Message field 'armed'."""
        return self._armed

    @armed.setter
    def armed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'armed' field must be of type 'bool'"
        self._armed = value

    @builtins.property
    def channel_0(self):
        """Message field 'channel_0'."""
        return self._channel_0

    @channel_0.setter
    def channel_0(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_0' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_0' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_0 = value

    @builtins.property
    def channel_1(self):
        """Message field 'channel_1'."""
        return self._channel_1

    @channel_1.setter
    def channel_1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_1 = value

    @builtins.property
    def channel_2(self):
        """Message field 'channel_2'."""
        return self._channel_2

    @channel_2.setter
    def channel_2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_2 = value

    @builtins.property
    def channel_3(self):
        """Message field 'channel_3'."""
        return self._channel_3

    @channel_3.setter
    def channel_3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_3 = value

    @builtins.property
    def channel_4(self):
        """Message field 'channel_4'."""
        return self._channel_4

    @channel_4.setter
    def channel_4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_4 = value

    @builtins.property
    def channel_5(self):
        """Message field 'channel_5'."""
        return self._channel_5

    @channel_5.setter
    def channel_5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_5' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_5' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_5 = value

    @builtins.property
    def channel_6(self):
        """Message field 'channel_6'."""
        return self._channel_6

    @channel_6.setter
    def channel_6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_6' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_6' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_6 = value

    @builtins.property
    def channel_7(self):
        """Message field 'channel_7'."""
        return self._channel_7

    @channel_7.setter
    def channel_7(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_7' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_7' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_7 = value

    @builtins.property
    def channel_8(self):
        """Message field 'channel_8'."""
        return self._channel_8

    @channel_8.setter
    def channel_8(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_8' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_8' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_8 = value

    @builtins.property
    def channel_9(self):
        """Message field 'channel_9'."""
        return self._channel_9

    @channel_9.setter
    def channel_9(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_9' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_9' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_9 = value

    @builtins.property
    def channel_10(self):
        """Message field 'channel_10'."""
        return self._channel_10

    @channel_10.setter
    def channel_10(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'channel_10' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'channel_10' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._channel_10 = value
