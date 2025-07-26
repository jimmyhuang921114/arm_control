# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tm_robot_if:srv/GroundedSAM2Interface.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GroundedSAM2Interface_Request(type):
    """Metaclass of message 'GroundedSAM2Interface_Request'."""

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
            module = import_type_support('tm_robot_if')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tm_robot_if.srv.GroundedSAM2Interface_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__grounded_sam2_interface__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__grounded_sam2_interface__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__grounded_sam2_interface__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__grounded_sam2_interface__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__grounded_sam2_interface__request

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GroundedSAM2Interface_Request(metaclass=Metaclass_GroundedSAM2Interface_Request):
    """Message class 'GroundedSAM2Interface_Request'."""

    __slots__ = [
        '_image',
        '_depth',
        '_prompt',
        '_confidence_threshold',
        '_size_threshold',
        '_selection_mode',
    ]

    _fields_and_field_types = {
        'image': 'sensor_msgs/Image',
        'depth': 'sensor_msgs/Image',
        'prompt': 'string',
        'confidence_threshold': 'float',
        'size_threshold': 'float',
        'selection_mode': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import Image
        self.image = kwargs.get('image', Image())
        from sensor_msgs.msg import Image
        self.depth = kwargs.get('depth', Image())
        self.prompt = kwargs.get('prompt', str())
        self.confidence_threshold = kwargs.get('confidence_threshold', float())
        self.size_threshold = kwargs.get('size_threshold', float())
        self.selection_mode = kwargs.get('selection_mode', str())

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
        if self.image != other.image:
            return False
        if self.depth != other.depth:
            return False
        if self.prompt != other.prompt:
            return False
        if self.confidence_threshold != other.confidence_threshold:
            return False
        if self.size_threshold != other.size_threshold:
            return False
        if self.selection_mode != other.selection_mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def image(self):
        """Message field 'image'."""
        return self._image

    @image.setter
    def image(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'image' field must be a sub message of type 'Image'"
        self._image = value

    @builtins.property
    def depth(self):
        """Message field 'depth'."""
        return self._depth

    @depth.setter
    def depth(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'depth' field must be a sub message of type 'Image'"
        self._depth = value

    @builtins.property
    def prompt(self):
        """Message field 'prompt'."""
        return self._prompt

    @prompt.setter
    def prompt(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'prompt' field must be of type 'str'"
        self._prompt = value

    @builtins.property
    def confidence_threshold(self):
        """Message field 'confidence_threshold'."""
        return self._confidence_threshold

    @confidence_threshold.setter
    def confidence_threshold(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence_threshold' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence_threshold' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence_threshold = value

    @builtins.property
    def size_threshold(self):
        """Message field 'size_threshold'."""
        return self._size_threshold

    @size_threshold.setter
    def size_threshold(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'size_threshold' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'size_threshold' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._size_threshold = value

    @builtins.property
    def selection_mode(self):
        """Message field 'selection_mode'."""
        return self._selection_mode

    @selection_mode.setter
    def selection_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'selection_mode' field must be of type 'str'"
        self._selection_mode = value


# Import statements for member types

# Member 'bbox'
# Member 'score'
import array  # noqa: E402, I100

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_GroundedSAM2Interface_Response(type):
    """Metaclass of message 'GroundedSAM2Interface_Response'."""

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
            module = import_type_support('tm_robot_if')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tm_robot_if.srv.GroundedSAM2Interface_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__grounded_sam2_interface__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__grounded_sam2_interface__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__grounded_sam2_interface__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__grounded_sam2_interface__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__grounded_sam2_interface__response

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GroundedSAM2Interface_Response(metaclass=Metaclass_GroundedSAM2Interface_Response):
    """Message class 'GroundedSAM2Interface_Response'."""

    __slots__ = [
        '_binary_image',
        '_bbox',
        '_seg',
        '_label',
        '_score',
    ]

    _fields_and_field_types = {
        'binary_image': 'sensor_msgs/Image',
        'bbox': 'sequence<float>',
        'seg': 'sequence<string>',
        'label': 'sequence<string>',
        'score': 'sequence<float>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import Image
        self.binary_image = kwargs.get('binary_image', Image())
        self.bbox = array.array('f', kwargs.get('bbox', []))
        self.seg = kwargs.get('seg', [])
        self.label = kwargs.get('label', [])
        self.score = array.array('f', kwargs.get('score', []))

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
        if self.binary_image != other.binary_image:
            return False
        if self.bbox != other.bbox:
            return False
        if self.seg != other.seg:
            return False
        if self.label != other.label:
            return False
        if self.score != other.score:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def binary_image(self):
        """Message field 'binary_image'."""
        return self._binary_image

    @binary_image.setter
    def binary_image(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'binary_image' field must be a sub message of type 'Image'"
        self._binary_image = value

    @builtins.property
    def bbox(self):
        """Message field 'bbox'."""
        return self._bbox

    @bbox.setter
    def bbox(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'bbox' array.array() must have the type code of 'f'"
            self._bbox = value
            return
        if __debug__:
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'bbox' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._bbox = array.array('f', value)

    @builtins.property
    def seg(self):
        """Message field 'seg'."""
        return self._seg

    @seg.setter
    def seg(self, value):
        if __debug__:
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'seg' field must be a set or sequence and each value of type 'str'"
        self._seg = value

    @builtins.property
    def label(self):
        """Message field 'label'."""
        return self._label

    @label.setter
    def label(self, value):
        if __debug__:
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'label' field must be a set or sequence and each value of type 'str'"
        self._label = value

    @builtins.property
    def score(self):
        """Message field 'score'."""
        return self._score

    @score.setter
    def score(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'score' array.array() must have the type code of 'f'"
            self._score = value
            return
        if __debug__:
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'score' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._score = array.array('f', value)


class Metaclass_GroundedSAM2Interface(type):
    """Metaclass of service 'GroundedSAM2Interface'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tm_robot_if')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tm_robot_if.srv.GroundedSAM2Interface')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__grounded_sam2_interface

            from tm_robot_if.srv import _grounded_sam2_interface
            if _grounded_sam2_interface.Metaclass_GroundedSAM2Interface_Request._TYPE_SUPPORT is None:
                _grounded_sam2_interface.Metaclass_GroundedSAM2Interface_Request.__import_type_support__()
            if _grounded_sam2_interface.Metaclass_GroundedSAM2Interface_Response._TYPE_SUPPORT is None:
                _grounded_sam2_interface.Metaclass_GroundedSAM2Interface_Response.__import_type_support__()


class GroundedSAM2Interface(metaclass=Metaclass_GroundedSAM2Interface):
    from tm_robot_if.srv._grounded_sam2_interface import GroundedSAM2Interface_Request as Request
    from tm_robot_if.srv._grounded_sam2_interface import GroundedSAM2Interface_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
