// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from interface:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice
#include "interface/srv/detail/grounded_sam2_interface__rosidl_typesupport_fastrtps_cpp.hpp"
#include "interface/srv/detail/grounded_sam2_interface__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace sensor_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const sensor_msgs::msg::Image &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  sensor_msgs::msg::Image &);
size_t get_serialized_size(
  const sensor_msgs::msg::Image &,
  size_t current_alignment);
size_t
max_serialized_size_Image(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace sensor_msgs


namespace interface
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface
cdr_serialize(
  const interface::srv::GroundedSAM2Interface_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: prompt
  cdr << ros_message.prompt;
  // Member: image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.image,
    cdr);
  // Member: confidence_threshold
  cdr << ros_message.confidence_threshold;
  // Member: size_threshold
  cdr << ros_message.size_threshold;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  interface::srv::GroundedSAM2Interface_Request & ros_message)
{
  // Member: prompt
  cdr >> ros_message.prompt;

  // Member: image
  sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.image);

  // Member: confidence_threshold
  cdr >> ros_message.confidence_threshold;

  // Member: size_threshold
  cdr >> ros_message.size_threshold;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface
get_serialized_size(
  const interface::srv::GroundedSAM2Interface_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: prompt
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.prompt.size() + 1);
  // Member: image

  current_alignment +=
    sensor_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.image, current_alignment);
  // Member: confidence_threshold
  {
    size_t item_size = sizeof(ros_message.confidence_threshold);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: size_threshold
  {
    size_t item_size = sizeof(ros_message.size_threshold);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface
max_serialized_size_GroundedSAM2Interface_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: prompt
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: image
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        sensor_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Image(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: confidence_threshold
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: size_threshold
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = interface::srv::GroundedSAM2Interface_Request;
    is_plain =
      (
      offsetof(DataType, size_threshold) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GroundedSAM2Interface_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const interface::srv::GroundedSAM2Interface_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GroundedSAM2Interface_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<interface::srv::GroundedSAM2Interface_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GroundedSAM2Interface_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const interface::srv::GroundedSAM2Interface_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GroundedSAM2Interface_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GroundedSAM2Interface_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GroundedSAM2Interface_Request__callbacks = {
  "interface::srv",
  "GroundedSAM2Interface_Request",
  _GroundedSAM2Interface_Request__cdr_serialize,
  _GroundedSAM2Interface_Request__cdr_deserialize,
  _GroundedSAM2Interface_Request__get_serialized_size,
  _GroundedSAM2Interface_Request__max_serialized_size
};

static rosidl_message_type_support_t _GroundedSAM2Interface_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GroundedSAM2Interface_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace interface

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_interface
const rosidl_message_type_support_t *
get_message_type_support_handle<interface::srv::GroundedSAM2Interface_Request>()
{
  return &interface::srv::typesupport_fastrtps_cpp::_GroundedSAM2Interface_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, interface, srv, GroundedSAM2Interface_Request)() {
  return &interface::srv::typesupport_fastrtps_cpp::_GroundedSAM2Interface_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace interface
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface
cdr_serialize(
  const interface::srv::GroundedSAM2Interface_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: bbox
  {
    cdr << ros_message.bbox;
  }
  // Member: seg
  {
    cdr << ros_message.seg;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  interface::srv::GroundedSAM2Interface_Response & ros_message)
{
  // Member: bbox
  {
    cdr >> ros_message.bbox;
  }

  // Member: seg
  {
    cdr >> ros_message.seg;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface
get_serialized_size(
  const interface::srv::GroundedSAM2Interface_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: bbox
  {
    size_t array_size = ros_message.bbox.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.bbox[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: seg
  {
    size_t array_size = ros_message.seg.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.seg[index].size() + 1);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interface
max_serialized_size_GroundedSAM2Interface_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: bbox
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: seg
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = interface::srv::GroundedSAM2Interface_Response;
    is_plain =
      (
      offsetof(DataType, seg) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GroundedSAM2Interface_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const interface::srv::GroundedSAM2Interface_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GroundedSAM2Interface_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<interface::srv::GroundedSAM2Interface_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GroundedSAM2Interface_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const interface::srv::GroundedSAM2Interface_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GroundedSAM2Interface_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GroundedSAM2Interface_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GroundedSAM2Interface_Response__callbacks = {
  "interface::srv",
  "GroundedSAM2Interface_Response",
  _GroundedSAM2Interface_Response__cdr_serialize,
  _GroundedSAM2Interface_Response__cdr_deserialize,
  _GroundedSAM2Interface_Response__get_serialized_size,
  _GroundedSAM2Interface_Response__max_serialized_size
};

static rosidl_message_type_support_t _GroundedSAM2Interface_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GroundedSAM2Interface_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace interface

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_interface
const rosidl_message_type_support_t *
get_message_type_support_handle<interface::srv::GroundedSAM2Interface_Response>()
{
  return &interface::srv::typesupport_fastrtps_cpp::_GroundedSAM2Interface_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, interface, srv, GroundedSAM2Interface_Response)() {
  return &interface::srv::typesupport_fastrtps_cpp::_GroundedSAM2Interface_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace interface
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _GroundedSAM2Interface__callbacks = {
  "interface::srv",
  "GroundedSAM2Interface",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, interface, srv, GroundedSAM2Interface_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, interface, srv, GroundedSAM2Interface_Response)(),
};

static rosidl_service_type_support_t _GroundedSAM2Interface__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GroundedSAM2Interface__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace interface

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_interface
const rosidl_service_type_support_t *
get_service_type_support_handle<interface::srv::GroundedSAM2Interface>()
{
  return &interface::srv::typesupport_fastrtps_cpp::_GroundedSAM2Interface__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, interface, srv, GroundedSAM2Interface)() {
  return &interface::srv::typesupport_fastrtps_cpp::_GroundedSAM2Interface__handle;
}

#ifdef __cplusplus
}
#endif
