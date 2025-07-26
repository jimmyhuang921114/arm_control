// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from interface:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice
#include "interface/srv/detail/grounded_sam2_interface__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interface/srv/detail/grounded_sam2_interface__struct.h"
#include "interface/srv/detail/grounded_sam2_interface__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // prompt
#include "rosidl_runtime_c/string_functions.h"  // prompt
#include "sensor_msgs/msg/detail/image__functions.h"  // image

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t get_serialized_size_sensor_msgs__msg__Image(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
size_t max_serialized_size_sensor_msgs__msg__Image(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_interface
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image)();


using _GroundedSAM2Interface_Request__ros_msg_type = interface__srv__GroundedSAM2Interface_Request;

static bool _GroundedSAM2Interface_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GroundedSAM2Interface_Request__ros_msg_type * ros_message = static_cast<const _GroundedSAM2Interface_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: prompt
  {
    const rosidl_runtime_c__String * str = &ros_message->prompt;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->image, cdr))
    {
      return false;
    }
  }

  // Field name: confidence_threshold
  {
    cdr << ros_message->confidence_threshold;
  }

  // Field name: size_threshold
  {
    cdr << ros_message->size_threshold;
  }

  return true;
}

static bool _GroundedSAM2Interface_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GroundedSAM2Interface_Request__ros_msg_type * ros_message = static_cast<_GroundedSAM2Interface_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: prompt
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->prompt.data) {
      rosidl_runtime_c__String__init(&ros_message->prompt);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->prompt,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'prompt'\n");
      return false;
    }
  }

  // Field name: image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->image))
    {
      return false;
    }
  }

  // Field name: confidence_threshold
  {
    cdr >> ros_message->confidence_threshold;
  }

  // Field name: size_threshold
  {
    cdr >> ros_message->size_threshold;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t get_serialized_size_interface__srv__GroundedSAM2Interface_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GroundedSAM2Interface_Request__ros_msg_type * ros_message = static_cast<const _GroundedSAM2Interface_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name prompt
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->prompt.size + 1);
  // field.name image

  current_alignment += get_serialized_size_sensor_msgs__msg__Image(
    &(ros_message->image), current_alignment);
  // field.name confidence_threshold
  {
    size_t item_size = sizeof(ros_message->confidence_threshold);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name size_threshold
  {
    size_t item_size = sizeof(ros_message->size_threshold);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GroundedSAM2Interface_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interface__srv__GroundedSAM2Interface_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t max_serialized_size_interface__srv__GroundedSAM2Interface_Request(
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

  // member: prompt
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
  // member: image
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__Image(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: confidence_threshold
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: size_threshold
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
    using DataType = interface__srv__GroundedSAM2Interface_Request;
    is_plain =
      (
      offsetof(DataType, size_threshold) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GroundedSAM2Interface_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_interface__srv__GroundedSAM2Interface_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GroundedSAM2Interface_Request = {
  "interface::srv",
  "GroundedSAM2Interface_Request",
  _GroundedSAM2Interface_Request__cdr_serialize,
  _GroundedSAM2Interface_Request__cdr_deserialize,
  _GroundedSAM2Interface_Request__get_serialized_size,
  _GroundedSAM2Interface_Request__max_serialized_size
};

static rosidl_message_type_support_t _GroundedSAM2Interface_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GroundedSAM2Interface_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, GroundedSAM2Interface_Request)() {
  return &_GroundedSAM2Interface_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "interface/srv/detail/grounded_sam2_interface__struct.h"
// already included above
// #include "interface/srv/detail/grounded_sam2_interface__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // bbox
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // bbox
// already included above
// #include "rosidl_runtime_c/string.h"  // seg
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // seg

// forward declare type support functions


using _GroundedSAM2Interface_Response__ros_msg_type = interface__srv__GroundedSAM2Interface_Response;

static bool _GroundedSAM2Interface_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GroundedSAM2Interface_Response__ros_msg_type * ros_message = static_cast<const _GroundedSAM2Interface_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: bbox
  {
    size_t size = ros_message->bbox.size;
    auto array_ptr = ros_message->bbox.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: seg
  {
    size_t size = ros_message->seg.size;
    auto array_ptr = ros_message->seg.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  return true;
}

static bool _GroundedSAM2Interface_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GroundedSAM2Interface_Response__ros_msg_type * ros_message = static_cast<_GroundedSAM2Interface_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: bbox
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->bbox.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->bbox);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->bbox, size)) {
      fprintf(stderr, "failed to create array for field 'bbox'");
      return false;
    }
    auto array_ptr = ros_message->bbox.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: seg
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->seg.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->seg);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->seg, size)) {
      fprintf(stderr, "failed to create array for field 'seg'");
      return false;
    }
    auto array_ptr = ros_message->seg.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'seg'\n");
        return false;
      }
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t get_serialized_size_interface__srv__GroundedSAM2Interface_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GroundedSAM2Interface_Response__ros_msg_type * ros_message = static_cast<const _GroundedSAM2Interface_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name bbox
  {
    size_t array_size = ros_message->bbox.size;
    auto array_ptr = ros_message->bbox.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name seg
  {
    size_t array_size = ros_message->seg.size;
    auto array_ptr = ros_message->seg.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GroundedSAM2Interface_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interface__srv__GroundedSAM2Interface_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interface
size_t max_serialized_size_interface__srv__GroundedSAM2Interface_Response(
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

  // member: bbox
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
  // member: seg
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
    using DataType = interface__srv__GroundedSAM2Interface_Response;
    is_plain =
      (
      offsetof(DataType, seg) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GroundedSAM2Interface_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_interface__srv__GroundedSAM2Interface_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GroundedSAM2Interface_Response = {
  "interface::srv",
  "GroundedSAM2Interface_Response",
  _GroundedSAM2Interface_Response__cdr_serialize,
  _GroundedSAM2Interface_Response__cdr_deserialize,
  _GroundedSAM2Interface_Response__get_serialized_size,
  _GroundedSAM2Interface_Response__max_serialized_size
};

static rosidl_message_type_support_t _GroundedSAM2Interface_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GroundedSAM2Interface_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, GroundedSAM2Interface_Response)() {
  return &_GroundedSAM2Interface_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interface/srv/grounded_sam2_interface.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GroundedSAM2Interface__callbacks = {
  "interface::srv",
  "GroundedSAM2Interface",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, GroundedSAM2Interface_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, GroundedSAM2Interface_Response)(),
};

static rosidl_service_type_support_t GroundedSAM2Interface__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GroundedSAM2Interface__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interface, srv, GroundedSAM2Interface)() {
  return &GroundedSAM2Interface__handle;
}

#if defined(__cplusplus)
}
#endif
