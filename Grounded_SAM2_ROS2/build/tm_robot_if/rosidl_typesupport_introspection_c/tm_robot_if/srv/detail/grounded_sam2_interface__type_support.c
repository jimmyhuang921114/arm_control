// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tm_robot_if:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tm_robot_if/srv/detail/grounded_sam2_interface__rosidl_typesupport_introspection_c.h"
#include "tm_robot_if/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tm_robot_if/srv/detail/grounded_sam2_interface__functions.h"
#include "tm_robot_if/srv/detail/grounded_sam2_interface__struct.h"


// Include directives for member types
// Member `image`
// Member `depth`
#include "sensor_msgs/msg/image.h"
// Member `image`
// Member `depth`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"
// Member `prompt`
// Member `selection_mode`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tm_robot_if__srv__GroundedSAM2Interface_Request__init(message_memory);
}

void tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_fini_function(void * message_memory)
{
  tm_robot_if__srv__GroundedSAM2Interface_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_member_array[6] = {
  {
    "image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Request, image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "depth",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Request, depth),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "prompt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Request, prompt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence_threshold",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Request, confidence_threshold),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size_threshold",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Request, size_threshold),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "selection_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Request, selection_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_members = {
  "tm_robot_if__srv",  // message namespace
  "GroundedSAM2Interface_Request",  // message name
  6,  // number of fields
  sizeof(tm_robot_if__srv__GroundedSAM2Interface_Request),
  tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_member_array,  // message members
  tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_type_support_handle = {
  0,
  &tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tm_robot_if
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tm_robot_if, srv, GroundedSAM2Interface_Request)() {
  tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_type_support_handle.typesupport_identifier) {
    tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tm_robot_if__srv__GroundedSAM2Interface_Request__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "tm_robot_if/srv/detail/grounded_sam2_interface__rosidl_typesupport_introspection_c.h"
// already included above
// #include "tm_robot_if/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "tm_robot_if/srv/detail/grounded_sam2_interface__functions.h"
// already included above
// #include "tm_robot_if/srv/detail/grounded_sam2_interface__struct.h"


// Include directives for member types
// Member `binary_image`
// already included above
// #include "sensor_msgs/msg/image.h"
// Member `binary_image`
// already included above
// #include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"
// Member `bbox`
// Member `score`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `seg`
// Member `label`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tm_robot_if__srv__GroundedSAM2Interface_Response__init(message_memory);
}

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_fini_function(void * message_memory)
{
  tm_robot_if__srv__GroundedSAM2Interface_Response__fini(message_memory);
}

size_t tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__size_function__GroundedSAM2Interface_Response__bbox(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__bbox(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__bbox(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__fetch_function__GroundedSAM2Interface_Response__bbox(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__bbox(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__assign_function__GroundedSAM2Interface_Response__bbox(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__bbox(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__resize_function__GroundedSAM2Interface_Response__bbox(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__size_function__GroundedSAM2Interface_Response__seg(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__seg(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__seg(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__fetch_function__GroundedSAM2Interface_Response__seg(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__seg(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__assign_function__GroundedSAM2Interface_Response__seg(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__seg(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__resize_function__GroundedSAM2Interface_Response__seg(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__size_function__GroundedSAM2Interface_Response__label(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__label(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__label(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__fetch_function__GroundedSAM2Interface_Response__label(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__label(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__assign_function__GroundedSAM2Interface_Response__label(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__label(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__resize_function__GroundedSAM2Interface_Response__label(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__size_function__GroundedSAM2Interface_Response__score(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__score(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__score(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__fetch_function__GroundedSAM2Interface_Response__score(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__score(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__assign_function__GroundedSAM2Interface_Response__score(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__score(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__resize_function__GroundedSAM2Interface_Response__score(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_member_array[5] = {
  {
    "binary_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Response, binary_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bbox",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Response, bbox),  // bytes offset in struct
    NULL,  // default value
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__size_function__GroundedSAM2Interface_Response__bbox,  // size() function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__bbox,  // get_const(index) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__bbox,  // get(index) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__fetch_function__GroundedSAM2Interface_Response__bbox,  // fetch(index, &value) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__assign_function__GroundedSAM2Interface_Response__bbox,  // assign(index, value) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__resize_function__GroundedSAM2Interface_Response__bbox  // resize(index) function pointer
  },
  {
    "seg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Response, seg),  // bytes offset in struct
    NULL,  // default value
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__size_function__GroundedSAM2Interface_Response__seg,  // size() function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__seg,  // get_const(index) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__seg,  // get(index) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__fetch_function__GroundedSAM2Interface_Response__seg,  // fetch(index, &value) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__assign_function__GroundedSAM2Interface_Response__seg,  // assign(index, value) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__resize_function__GroundedSAM2Interface_Response__seg  // resize(index) function pointer
  },
  {
    "label",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Response, label),  // bytes offset in struct
    NULL,  // default value
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__size_function__GroundedSAM2Interface_Response__label,  // size() function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__label,  // get_const(index) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__label,  // get(index) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__fetch_function__GroundedSAM2Interface_Response__label,  // fetch(index, &value) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__assign_function__GroundedSAM2Interface_Response__label,  // assign(index, value) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__resize_function__GroundedSAM2Interface_Response__label  // resize(index) function pointer
  },
  {
    "score",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if__srv__GroundedSAM2Interface_Response, score),  // bytes offset in struct
    NULL,  // default value
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__size_function__GroundedSAM2Interface_Response__score,  // size() function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_const_function__GroundedSAM2Interface_Response__score,  // get_const(index) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__get_function__GroundedSAM2Interface_Response__score,  // get(index) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__fetch_function__GroundedSAM2Interface_Response__score,  // fetch(index, &value) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__assign_function__GroundedSAM2Interface_Response__score,  // assign(index, value) function pointer
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__resize_function__GroundedSAM2Interface_Response__score  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_members = {
  "tm_robot_if__srv",  // message namespace
  "GroundedSAM2Interface_Response",  // message name
  5,  // number of fields
  sizeof(tm_robot_if__srv__GroundedSAM2Interface_Response),
  tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_member_array,  // message members
  tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_type_support_handle = {
  0,
  &tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tm_robot_if
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tm_robot_if, srv, GroundedSAM2Interface_Response)() {
  tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_type_support_handle.typesupport_identifier) {
    tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tm_robot_if__srv__GroundedSAM2Interface_Response__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "tm_robot_if/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "tm_robot_if/srv/detail/grounded_sam2_interface__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers tm_robot_if__srv__detail__grounded_sam2_interface__rosidl_typesupport_introspection_c__GroundedSAM2Interface_service_members = {
  "tm_robot_if__srv",  // service namespace
  "GroundedSAM2Interface",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // tm_robot_if__srv__detail__grounded_sam2_interface__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Request_message_type_support_handle,
  NULL  // response message
  // tm_robot_if__srv__detail__grounded_sam2_interface__rosidl_typesupport_introspection_c__GroundedSAM2Interface_Response_message_type_support_handle
};

static rosidl_service_type_support_t tm_robot_if__srv__detail__grounded_sam2_interface__rosidl_typesupport_introspection_c__GroundedSAM2Interface_service_type_support_handle = {
  0,
  &tm_robot_if__srv__detail__grounded_sam2_interface__rosidl_typesupport_introspection_c__GroundedSAM2Interface_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tm_robot_if, srv, GroundedSAM2Interface_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tm_robot_if, srv, GroundedSAM2Interface_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tm_robot_if
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tm_robot_if, srv, GroundedSAM2Interface)() {
  if (!tm_robot_if__srv__detail__grounded_sam2_interface__rosidl_typesupport_introspection_c__GroundedSAM2Interface_service_type_support_handle.typesupport_identifier) {
    tm_robot_if__srv__detail__grounded_sam2_interface__rosidl_typesupport_introspection_c__GroundedSAM2Interface_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)tm_robot_if__srv__detail__grounded_sam2_interface__rosidl_typesupport_introspection_c__GroundedSAM2Interface_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tm_robot_if, srv, GroundedSAM2Interface_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tm_robot_if, srv, GroundedSAM2Interface_Response)()->data;
  }

  return &tm_robot_if__srv__detail__grounded_sam2_interface__rosidl_typesupport_introspection_c__GroundedSAM2Interface_service_type_support_handle;
}
