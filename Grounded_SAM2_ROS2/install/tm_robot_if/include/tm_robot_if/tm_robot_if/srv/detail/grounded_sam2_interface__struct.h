// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tm_robot_if:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice

#ifndef TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__STRUCT_H_
#define TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'image'
// Member 'depth'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'prompt'
// Member 'selection_mode'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GroundedSAM2Interface in the package tm_robot_if.
typedef struct tm_robot_if__srv__GroundedSAM2Interface_Request
{
  sensor_msgs__msg__Image image;
  sensor_msgs__msg__Image depth;
  rosidl_runtime_c__String prompt;
  float confidence_threshold;
  float size_threshold;
  rosidl_runtime_c__String selection_mode;
} tm_robot_if__srv__GroundedSAM2Interface_Request;

// Struct for a sequence of tm_robot_if__srv__GroundedSAM2Interface_Request.
typedef struct tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence
{
  tm_robot_if__srv__GroundedSAM2Interface_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'binary_image'
// already included above
// #include "sensor_msgs/msg/detail/image__struct.h"
// Member 'bbox'
// Member 'score'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'seg'
// Member 'label'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GroundedSAM2Interface in the package tm_robot_if.
typedef struct tm_robot_if__srv__GroundedSAM2Interface_Response
{
  sensor_msgs__msg__Image binary_image;
  rosidl_runtime_c__float__Sequence bbox;
  rosidl_runtime_c__String__Sequence seg;
  rosidl_runtime_c__String__Sequence label;
  rosidl_runtime_c__float__Sequence score;
} tm_robot_if__srv__GroundedSAM2Interface_Response;

// Struct for a sequence of tm_robot_if__srv__GroundedSAM2Interface_Response.
typedef struct tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence
{
  tm_robot_if__srv__GroundedSAM2Interface_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__STRUCT_H_
