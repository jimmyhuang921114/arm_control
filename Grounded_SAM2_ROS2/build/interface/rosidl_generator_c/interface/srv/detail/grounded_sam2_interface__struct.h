// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__STRUCT_H_
#define INTERFACE__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'prompt'
#include "rosidl_runtime_c/string.h"
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in srv/GroundedSAM2Interface in the package interface.
typedef struct interface__srv__GroundedSAM2Interface_Request
{
  /// 提示詞
  rosidl_runtime_c__String prompt;
  /// 圖片數據
  sensor_msgs__msg__Image image;
  /// 信心閥值
  float confidence_threshold;
  /// 大小限制閥值
  float size_threshold;
} interface__srv__GroundedSAM2Interface_Request;

// Struct for a sequence of interface__srv__GroundedSAM2Interface_Request.
typedef struct interface__srv__GroundedSAM2Interface_Request__Sequence
{
  interface__srv__GroundedSAM2Interface_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__GroundedSAM2Interface_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'bbox'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'seg'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GroundedSAM2Interface in the package interface.
typedef struct interface__srv__GroundedSAM2Interface_Response
{
  /// 邊界框 (格式：[xmin, ymin, xmax, ymax])
  rosidl_runtime_c__float__Sequence bbox;
  /// 分割結果 (RLE編碼格式)
  rosidl_runtime_c__String__Sequence seg;
} interface__srv__GroundedSAM2Interface_Response;

// Struct for a sequence of interface__srv__GroundedSAM2Interface_Response.
typedef struct interface__srv__GroundedSAM2Interface_Response__Sequence
{
  interface__srv__GroundedSAM2Interface_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__srv__GroundedSAM2Interface_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__STRUCT_H_
