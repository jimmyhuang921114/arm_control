// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tm_robot_if:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tm_robot_if/srv/detail/grounded_sam2_interface__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tm_robot_if
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GroundedSAM2Interface_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tm_robot_if::srv::GroundedSAM2Interface_Request(_init);
}

void GroundedSAM2Interface_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tm_robot_if::srv::GroundedSAM2Interface_Request *>(message_memory);
  typed_message->~GroundedSAM2Interface_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GroundedSAM2Interface_Request_message_member_array[6] = {
  {
    "image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Request, image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "depth",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Request, depth),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "prompt",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Request, prompt),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "confidence_threshold",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Request, confidence_threshold),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "size_threshold",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Request, size_threshold),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "selection_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Request, selection_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GroundedSAM2Interface_Request_message_members = {
  "tm_robot_if::srv",  // message namespace
  "GroundedSAM2Interface_Request",  // message name
  6,  // number of fields
  sizeof(tm_robot_if::srv::GroundedSAM2Interface_Request),
  GroundedSAM2Interface_Request_message_member_array,  // message members
  GroundedSAM2Interface_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GroundedSAM2Interface_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GroundedSAM2Interface_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GroundedSAM2Interface_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace tm_robot_if


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tm_robot_if::srv::GroundedSAM2Interface_Request>()
{
  return &::tm_robot_if::srv::rosidl_typesupport_introspection_cpp::GroundedSAM2Interface_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tm_robot_if, srv, GroundedSAM2Interface_Request)() {
  return &::tm_robot_if::srv::rosidl_typesupport_introspection_cpp::GroundedSAM2Interface_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "tm_robot_if/srv/detail/grounded_sam2_interface__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tm_robot_if
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GroundedSAM2Interface_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tm_robot_if::srv::GroundedSAM2Interface_Response(_init);
}

void GroundedSAM2Interface_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tm_robot_if::srv::GroundedSAM2Interface_Response *>(message_memory);
  typed_message->~GroundedSAM2Interface_Response();
}

size_t size_function__GroundedSAM2Interface_Response__bbox(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GroundedSAM2Interface_Response__bbox(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__GroundedSAM2Interface_Response__bbox(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__GroundedSAM2Interface_Response__bbox(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__GroundedSAM2Interface_Response__bbox(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__GroundedSAM2Interface_Response__bbox(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__GroundedSAM2Interface_Response__bbox(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__GroundedSAM2Interface_Response__bbox(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GroundedSAM2Interface_Response__seg(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GroundedSAM2Interface_Response__seg(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__GroundedSAM2Interface_Response__seg(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__GroundedSAM2Interface_Response__seg(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__GroundedSAM2Interface_Response__seg(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__GroundedSAM2Interface_Response__seg(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__GroundedSAM2Interface_Response__seg(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__GroundedSAM2Interface_Response__seg(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GroundedSAM2Interface_Response__label(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GroundedSAM2Interface_Response__label(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__GroundedSAM2Interface_Response__label(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__GroundedSAM2Interface_Response__label(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__GroundedSAM2Interface_Response__label(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__GroundedSAM2Interface_Response__label(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__GroundedSAM2Interface_Response__label(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__GroundedSAM2Interface_Response__label(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GroundedSAM2Interface_Response__score(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GroundedSAM2Interface_Response__score(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__GroundedSAM2Interface_Response__score(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__GroundedSAM2Interface_Response__score(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__GroundedSAM2Interface_Response__score(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__GroundedSAM2Interface_Response__score(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__GroundedSAM2Interface_Response__score(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__GroundedSAM2Interface_Response__score(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GroundedSAM2Interface_Response_message_member_array[5] = {
  {
    "binary_image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Response, binary_image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bbox",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Response, bbox),  // bytes offset in struct
    nullptr,  // default value
    size_function__GroundedSAM2Interface_Response__bbox,  // size() function pointer
    get_const_function__GroundedSAM2Interface_Response__bbox,  // get_const(index) function pointer
    get_function__GroundedSAM2Interface_Response__bbox,  // get(index) function pointer
    fetch_function__GroundedSAM2Interface_Response__bbox,  // fetch(index, &value) function pointer
    assign_function__GroundedSAM2Interface_Response__bbox,  // assign(index, value) function pointer
    resize_function__GroundedSAM2Interface_Response__bbox  // resize(index) function pointer
  },
  {
    "seg",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Response, seg),  // bytes offset in struct
    nullptr,  // default value
    size_function__GroundedSAM2Interface_Response__seg,  // size() function pointer
    get_const_function__GroundedSAM2Interface_Response__seg,  // get_const(index) function pointer
    get_function__GroundedSAM2Interface_Response__seg,  // get(index) function pointer
    fetch_function__GroundedSAM2Interface_Response__seg,  // fetch(index, &value) function pointer
    assign_function__GroundedSAM2Interface_Response__seg,  // assign(index, value) function pointer
    resize_function__GroundedSAM2Interface_Response__seg  // resize(index) function pointer
  },
  {
    "label",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Response, label),  // bytes offset in struct
    nullptr,  // default value
    size_function__GroundedSAM2Interface_Response__label,  // size() function pointer
    get_const_function__GroundedSAM2Interface_Response__label,  // get_const(index) function pointer
    get_function__GroundedSAM2Interface_Response__label,  // get(index) function pointer
    fetch_function__GroundedSAM2Interface_Response__label,  // fetch(index, &value) function pointer
    assign_function__GroundedSAM2Interface_Response__label,  // assign(index, value) function pointer
    resize_function__GroundedSAM2Interface_Response__label  // resize(index) function pointer
  },
  {
    "score",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tm_robot_if::srv::GroundedSAM2Interface_Response, score),  // bytes offset in struct
    nullptr,  // default value
    size_function__GroundedSAM2Interface_Response__score,  // size() function pointer
    get_const_function__GroundedSAM2Interface_Response__score,  // get_const(index) function pointer
    get_function__GroundedSAM2Interface_Response__score,  // get(index) function pointer
    fetch_function__GroundedSAM2Interface_Response__score,  // fetch(index, &value) function pointer
    assign_function__GroundedSAM2Interface_Response__score,  // assign(index, value) function pointer
    resize_function__GroundedSAM2Interface_Response__score  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GroundedSAM2Interface_Response_message_members = {
  "tm_robot_if::srv",  // message namespace
  "GroundedSAM2Interface_Response",  // message name
  5,  // number of fields
  sizeof(tm_robot_if::srv::GroundedSAM2Interface_Response),
  GroundedSAM2Interface_Response_message_member_array,  // message members
  GroundedSAM2Interface_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GroundedSAM2Interface_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GroundedSAM2Interface_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GroundedSAM2Interface_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace tm_robot_if


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tm_robot_if::srv::GroundedSAM2Interface_Response>()
{
  return &::tm_robot_if::srv::rosidl_typesupport_introspection_cpp::GroundedSAM2Interface_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tm_robot_if, srv, GroundedSAM2Interface_Response)() {
  return &::tm_robot_if::srv::rosidl_typesupport_introspection_cpp::GroundedSAM2Interface_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "tm_robot_if/srv/detail/grounded_sam2_interface__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace tm_robot_if
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GroundedSAM2Interface_service_members = {
  "tm_robot_if::srv",  // service namespace
  "GroundedSAM2Interface",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<tm_robot_if::srv::GroundedSAM2Interface>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t GroundedSAM2Interface_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GroundedSAM2Interface_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace tm_robot_if


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<tm_robot_if::srv::GroundedSAM2Interface>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::tm_robot_if::srv::rosidl_typesupport_introspection_cpp::GroundedSAM2Interface_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::tm_robot_if::srv::GroundedSAM2Interface_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::tm_robot_if::srv::GroundedSAM2Interface_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tm_robot_if, srv, GroundedSAM2Interface)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<tm_robot_if::srv::GroundedSAM2Interface>();
}

#ifdef __cplusplus
}
#endif
