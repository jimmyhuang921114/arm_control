// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__BUILDER_HPP_
#define INTERFACE__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface/srv/detail/grounded_sam2_interface__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface
{

namespace srv
{

namespace builder
{

class Init_GroundedSAM2Interface_Request_size_threshold
{
public:
  explicit Init_GroundedSAM2Interface_Request_size_threshold(::interface::srv::GroundedSAM2Interface_Request & msg)
  : msg_(msg)
  {}
  ::interface::srv::GroundedSAM2Interface_Request size_threshold(::interface::srv::GroundedSAM2Interface_Request::_size_threshold_type arg)
  {
    msg_.size_threshold = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::GroundedSAM2Interface_Request msg_;
};

class Init_GroundedSAM2Interface_Request_confidence_threshold
{
public:
  explicit Init_GroundedSAM2Interface_Request_confidence_threshold(::interface::srv::GroundedSAM2Interface_Request & msg)
  : msg_(msg)
  {}
  Init_GroundedSAM2Interface_Request_size_threshold confidence_threshold(::interface::srv::GroundedSAM2Interface_Request::_confidence_threshold_type arg)
  {
    msg_.confidence_threshold = std::move(arg);
    return Init_GroundedSAM2Interface_Request_size_threshold(msg_);
  }

private:
  ::interface::srv::GroundedSAM2Interface_Request msg_;
};

class Init_GroundedSAM2Interface_Request_image
{
public:
  explicit Init_GroundedSAM2Interface_Request_image(::interface::srv::GroundedSAM2Interface_Request & msg)
  : msg_(msg)
  {}
  Init_GroundedSAM2Interface_Request_confidence_threshold image(::interface::srv::GroundedSAM2Interface_Request::_image_type arg)
  {
    msg_.image = std::move(arg);
    return Init_GroundedSAM2Interface_Request_confidence_threshold(msg_);
  }

private:
  ::interface::srv::GroundedSAM2Interface_Request msg_;
};

class Init_GroundedSAM2Interface_Request_prompt
{
public:
  Init_GroundedSAM2Interface_Request_prompt()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GroundedSAM2Interface_Request_image prompt(::interface::srv::GroundedSAM2Interface_Request::_prompt_type arg)
  {
    msg_.prompt = std::move(arg);
    return Init_GroundedSAM2Interface_Request_image(msg_);
  }

private:
  ::interface::srv::GroundedSAM2Interface_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::GroundedSAM2Interface_Request>()
{
  return interface::srv::builder::Init_GroundedSAM2Interface_Request_prompt();
}

}  // namespace interface


namespace interface
{

namespace srv
{

namespace builder
{

class Init_GroundedSAM2Interface_Response_seg
{
public:
  explicit Init_GroundedSAM2Interface_Response_seg(::interface::srv::GroundedSAM2Interface_Response & msg)
  : msg_(msg)
  {}
  ::interface::srv::GroundedSAM2Interface_Response seg(::interface::srv::GroundedSAM2Interface_Response::_seg_type arg)
  {
    msg_.seg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::srv::GroundedSAM2Interface_Response msg_;
};

class Init_GroundedSAM2Interface_Response_bbox
{
public:
  Init_GroundedSAM2Interface_Response_bbox()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GroundedSAM2Interface_Response_seg bbox(::interface::srv::GroundedSAM2Interface_Response::_bbox_type arg)
  {
    msg_.bbox = std::move(arg);
    return Init_GroundedSAM2Interface_Response_seg(msg_);
  }

private:
  ::interface::srv::GroundedSAM2Interface_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::srv::GroundedSAM2Interface_Response>()
{
  return interface::srv::builder::Init_GroundedSAM2Interface_Response_bbox();
}

}  // namespace interface

#endif  // INTERFACE__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__BUILDER_HPP_
