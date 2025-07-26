// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tm_robot_if:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice

#ifndef TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__BUILDER_HPP_
#define TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tm_robot_if/srv/detail/grounded_sam2_interface__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tm_robot_if
{

namespace srv
{

namespace builder
{

class Init_GroundedSAM2Interface_Request_selection_mode
{
public:
  explicit Init_GroundedSAM2Interface_Request_selection_mode(::tm_robot_if::srv::GroundedSAM2Interface_Request & msg)
  : msg_(msg)
  {}
  ::tm_robot_if::srv::GroundedSAM2Interface_Request selection_mode(::tm_robot_if::srv::GroundedSAM2Interface_Request::_selection_mode_type arg)
  {
    msg_.selection_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Request msg_;
};

class Init_GroundedSAM2Interface_Request_size_threshold
{
public:
  explicit Init_GroundedSAM2Interface_Request_size_threshold(::tm_robot_if::srv::GroundedSAM2Interface_Request & msg)
  : msg_(msg)
  {}
  Init_GroundedSAM2Interface_Request_selection_mode size_threshold(::tm_robot_if::srv::GroundedSAM2Interface_Request::_size_threshold_type arg)
  {
    msg_.size_threshold = std::move(arg);
    return Init_GroundedSAM2Interface_Request_selection_mode(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Request msg_;
};

class Init_GroundedSAM2Interface_Request_confidence_threshold
{
public:
  explicit Init_GroundedSAM2Interface_Request_confidence_threshold(::tm_robot_if::srv::GroundedSAM2Interface_Request & msg)
  : msg_(msg)
  {}
  Init_GroundedSAM2Interface_Request_size_threshold confidence_threshold(::tm_robot_if::srv::GroundedSAM2Interface_Request::_confidence_threshold_type arg)
  {
    msg_.confidence_threshold = std::move(arg);
    return Init_GroundedSAM2Interface_Request_size_threshold(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Request msg_;
};

class Init_GroundedSAM2Interface_Request_prompt
{
public:
  explicit Init_GroundedSAM2Interface_Request_prompt(::tm_robot_if::srv::GroundedSAM2Interface_Request & msg)
  : msg_(msg)
  {}
  Init_GroundedSAM2Interface_Request_confidence_threshold prompt(::tm_robot_if::srv::GroundedSAM2Interface_Request::_prompt_type arg)
  {
    msg_.prompt = std::move(arg);
    return Init_GroundedSAM2Interface_Request_confidence_threshold(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Request msg_;
};

class Init_GroundedSAM2Interface_Request_depth
{
public:
  explicit Init_GroundedSAM2Interface_Request_depth(::tm_robot_if::srv::GroundedSAM2Interface_Request & msg)
  : msg_(msg)
  {}
  Init_GroundedSAM2Interface_Request_prompt depth(::tm_robot_if::srv::GroundedSAM2Interface_Request::_depth_type arg)
  {
    msg_.depth = std::move(arg);
    return Init_GroundedSAM2Interface_Request_prompt(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Request msg_;
};

class Init_GroundedSAM2Interface_Request_image
{
public:
  Init_GroundedSAM2Interface_Request_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GroundedSAM2Interface_Request_depth image(::tm_robot_if::srv::GroundedSAM2Interface_Request::_image_type arg)
  {
    msg_.image = std::move(arg);
    return Init_GroundedSAM2Interface_Request_depth(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tm_robot_if::srv::GroundedSAM2Interface_Request>()
{
  return tm_robot_if::srv::builder::Init_GroundedSAM2Interface_Request_image();
}

}  // namespace tm_robot_if


namespace tm_robot_if
{

namespace srv
{

namespace builder
{

class Init_GroundedSAM2Interface_Response_score
{
public:
  explicit Init_GroundedSAM2Interface_Response_score(::tm_robot_if::srv::GroundedSAM2Interface_Response & msg)
  : msg_(msg)
  {}
  ::tm_robot_if::srv::GroundedSAM2Interface_Response score(::tm_robot_if::srv::GroundedSAM2Interface_Response::_score_type arg)
  {
    msg_.score = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Response msg_;
};

class Init_GroundedSAM2Interface_Response_label
{
public:
  explicit Init_GroundedSAM2Interface_Response_label(::tm_robot_if::srv::GroundedSAM2Interface_Response & msg)
  : msg_(msg)
  {}
  Init_GroundedSAM2Interface_Response_score label(::tm_robot_if::srv::GroundedSAM2Interface_Response::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_GroundedSAM2Interface_Response_score(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Response msg_;
};

class Init_GroundedSAM2Interface_Response_seg
{
public:
  explicit Init_GroundedSAM2Interface_Response_seg(::tm_robot_if::srv::GroundedSAM2Interface_Response & msg)
  : msg_(msg)
  {}
  Init_GroundedSAM2Interface_Response_label seg(::tm_robot_if::srv::GroundedSAM2Interface_Response::_seg_type arg)
  {
    msg_.seg = std::move(arg);
    return Init_GroundedSAM2Interface_Response_label(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Response msg_;
};

class Init_GroundedSAM2Interface_Response_bbox
{
public:
  explicit Init_GroundedSAM2Interface_Response_bbox(::tm_robot_if::srv::GroundedSAM2Interface_Response & msg)
  : msg_(msg)
  {}
  Init_GroundedSAM2Interface_Response_seg bbox(::tm_robot_if::srv::GroundedSAM2Interface_Response::_bbox_type arg)
  {
    msg_.bbox = std::move(arg);
    return Init_GroundedSAM2Interface_Response_seg(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Response msg_;
};

class Init_GroundedSAM2Interface_Response_binary_image
{
public:
  Init_GroundedSAM2Interface_Response_binary_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GroundedSAM2Interface_Response_bbox binary_image(::tm_robot_if::srv::GroundedSAM2Interface_Response::_binary_image_type arg)
  {
    msg_.binary_image = std::move(arg);
    return Init_GroundedSAM2Interface_Response_bbox(msg_);
  }

private:
  ::tm_robot_if::srv::GroundedSAM2Interface_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::tm_robot_if::srv::GroundedSAM2Interface_Response>()
{
  return tm_robot_if::srv::builder::Init_GroundedSAM2Interface_Response_binary_image();
}

}  // namespace tm_robot_if

#endif  // TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__BUILDER_HPP_
