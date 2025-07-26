// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tm_robot_if:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice

#ifndef TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__TRAITS_HPP_
#define TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tm_robot_if/srv/detail/grounded_sam2_interface__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'image'
// Member 'depth'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace tm_robot_if
{

namespace srv
{

inline void to_flow_style_yaml(
  const GroundedSAM2Interface_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: image
  {
    out << "image: ";
    to_flow_style_yaml(msg.image, out);
    out << ", ";
  }

  // member: depth
  {
    out << "depth: ";
    to_flow_style_yaml(msg.depth, out);
    out << ", ";
  }

  // member: prompt
  {
    out << "prompt: ";
    rosidl_generator_traits::value_to_yaml(msg.prompt, out);
    out << ", ";
  }

  // member: confidence_threshold
  {
    out << "confidence_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence_threshold, out);
    out << ", ";
  }

  // member: size_threshold
  {
    out << "size_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.size_threshold, out);
    out << ", ";
  }

  // member: selection_mode
  {
    out << "selection_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GroundedSAM2Interface_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image:\n";
    to_block_style_yaml(msg.image, out, indentation + 2);
  }

  // member: depth
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "depth:\n";
    to_block_style_yaml(msg.depth, out, indentation + 2);
  }

  // member: prompt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "prompt: ";
    rosidl_generator_traits::value_to_yaml(msg.prompt, out);
    out << "\n";
  }

  // member: confidence_threshold
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence_threshold, out);
    out << "\n";
  }

  // member: size_threshold
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size_threshold: ";
    rosidl_generator_traits::value_to_yaml(msg.size_threshold, out);
    out << "\n";
  }

  // member: selection_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selection_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.selection_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GroundedSAM2Interface_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tm_robot_if

namespace rosidl_generator_traits
{

[[deprecated("use tm_robot_if::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tm_robot_if::srv::GroundedSAM2Interface_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_robot_if::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_robot_if::srv::to_yaml() instead")]]
inline std::string to_yaml(const tm_robot_if::srv::GroundedSAM2Interface_Request & msg)
{
  return tm_robot_if::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tm_robot_if::srv::GroundedSAM2Interface_Request>()
{
  return "tm_robot_if::srv::GroundedSAM2Interface_Request";
}

template<>
inline const char * name<tm_robot_if::srv::GroundedSAM2Interface_Request>()
{
  return "tm_robot_if/srv/GroundedSAM2Interface_Request";
}

template<>
struct has_fixed_size<tm_robot_if::srv::GroundedSAM2Interface_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_robot_if::srv::GroundedSAM2Interface_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_robot_if::srv::GroundedSAM2Interface_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'binary_image'
// already included above
// #include "sensor_msgs/msg/detail/image__traits.hpp"

namespace tm_robot_if
{

namespace srv
{

inline void to_flow_style_yaml(
  const GroundedSAM2Interface_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: binary_image
  {
    out << "binary_image: ";
    to_flow_style_yaml(msg.binary_image, out);
    out << ", ";
  }

  // member: bbox
  {
    if (msg.bbox.size() == 0) {
      out << "bbox: []";
    } else {
      out << "bbox: [";
      size_t pending_items = msg.bbox.size();
      for (auto item : msg.bbox) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: seg
  {
    if (msg.seg.size() == 0) {
      out << "seg: []";
    } else {
      out << "seg: [";
      size_t pending_items = msg.seg.size();
      for (auto item : msg.seg) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: label
  {
    if (msg.label.size() == 0) {
      out << "label: []";
    } else {
      out << "label: [";
      size_t pending_items = msg.label.size();
      for (auto item : msg.label) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: score
  {
    if (msg.score.size() == 0) {
      out << "score: []";
    } else {
      out << "score: [";
      size_t pending_items = msg.score.size();
      for (auto item : msg.score) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GroundedSAM2Interface_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: binary_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "binary_image:\n";
    to_block_style_yaml(msg.binary_image, out, indentation + 2);
  }

  // member: bbox
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bbox.size() == 0) {
      out << "bbox: []\n";
    } else {
      out << "bbox:\n";
      for (auto item : msg.bbox) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: seg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.seg.size() == 0) {
      out << "seg: []\n";
    } else {
      out << "seg:\n";
      for (auto item : msg.seg) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.label.size() == 0) {
      out << "label: []\n";
    } else {
      out << "label:\n";
      for (auto item : msg.label) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.score.size() == 0) {
      out << "score: []\n";
    } else {
      out << "score:\n";
      for (auto item : msg.score) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GroundedSAM2Interface_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace tm_robot_if

namespace rosidl_generator_traits
{

[[deprecated("use tm_robot_if::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tm_robot_if::srv::GroundedSAM2Interface_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  tm_robot_if::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tm_robot_if::srv::to_yaml() instead")]]
inline std::string to_yaml(const tm_robot_if::srv::GroundedSAM2Interface_Response & msg)
{
  return tm_robot_if::srv::to_yaml(msg);
}

template<>
inline const char * data_type<tm_robot_if::srv::GroundedSAM2Interface_Response>()
{
  return "tm_robot_if::srv::GroundedSAM2Interface_Response";
}

template<>
inline const char * name<tm_robot_if::srv::GroundedSAM2Interface_Response>()
{
  return "tm_robot_if/srv/GroundedSAM2Interface_Response";
}

template<>
struct has_fixed_size<tm_robot_if::srv::GroundedSAM2Interface_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tm_robot_if::srv::GroundedSAM2Interface_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tm_robot_if::srv::GroundedSAM2Interface_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<tm_robot_if::srv::GroundedSAM2Interface>()
{
  return "tm_robot_if::srv::GroundedSAM2Interface";
}

template<>
inline const char * name<tm_robot_if::srv::GroundedSAM2Interface>()
{
  return "tm_robot_if/srv/GroundedSAM2Interface";
}

template<>
struct has_fixed_size<tm_robot_if::srv::GroundedSAM2Interface>
  : std::integral_constant<
    bool,
    has_fixed_size<tm_robot_if::srv::GroundedSAM2Interface_Request>::value &&
    has_fixed_size<tm_robot_if::srv::GroundedSAM2Interface_Response>::value
  >
{
};

template<>
struct has_bounded_size<tm_robot_if::srv::GroundedSAM2Interface>
  : std::integral_constant<
    bool,
    has_bounded_size<tm_robot_if::srv::GroundedSAM2Interface_Request>::value &&
    has_bounded_size<tm_robot_if::srv::GroundedSAM2Interface_Response>::value
  >
{
};

template<>
struct is_service<tm_robot_if::srv::GroundedSAM2Interface>
  : std::true_type
{
};

template<>
struct is_service_request<tm_robot_if::srv::GroundedSAM2Interface_Request>
  : std::true_type
{
};

template<>
struct is_service_response<tm_robot_if::srv::GroundedSAM2Interface_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__TRAITS_HPP_
