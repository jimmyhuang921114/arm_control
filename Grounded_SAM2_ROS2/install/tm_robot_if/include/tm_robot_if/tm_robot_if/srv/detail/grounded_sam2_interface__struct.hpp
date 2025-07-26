// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tm_robot_if:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice

#ifndef TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__STRUCT_HPP_
#define TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'image'
// Member 'depth'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tm_robot_if__srv__GroundedSAM2Interface_Request __attribute__((deprecated))
#else
# define DEPRECATED__tm_robot_if__srv__GroundedSAM2Interface_Request __declspec(deprecated)
#endif

namespace tm_robot_if
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GroundedSAM2Interface_Request_
{
  using Type = GroundedSAM2Interface_Request_<ContainerAllocator>;

  explicit GroundedSAM2Interface_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_init),
    depth(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->prompt = "";
      this->confidence_threshold = 0.0f;
      this->size_threshold = 0.0f;
      this->selection_mode = "";
    }
  }

  explicit GroundedSAM2Interface_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_alloc, _init),
    depth(_alloc, _init),
    prompt(_alloc),
    selection_mode(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->prompt = "";
      this->confidence_threshold = 0.0f;
      this->size_threshold = 0.0f;
      this->selection_mode = "";
    }
  }

  // field types and members
  using _image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _image_type image;
  using _depth_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _depth_type depth;
  using _prompt_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _prompt_type prompt;
  using _confidence_threshold_type =
    float;
  _confidence_threshold_type confidence_threshold;
  using _size_threshold_type =
    float;
  _size_threshold_type size_threshold;
  using _selection_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _selection_mode_type selection_mode;

  // setters for named parameter idiom
  Type & set__image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->image = _arg;
    return *this;
  }
  Type & set__depth(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->depth = _arg;
    return *this;
  }
  Type & set__prompt(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->prompt = _arg;
    return *this;
  }
  Type & set__confidence_threshold(
    const float & _arg)
  {
    this->confidence_threshold = _arg;
    return *this;
  }
  Type & set__size_threshold(
    const float & _arg)
  {
    this->size_threshold = _arg;
    return *this;
  }
  Type & set__selection_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->selection_mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tm_robot_if__srv__GroundedSAM2Interface_Request
    std::shared_ptr<tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tm_robot_if__srv__GroundedSAM2Interface_Request
    std::shared_ptr<tm_robot_if::srv::GroundedSAM2Interface_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GroundedSAM2Interface_Request_ & other) const
  {
    if (this->image != other.image) {
      return false;
    }
    if (this->depth != other.depth) {
      return false;
    }
    if (this->prompt != other.prompt) {
      return false;
    }
    if (this->confidence_threshold != other.confidence_threshold) {
      return false;
    }
    if (this->size_threshold != other.size_threshold) {
      return false;
    }
    if (this->selection_mode != other.selection_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const GroundedSAM2Interface_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GroundedSAM2Interface_Request_

// alias to use template instance with default allocator
using GroundedSAM2Interface_Request =
  tm_robot_if::srv::GroundedSAM2Interface_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace tm_robot_if


// Include directives for member types
// Member 'binary_image'
// already included above
// #include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tm_robot_if__srv__GroundedSAM2Interface_Response __attribute__((deprecated))
#else
# define DEPRECATED__tm_robot_if__srv__GroundedSAM2Interface_Response __declspec(deprecated)
#endif

namespace tm_robot_if
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GroundedSAM2Interface_Response_
{
  using Type = GroundedSAM2Interface_Response_<ContainerAllocator>;

  explicit GroundedSAM2Interface_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : binary_image(_init)
  {
    (void)_init;
  }

  explicit GroundedSAM2Interface_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : binary_image(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _binary_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _binary_image_type binary_image;
  using _bbox_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _bbox_type bbox;
  using _seg_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _seg_type seg;
  using _label_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _label_type label;
  using _score_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _score_type score;

  // setters for named parameter idiom
  Type & set__binary_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->binary_image = _arg;
    return *this;
  }
  Type & set__bbox(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->bbox = _arg;
    return *this;
  }
  Type & set__seg(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->seg = _arg;
    return *this;
  }
  Type & set__label(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__score(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->score = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tm_robot_if__srv__GroundedSAM2Interface_Response
    std::shared_ptr<tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tm_robot_if__srv__GroundedSAM2Interface_Response
    std::shared_ptr<tm_robot_if::srv::GroundedSAM2Interface_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GroundedSAM2Interface_Response_ & other) const
  {
    if (this->binary_image != other.binary_image) {
      return false;
    }
    if (this->bbox != other.bbox) {
      return false;
    }
    if (this->seg != other.seg) {
      return false;
    }
    if (this->label != other.label) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    return true;
  }
  bool operator!=(const GroundedSAM2Interface_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GroundedSAM2Interface_Response_

// alias to use template instance with default allocator
using GroundedSAM2Interface_Response =
  tm_robot_if::srv::GroundedSAM2Interface_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace tm_robot_if

namespace tm_robot_if
{

namespace srv
{

struct GroundedSAM2Interface
{
  using Request = tm_robot_if::srv::GroundedSAM2Interface_Request;
  using Response = tm_robot_if::srv::GroundedSAM2Interface_Response;
};

}  // namespace srv

}  // namespace tm_robot_if

#endif  // TM_ROBOT_IF__SRV__DETAIL__GROUNDED_SAM2_INTERFACE__STRUCT_HPP_
