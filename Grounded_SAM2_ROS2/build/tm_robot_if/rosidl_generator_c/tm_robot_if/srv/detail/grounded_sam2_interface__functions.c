// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tm_robot_if:srv/GroundedSAM2Interface.idl
// generated code does not contain a copyright notice
#include "tm_robot_if/srv/detail/grounded_sam2_interface__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `image`
// Member `depth`
#include "sensor_msgs/msg/detail/image__functions.h"
// Member `prompt`
// Member `selection_mode`
#include "rosidl_runtime_c/string_functions.h"

bool
tm_robot_if__srv__GroundedSAM2Interface_Request__init(tm_robot_if__srv__GroundedSAM2Interface_Request * msg)
{
  if (!msg) {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__init(&msg->image)) {
    tm_robot_if__srv__GroundedSAM2Interface_Request__fini(msg);
    return false;
  }
  // depth
  if (!sensor_msgs__msg__Image__init(&msg->depth)) {
    tm_robot_if__srv__GroundedSAM2Interface_Request__fini(msg);
    return false;
  }
  // prompt
  if (!rosidl_runtime_c__String__init(&msg->prompt)) {
    tm_robot_if__srv__GroundedSAM2Interface_Request__fini(msg);
    return false;
  }
  // confidence_threshold
  // size_threshold
  // selection_mode
  if (!rosidl_runtime_c__String__init(&msg->selection_mode)) {
    tm_robot_if__srv__GroundedSAM2Interface_Request__fini(msg);
    return false;
  }
  return true;
}

void
tm_robot_if__srv__GroundedSAM2Interface_Request__fini(tm_robot_if__srv__GroundedSAM2Interface_Request * msg)
{
  if (!msg) {
    return;
  }
  // image
  sensor_msgs__msg__Image__fini(&msg->image);
  // depth
  sensor_msgs__msg__Image__fini(&msg->depth);
  // prompt
  rosidl_runtime_c__String__fini(&msg->prompt);
  // confidence_threshold
  // size_threshold
  // selection_mode
  rosidl_runtime_c__String__fini(&msg->selection_mode);
}

bool
tm_robot_if__srv__GroundedSAM2Interface_Request__are_equal(const tm_robot_if__srv__GroundedSAM2Interface_Request * lhs, const tm_robot_if__srv__GroundedSAM2Interface_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->image), &(rhs->image)))
  {
    return false;
  }
  // depth
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->depth), &(rhs->depth)))
  {
    return false;
  }
  // prompt
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->prompt), &(rhs->prompt)))
  {
    return false;
  }
  // confidence_threshold
  if (lhs->confidence_threshold != rhs->confidence_threshold) {
    return false;
  }
  // size_threshold
  if (lhs->size_threshold != rhs->size_threshold) {
    return false;
  }
  // selection_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->selection_mode), &(rhs->selection_mode)))
  {
    return false;
  }
  return true;
}

bool
tm_robot_if__srv__GroundedSAM2Interface_Request__copy(
  const tm_robot_if__srv__GroundedSAM2Interface_Request * input,
  tm_robot_if__srv__GroundedSAM2Interface_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__copy(
      &(input->image), &(output->image)))
  {
    return false;
  }
  // depth
  if (!sensor_msgs__msg__Image__copy(
      &(input->depth), &(output->depth)))
  {
    return false;
  }
  // prompt
  if (!rosidl_runtime_c__String__copy(
      &(input->prompt), &(output->prompt)))
  {
    return false;
  }
  // confidence_threshold
  output->confidence_threshold = input->confidence_threshold;
  // size_threshold
  output->size_threshold = input->size_threshold;
  // selection_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->selection_mode), &(output->selection_mode)))
  {
    return false;
  }
  return true;
}

tm_robot_if__srv__GroundedSAM2Interface_Request *
tm_robot_if__srv__GroundedSAM2Interface_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_robot_if__srv__GroundedSAM2Interface_Request * msg = (tm_robot_if__srv__GroundedSAM2Interface_Request *)allocator.allocate(sizeof(tm_robot_if__srv__GroundedSAM2Interface_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_robot_if__srv__GroundedSAM2Interface_Request));
  bool success = tm_robot_if__srv__GroundedSAM2Interface_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_robot_if__srv__GroundedSAM2Interface_Request__destroy(tm_robot_if__srv__GroundedSAM2Interface_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_robot_if__srv__GroundedSAM2Interface_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence__init(tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_robot_if__srv__GroundedSAM2Interface_Request * data = NULL;

  if (size) {
    data = (tm_robot_if__srv__GroundedSAM2Interface_Request *)allocator.zero_allocate(size, sizeof(tm_robot_if__srv__GroundedSAM2Interface_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_robot_if__srv__GroundedSAM2Interface_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_robot_if__srv__GroundedSAM2Interface_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence__fini(tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tm_robot_if__srv__GroundedSAM2Interface_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence *
tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence * array = (tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence *)allocator.allocate(sizeof(tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence__destroy(tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence__are_equal(const tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence * lhs, const tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_robot_if__srv__GroundedSAM2Interface_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence__copy(
  const tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence * input,
  tm_robot_if__srv__GroundedSAM2Interface_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_robot_if__srv__GroundedSAM2Interface_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_robot_if__srv__GroundedSAM2Interface_Request * data =
      (tm_robot_if__srv__GroundedSAM2Interface_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_robot_if__srv__GroundedSAM2Interface_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_robot_if__srv__GroundedSAM2Interface_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_robot_if__srv__GroundedSAM2Interface_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `binary_image`
// already included above
// #include "sensor_msgs/msg/detail/image__functions.h"
// Member `bbox`
// Member `score`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `seg`
// Member `label`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
tm_robot_if__srv__GroundedSAM2Interface_Response__init(tm_robot_if__srv__GroundedSAM2Interface_Response * msg)
{
  if (!msg) {
    return false;
  }
  // binary_image
  if (!sensor_msgs__msg__Image__init(&msg->binary_image)) {
    tm_robot_if__srv__GroundedSAM2Interface_Response__fini(msg);
    return false;
  }
  // bbox
  if (!rosidl_runtime_c__float__Sequence__init(&msg->bbox, 0)) {
    tm_robot_if__srv__GroundedSAM2Interface_Response__fini(msg);
    return false;
  }
  // seg
  if (!rosidl_runtime_c__String__Sequence__init(&msg->seg, 0)) {
    tm_robot_if__srv__GroundedSAM2Interface_Response__fini(msg);
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__Sequence__init(&msg->label, 0)) {
    tm_robot_if__srv__GroundedSAM2Interface_Response__fini(msg);
    return false;
  }
  // score
  if (!rosidl_runtime_c__float__Sequence__init(&msg->score, 0)) {
    tm_robot_if__srv__GroundedSAM2Interface_Response__fini(msg);
    return false;
  }
  return true;
}

void
tm_robot_if__srv__GroundedSAM2Interface_Response__fini(tm_robot_if__srv__GroundedSAM2Interface_Response * msg)
{
  if (!msg) {
    return;
  }
  // binary_image
  sensor_msgs__msg__Image__fini(&msg->binary_image);
  // bbox
  rosidl_runtime_c__float__Sequence__fini(&msg->bbox);
  // seg
  rosidl_runtime_c__String__Sequence__fini(&msg->seg);
  // label
  rosidl_runtime_c__String__Sequence__fini(&msg->label);
  // score
  rosidl_runtime_c__float__Sequence__fini(&msg->score);
}

bool
tm_robot_if__srv__GroundedSAM2Interface_Response__are_equal(const tm_robot_if__srv__GroundedSAM2Interface_Response * lhs, const tm_robot_if__srv__GroundedSAM2Interface_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // binary_image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->binary_image), &(rhs->binary_image)))
  {
    return false;
  }
  // bbox
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->bbox), &(rhs->bbox)))
  {
    return false;
  }
  // seg
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->seg), &(rhs->seg)))
  {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // score
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->score), &(rhs->score)))
  {
    return false;
  }
  return true;
}

bool
tm_robot_if__srv__GroundedSAM2Interface_Response__copy(
  const tm_robot_if__srv__GroundedSAM2Interface_Response * input,
  tm_robot_if__srv__GroundedSAM2Interface_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // binary_image
  if (!sensor_msgs__msg__Image__copy(
      &(input->binary_image), &(output->binary_image)))
  {
    return false;
  }
  // bbox
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->bbox), &(output->bbox)))
  {
    return false;
  }
  // seg
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->seg), &(output->seg)))
  {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // score
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->score), &(output->score)))
  {
    return false;
  }
  return true;
}

tm_robot_if__srv__GroundedSAM2Interface_Response *
tm_robot_if__srv__GroundedSAM2Interface_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_robot_if__srv__GroundedSAM2Interface_Response * msg = (tm_robot_if__srv__GroundedSAM2Interface_Response *)allocator.allocate(sizeof(tm_robot_if__srv__GroundedSAM2Interface_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tm_robot_if__srv__GroundedSAM2Interface_Response));
  bool success = tm_robot_if__srv__GroundedSAM2Interface_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tm_robot_if__srv__GroundedSAM2Interface_Response__destroy(tm_robot_if__srv__GroundedSAM2Interface_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tm_robot_if__srv__GroundedSAM2Interface_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence__init(tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_robot_if__srv__GroundedSAM2Interface_Response * data = NULL;

  if (size) {
    data = (tm_robot_if__srv__GroundedSAM2Interface_Response *)allocator.zero_allocate(size, sizeof(tm_robot_if__srv__GroundedSAM2Interface_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tm_robot_if__srv__GroundedSAM2Interface_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tm_robot_if__srv__GroundedSAM2Interface_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence__fini(tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tm_robot_if__srv__GroundedSAM2Interface_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence *
tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence * array = (tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence *)allocator.allocate(sizeof(tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence__destroy(tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence__are_equal(const tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence * lhs, const tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tm_robot_if__srv__GroundedSAM2Interface_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence__copy(
  const tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence * input,
  tm_robot_if__srv__GroundedSAM2Interface_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tm_robot_if__srv__GroundedSAM2Interface_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tm_robot_if__srv__GroundedSAM2Interface_Response * data =
      (tm_robot_if__srv__GroundedSAM2Interface_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tm_robot_if__srv__GroundedSAM2Interface_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tm_robot_if__srv__GroundedSAM2Interface_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tm_robot_if__srv__GroundedSAM2Interface_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
