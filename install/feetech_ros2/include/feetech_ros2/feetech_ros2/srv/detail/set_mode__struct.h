// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from feetech_ros2:srv/SetMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "feetech_ros2/srv/set_mode.h"


#ifndef FEETECH_ROS2__SRV__DETAIL__SET_MODE__STRUCT_H_
#define FEETECH_ROS2__SRV__DETAIL__SET_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetMode in the package feetech_ros2.
typedef struct feetech_ros2__srv__SetMode_Request
{
  int64_t operating_mode;
} feetech_ros2__srv__SetMode_Request;

// Struct for a sequence of feetech_ros2__srv__SetMode_Request.
typedef struct feetech_ros2__srv__SetMode_Request__Sequence
{
  feetech_ros2__srv__SetMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} feetech_ros2__srv__SetMode_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/SetMode in the package feetech_ros2.
typedef struct feetech_ros2__srv__SetMode_Response
{
  bool success;
} feetech_ros2__srv__SetMode_Response;

// Struct for a sequence of feetech_ros2__srv__SetMode_Response.
typedef struct feetech_ros2__srv__SetMode_Response__Sequence
{
  feetech_ros2__srv__SetMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} feetech_ros2__srv__SetMode_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  feetech_ros2__srv__SetMode_Event__request__MAX_SIZE = 1
};
// response
enum
{
  feetech_ros2__srv__SetMode_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetMode in the package feetech_ros2.
typedef struct feetech_ros2__srv__SetMode_Event
{
  service_msgs__msg__ServiceEventInfo info;
  feetech_ros2__srv__SetMode_Request__Sequence request;
  feetech_ros2__srv__SetMode_Response__Sequence response;
} feetech_ros2__srv__SetMode_Event;

// Struct for a sequence of feetech_ros2__srv__SetMode_Event.
typedef struct feetech_ros2__srv__SetMode_Event__Sequence
{
  feetech_ros2__srv__SetMode_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} feetech_ros2__srv__SetMode_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FEETECH_ROS2__SRV__DETAIL__SET_MODE__STRUCT_H_
