// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from feetech_ros2:srv/SetMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "feetech_ros2/srv/set_mode.hpp"


#ifndef FEETECH_ROS2__SRV__DETAIL__SET_MODE__BUILDER_HPP_
#define FEETECH_ROS2__SRV__DETAIL__SET_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "feetech_ros2/srv/detail/set_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace feetech_ros2
{

namespace srv
{

namespace builder
{

class Init_SetMode_Request_operating_mode
{
public:
  Init_SetMode_Request_operating_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::feetech_ros2::srv::SetMode_Request operating_mode(::feetech_ros2::srv::SetMode_Request::_operating_mode_type arg)
  {
    msg_.operating_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::feetech_ros2::srv::SetMode_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::feetech_ros2::srv::SetMode_Request>()
{
  return feetech_ros2::srv::builder::Init_SetMode_Request_operating_mode();
}

}  // namespace feetech_ros2


namespace feetech_ros2
{

namespace srv
{

namespace builder
{

class Init_SetMode_Response_success
{
public:
  Init_SetMode_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::feetech_ros2::srv::SetMode_Response success(::feetech_ros2::srv::SetMode_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::feetech_ros2::srv::SetMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::feetech_ros2::srv::SetMode_Response>()
{
  return feetech_ros2::srv::builder::Init_SetMode_Response_success();
}

}  // namespace feetech_ros2


namespace feetech_ros2
{

namespace srv
{

namespace builder
{

class Init_SetMode_Event_response
{
public:
  explicit Init_SetMode_Event_response(::feetech_ros2::srv::SetMode_Event & msg)
  : msg_(msg)
  {}
  ::feetech_ros2::srv::SetMode_Event response(::feetech_ros2::srv::SetMode_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::feetech_ros2::srv::SetMode_Event msg_;
};

class Init_SetMode_Event_request
{
public:
  explicit Init_SetMode_Event_request(::feetech_ros2::srv::SetMode_Event & msg)
  : msg_(msg)
  {}
  Init_SetMode_Event_response request(::feetech_ros2::srv::SetMode_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetMode_Event_response(msg_);
  }

private:
  ::feetech_ros2::srv::SetMode_Event msg_;
};

class Init_SetMode_Event_info
{
public:
  Init_SetMode_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetMode_Event_request info(::feetech_ros2::srv::SetMode_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetMode_Event_request(msg_);
  }

private:
  ::feetech_ros2::srv::SetMode_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::feetech_ros2::srv::SetMode_Event>()
{
  return feetech_ros2::srv::builder::Init_SetMode_Event_info();
}

}  // namespace feetech_ros2

#endif  // FEETECH_ROS2__SRV__DETAIL__SET_MODE__BUILDER_HPP_
