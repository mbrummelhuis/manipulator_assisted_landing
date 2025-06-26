// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from feetech_ros2:srv/SetMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "feetech_ros2/srv/set_mode.hpp"


#ifndef FEETECH_ROS2__SRV__DETAIL__SET_MODE__TRAITS_HPP_
#define FEETECH_ROS2__SRV__DETAIL__SET_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "feetech_ros2/srv/detail/set_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace feetech_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetMode_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: operating_mode
  {
    out << "operating_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.operating_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: operating_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "operating_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.operating_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetMode_Request & msg, bool use_flow_style = false)
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

}  // namespace feetech_ros2

namespace rosidl_generator_traits
{

[[deprecated("use feetech_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const feetech_ros2::srv::SetMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  feetech_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use feetech_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const feetech_ros2::srv::SetMode_Request & msg)
{
  return feetech_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<feetech_ros2::srv::SetMode_Request>()
{
  return "feetech_ros2::srv::SetMode_Request";
}

template<>
inline const char * name<feetech_ros2::srv::SetMode_Request>()
{
  return "feetech_ros2/srv/SetMode_Request";
}

template<>
struct has_fixed_size<feetech_ros2::srv::SetMode_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<feetech_ros2::srv::SetMode_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<feetech_ros2::srv::SetMode_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace feetech_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetMode_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetMode_Response & msg, bool use_flow_style = false)
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

}  // namespace feetech_ros2

namespace rosidl_generator_traits
{

[[deprecated("use feetech_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const feetech_ros2::srv::SetMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  feetech_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use feetech_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const feetech_ros2::srv::SetMode_Response & msg)
{
  return feetech_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<feetech_ros2::srv::SetMode_Response>()
{
  return "feetech_ros2::srv::SetMode_Response";
}

template<>
inline const char * name<feetech_ros2::srv::SetMode_Response>()
{
  return "feetech_ros2/srv/SetMode_Response";
}

template<>
struct has_fixed_size<feetech_ros2::srv::SetMode_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<feetech_ros2::srv::SetMode_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<feetech_ros2::srv::SetMode_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace feetech_ros2
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetMode_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
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
  const SetMode_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetMode_Event & msg, bool use_flow_style = false)
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

}  // namespace feetech_ros2

namespace rosidl_generator_traits
{

[[deprecated("use feetech_ros2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const feetech_ros2::srv::SetMode_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  feetech_ros2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use feetech_ros2::srv::to_yaml() instead")]]
inline std::string to_yaml(const feetech_ros2::srv::SetMode_Event & msg)
{
  return feetech_ros2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<feetech_ros2::srv::SetMode_Event>()
{
  return "feetech_ros2::srv::SetMode_Event";
}

template<>
inline const char * name<feetech_ros2::srv::SetMode_Event>()
{
  return "feetech_ros2/srv/SetMode_Event";
}

template<>
struct has_fixed_size<feetech_ros2::srv::SetMode_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<feetech_ros2::srv::SetMode_Event>
  : std::integral_constant<bool, has_bounded_size<feetech_ros2::srv::SetMode_Request>::value && has_bounded_size<feetech_ros2::srv::SetMode_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<feetech_ros2::srv::SetMode_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<feetech_ros2::srv::SetMode>()
{
  return "feetech_ros2::srv::SetMode";
}

template<>
inline const char * name<feetech_ros2::srv::SetMode>()
{
  return "feetech_ros2/srv/SetMode";
}

template<>
struct has_fixed_size<feetech_ros2::srv::SetMode>
  : std::integral_constant<
    bool,
    has_fixed_size<feetech_ros2::srv::SetMode_Request>::value &&
    has_fixed_size<feetech_ros2::srv::SetMode_Response>::value
  >
{
};

template<>
struct has_bounded_size<feetech_ros2::srv::SetMode>
  : std::integral_constant<
    bool,
    has_bounded_size<feetech_ros2::srv::SetMode_Request>::value &&
    has_bounded_size<feetech_ros2::srv::SetMode_Response>::value
  >
{
};

template<>
struct is_service<feetech_ros2::srv::SetMode>
  : std::true_type
{
};

template<>
struct is_service_request<feetech_ros2::srv::SetMode_Request>
  : std::true_type
{
};

template<>
struct is_service_response<feetech_ros2::srv::SetMode_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // FEETECH_ROS2__SRV__DETAIL__SET_MODE__TRAITS_HPP_
