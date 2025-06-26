// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from feetech_ros2:srv/SetMode.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "feetech_ros2/srv/set_mode.hpp"


#ifndef FEETECH_ROS2__SRV__DETAIL__SET_MODE__STRUCT_HPP_
#define FEETECH_ROS2__SRV__DETAIL__SET_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__feetech_ros2__srv__SetMode_Request __attribute__((deprecated))
#else
# define DEPRECATED__feetech_ros2__srv__SetMode_Request __declspec(deprecated)
#endif

namespace feetech_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMode_Request_
{
  using Type = SetMode_Request_<ContainerAllocator>;

  explicit SetMode_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operating_mode = 0ll;
    }
  }

  explicit SetMode_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->operating_mode = 0ll;
    }
  }

  // field types and members
  using _operating_mode_type =
    int64_t;
  _operating_mode_type operating_mode;

  // setters for named parameter idiom
  Type & set__operating_mode(
    const int64_t & _arg)
  {
    this->operating_mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    feetech_ros2::srv::SetMode_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const feetech_ros2::srv::SetMode_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<feetech_ros2::srv::SetMode_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<feetech_ros2::srv::SetMode_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      feetech_ros2::srv::SetMode_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<feetech_ros2::srv::SetMode_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      feetech_ros2::srv::SetMode_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<feetech_ros2::srv::SetMode_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<feetech_ros2::srv::SetMode_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<feetech_ros2::srv::SetMode_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__feetech_ros2__srv__SetMode_Request
    std::shared_ptr<feetech_ros2::srv::SetMode_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__feetech_ros2__srv__SetMode_Request
    std::shared_ptr<feetech_ros2::srv::SetMode_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_Request_ & other) const
  {
    if (this->operating_mode != other.operating_mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_Request_

// alias to use template instance with default allocator
using SetMode_Request =
  feetech_ros2::srv::SetMode_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace feetech_ros2


#ifndef _WIN32
# define DEPRECATED__feetech_ros2__srv__SetMode_Response __attribute__((deprecated))
#else
# define DEPRECATED__feetech_ros2__srv__SetMode_Response __declspec(deprecated)
#endif

namespace feetech_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMode_Response_
{
  using Type = SetMode_Response_<ContainerAllocator>;

  explicit SetMode_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetMode_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    feetech_ros2::srv::SetMode_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const feetech_ros2::srv::SetMode_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<feetech_ros2::srv::SetMode_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<feetech_ros2::srv::SetMode_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      feetech_ros2::srv::SetMode_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<feetech_ros2::srv::SetMode_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      feetech_ros2::srv::SetMode_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<feetech_ros2::srv::SetMode_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<feetech_ros2::srv::SetMode_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<feetech_ros2::srv::SetMode_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__feetech_ros2__srv__SetMode_Response
    std::shared_ptr<feetech_ros2::srv::SetMode_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__feetech_ros2__srv__SetMode_Response
    std::shared_ptr<feetech_ros2::srv::SetMode_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_Response_

// alias to use template instance with default allocator
using SetMode_Response =
  feetech_ros2::srv::SetMode_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace feetech_ros2


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__feetech_ros2__srv__SetMode_Event __attribute__((deprecated))
#else
# define DEPRECATED__feetech_ros2__srv__SetMode_Event __declspec(deprecated)
#endif

namespace feetech_ros2
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetMode_Event_
{
  using Type = SetMode_Event_<ContainerAllocator>;

  explicit SetMode_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SetMode_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<feetech_ros2::srv::SetMode_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<feetech_ros2::srv::SetMode_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<feetech_ros2::srv::SetMode_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<feetech_ros2::srv::SetMode_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<feetech_ros2::srv::SetMode_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<feetech_ros2::srv::SetMode_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<feetech_ros2::srv::SetMode_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<feetech_ros2::srv::SetMode_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    feetech_ros2::srv::SetMode_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const feetech_ros2::srv::SetMode_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<feetech_ros2::srv::SetMode_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<feetech_ros2::srv::SetMode_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      feetech_ros2::srv::SetMode_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<feetech_ros2::srv::SetMode_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      feetech_ros2::srv::SetMode_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<feetech_ros2::srv::SetMode_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<feetech_ros2::srv::SetMode_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<feetech_ros2::srv::SetMode_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__feetech_ros2__srv__SetMode_Event
    std::shared_ptr<feetech_ros2::srv::SetMode_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__feetech_ros2__srv__SetMode_Event
    std::shared_ptr<feetech_ros2::srv::SetMode_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetMode_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetMode_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetMode_Event_

// alias to use template instance with default allocator
using SetMode_Event =
  feetech_ros2::srv::SetMode_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace feetech_ros2

namespace feetech_ros2
{

namespace srv
{

struct SetMode
{
  using Request = feetech_ros2::srv::SetMode_Request;
  using Response = feetech_ros2::srv::SetMode_Response;
  using Event = feetech_ros2::srv::SetMode_Event;
};

}  // namespace srv

}  // namespace feetech_ros2

#endif  // FEETECH_ROS2__SRV__DETAIL__SET_MODE__STRUCT_HPP_
