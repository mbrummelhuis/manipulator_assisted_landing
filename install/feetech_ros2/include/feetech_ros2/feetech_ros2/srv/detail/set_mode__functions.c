// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from feetech_ros2:srv/SetMode.idl
// generated code does not contain a copyright notice
#include "feetech_ros2/srv/detail/set_mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
feetech_ros2__srv__SetMode_Request__init(feetech_ros2__srv__SetMode_Request * msg)
{
  if (!msg) {
    return false;
  }
  // operating_mode
  return true;
}

void
feetech_ros2__srv__SetMode_Request__fini(feetech_ros2__srv__SetMode_Request * msg)
{
  if (!msg) {
    return;
  }
  // operating_mode
}

bool
feetech_ros2__srv__SetMode_Request__are_equal(const feetech_ros2__srv__SetMode_Request * lhs, const feetech_ros2__srv__SetMode_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // operating_mode
  if (lhs->operating_mode != rhs->operating_mode) {
    return false;
  }
  return true;
}

bool
feetech_ros2__srv__SetMode_Request__copy(
  const feetech_ros2__srv__SetMode_Request * input,
  feetech_ros2__srv__SetMode_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // operating_mode
  output->operating_mode = input->operating_mode;
  return true;
}

feetech_ros2__srv__SetMode_Request *
feetech_ros2__srv__SetMode_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2__srv__SetMode_Request * msg = (feetech_ros2__srv__SetMode_Request *)allocator.allocate(sizeof(feetech_ros2__srv__SetMode_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(feetech_ros2__srv__SetMode_Request));
  bool success = feetech_ros2__srv__SetMode_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
feetech_ros2__srv__SetMode_Request__destroy(feetech_ros2__srv__SetMode_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    feetech_ros2__srv__SetMode_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
feetech_ros2__srv__SetMode_Request__Sequence__init(feetech_ros2__srv__SetMode_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2__srv__SetMode_Request * data = NULL;

  if (size) {
    data = (feetech_ros2__srv__SetMode_Request *)allocator.zero_allocate(size, sizeof(feetech_ros2__srv__SetMode_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = feetech_ros2__srv__SetMode_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        feetech_ros2__srv__SetMode_Request__fini(&data[i - 1]);
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
feetech_ros2__srv__SetMode_Request__Sequence__fini(feetech_ros2__srv__SetMode_Request__Sequence * array)
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
      feetech_ros2__srv__SetMode_Request__fini(&array->data[i]);
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

feetech_ros2__srv__SetMode_Request__Sequence *
feetech_ros2__srv__SetMode_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2__srv__SetMode_Request__Sequence * array = (feetech_ros2__srv__SetMode_Request__Sequence *)allocator.allocate(sizeof(feetech_ros2__srv__SetMode_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = feetech_ros2__srv__SetMode_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
feetech_ros2__srv__SetMode_Request__Sequence__destroy(feetech_ros2__srv__SetMode_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    feetech_ros2__srv__SetMode_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
feetech_ros2__srv__SetMode_Request__Sequence__are_equal(const feetech_ros2__srv__SetMode_Request__Sequence * lhs, const feetech_ros2__srv__SetMode_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!feetech_ros2__srv__SetMode_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
feetech_ros2__srv__SetMode_Request__Sequence__copy(
  const feetech_ros2__srv__SetMode_Request__Sequence * input,
  feetech_ros2__srv__SetMode_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(feetech_ros2__srv__SetMode_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    feetech_ros2__srv__SetMode_Request * data =
      (feetech_ros2__srv__SetMode_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!feetech_ros2__srv__SetMode_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          feetech_ros2__srv__SetMode_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!feetech_ros2__srv__SetMode_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
feetech_ros2__srv__SetMode_Response__init(feetech_ros2__srv__SetMode_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
feetech_ros2__srv__SetMode_Response__fini(feetech_ros2__srv__SetMode_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
feetech_ros2__srv__SetMode_Response__are_equal(const feetech_ros2__srv__SetMode_Response * lhs, const feetech_ros2__srv__SetMode_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
feetech_ros2__srv__SetMode_Response__copy(
  const feetech_ros2__srv__SetMode_Response * input,
  feetech_ros2__srv__SetMode_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

feetech_ros2__srv__SetMode_Response *
feetech_ros2__srv__SetMode_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2__srv__SetMode_Response * msg = (feetech_ros2__srv__SetMode_Response *)allocator.allocate(sizeof(feetech_ros2__srv__SetMode_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(feetech_ros2__srv__SetMode_Response));
  bool success = feetech_ros2__srv__SetMode_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
feetech_ros2__srv__SetMode_Response__destroy(feetech_ros2__srv__SetMode_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    feetech_ros2__srv__SetMode_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
feetech_ros2__srv__SetMode_Response__Sequence__init(feetech_ros2__srv__SetMode_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2__srv__SetMode_Response * data = NULL;

  if (size) {
    data = (feetech_ros2__srv__SetMode_Response *)allocator.zero_allocate(size, sizeof(feetech_ros2__srv__SetMode_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = feetech_ros2__srv__SetMode_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        feetech_ros2__srv__SetMode_Response__fini(&data[i - 1]);
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
feetech_ros2__srv__SetMode_Response__Sequence__fini(feetech_ros2__srv__SetMode_Response__Sequence * array)
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
      feetech_ros2__srv__SetMode_Response__fini(&array->data[i]);
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

feetech_ros2__srv__SetMode_Response__Sequence *
feetech_ros2__srv__SetMode_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2__srv__SetMode_Response__Sequence * array = (feetech_ros2__srv__SetMode_Response__Sequence *)allocator.allocate(sizeof(feetech_ros2__srv__SetMode_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = feetech_ros2__srv__SetMode_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
feetech_ros2__srv__SetMode_Response__Sequence__destroy(feetech_ros2__srv__SetMode_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    feetech_ros2__srv__SetMode_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
feetech_ros2__srv__SetMode_Response__Sequence__are_equal(const feetech_ros2__srv__SetMode_Response__Sequence * lhs, const feetech_ros2__srv__SetMode_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!feetech_ros2__srv__SetMode_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
feetech_ros2__srv__SetMode_Response__Sequence__copy(
  const feetech_ros2__srv__SetMode_Response__Sequence * input,
  feetech_ros2__srv__SetMode_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(feetech_ros2__srv__SetMode_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    feetech_ros2__srv__SetMode_Response * data =
      (feetech_ros2__srv__SetMode_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!feetech_ros2__srv__SetMode_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          feetech_ros2__srv__SetMode_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!feetech_ros2__srv__SetMode_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "feetech_ros2/srv/detail/set_mode__functions.h"

bool
feetech_ros2__srv__SetMode_Event__init(feetech_ros2__srv__SetMode_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    feetech_ros2__srv__SetMode_Event__fini(msg);
    return false;
  }
  // request
  if (!feetech_ros2__srv__SetMode_Request__Sequence__init(&msg->request, 0)) {
    feetech_ros2__srv__SetMode_Event__fini(msg);
    return false;
  }
  // response
  if (!feetech_ros2__srv__SetMode_Response__Sequence__init(&msg->response, 0)) {
    feetech_ros2__srv__SetMode_Event__fini(msg);
    return false;
  }
  return true;
}

void
feetech_ros2__srv__SetMode_Event__fini(feetech_ros2__srv__SetMode_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  feetech_ros2__srv__SetMode_Request__Sequence__fini(&msg->request);
  // response
  feetech_ros2__srv__SetMode_Response__Sequence__fini(&msg->response);
}

bool
feetech_ros2__srv__SetMode_Event__are_equal(const feetech_ros2__srv__SetMode_Event * lhs, const feetech_ros2__srv__SetMode_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!feetech_ros2__srv__SetMode_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!feetech_ros2__srv__SetMode_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
feetech_ros2__srv__SetMode_Event__copy(
  const feetech_ros2__srv__SetMode_Event * input,
  feetech_ros2__srv__SetMode_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!feetech_ros2__srv__SetMode_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!feetech_ros2__srv__SetMode_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

feetech_ros2__srv__SetMode_Event *
feetech_ros2__srv__SetMode_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2__srv__SetMode_Event * msg = (feetech_ros2__srv__SetMode_Event *)allocator.allocate(sizeof(feetech_ros2__srv__SetMode_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(feetech_ros2__srv__SetMode_Event));
  bool success = feetech_ros2__srv__SetMode_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
feetech_ros2__srv__SetMode_Event__destroy(feetech_ros2__srv__SetMode_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    feetech_ros2__srv__SetMode_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
feetech_ros2__srv__SetMode_Event__Sequence__init(feetech_ros2__srv__SetMode_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2__srv__SetMode_Event * data = NULL;

  if (size) {
    data = (feetech_ros2__srv__SetMode_Event *)allocator.zero_allocate(size, sizeof(feetech_ros2__srv__SetMode_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = feetech_ros2__srv__SetMode_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        feetech_ros2__srv__SetMode_Event__fini(&data[i - 1]);
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
feetech_ros2__srv__SetMode_Event__Sequence__fini(feetech_ros2__srv__SetMode_Event__Sequence * array)
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
      feetech_ros2__srv__SetMode_Event__fini(&array->data[i]);
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

feetech_ros2__srv__SetMode_Event__Sequence *
feetech_ros2__srv__SetMode_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  feetech_ros2__srv__SetMode_Event__Sequence * array = (feetech_ros2__srv__SetMode_Event__Sequence *)allocator.allocate(sizeof(feetech_ros2__srv__SetMode_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = feetech_ros2__srv__SetMode_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
feetech_ros2__srv__SetMode_Event__Sequence__destroy(feetech_ros2__srv__SetMode_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    feetech_ros2__srv__SetMode_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
feetech_ros2__srv__SetMode_Event__Sequence__are_equal(const feetech_ros2__srv__SetMode_Event__Sequence * lhs, const feetech_ros2__srv__SetMode_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!feetech_ros2__srv__SetMode_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
feetech_ros2__srv__SetMode_Event__Sequence__copy(
  const feetech_ros2__srv__SetMode_Event__Sequence * input,
  feetech_ros2__srv__SetMode_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(feetech_ros2__srv__SetMode_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    feetech_ros2__srv__SetMode_Event * data =
      (feetech_ros2__srv__SetMode_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!feetech_ros2__srv__SetMode_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          feetech_ros2__srv__SetMode_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!feetech_ros2__srv__SetMode_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
