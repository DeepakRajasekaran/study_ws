// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from irobot_interfaces:msg/HardwareStatus.idl
// generated code does not contain a copyright notice
#include "irobot_interfaces/msg/detail/hardware_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `debug_message`
#include "rosidl_runtime_c/string_functions.h"

bool
irobot_interfaces__msg__HardwareStatus__init(irobot_interfaces__msg__HardwareStatus * msg)
{
  if (!msg) {
    return false;
  }
  // temperature
  // motor_status
  // debug_message
  if (!rosidl_runtime_c__String__init(&msg->debug_message)) {
    irobot_interfaces__msg__HardwareStatus__fini(msg);
    return false;
  }
  return true;
}

void
irobot_interfaces__msg__HardwareStatus__fini(irobot_interfaces__msg__HardwareStatus * msg)
{
  if (!msg) {
    return;
  }
  // temperature
  // motor_status
  // debug_message
  rosidl_runtime_c__String__fini(&msg->debug_message);
}

bool
irobot_interfaces__msg__HardwareStatus__are_equal(const irobot_interfaces__msg__HardwareStatus * lhs, const irobot_interfaces__msg__HardwareStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // temperature
  if (lhs->temperature != rhs->temperature) {
    return false;
  }
  // motor_status
  if (lhs->motor_status != rhs->motor_status) {
    return false;
  }
  // debug_message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->debug_message), &(rhs->debug_message)))
  {
    return false;
  }
  return true;
}

bool
irobot_interfaces__msg__HardwareStatus__copy(
  const irobot_interfaces__msg__HardwareStatus * input,
  irobot_interfaces__msg__HardwareStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // temperature
  output->temperature = input->temperature;
  // motor_status
  output->motor_status = input->motor_status;
  // debug_message
  if (!rosidl_runtime_c__String__copy(
      &(input->debug_message), &(output->debug_message)))
  {
    return false;
  }
  return true;
}

irobot_interfaces__msg__HardwareStatus *
irobot_interfaces__msg__HardwareStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  irobot_interfaces__msg__HardwareStatus * msg = (irobot_interfaces__msg__HardwareStatus *)allocator.allocate(sizeof(irobot_interfaces__msg__HardwareStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(irobot_interfaces__msg__HardwareStatus));
  bool success = irobot_interfaces__msg__HardwareStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
irobot_interfaces__msg__HardwareStatus__destroy(irobot_interfaces__msg__HardwareStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    irobot_interfaces__msg__HardwareStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
irobot_interfaces__msg__HardwareStatus__Sequence__init(irobot_interfaces__msg__HardwareStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  irobot_interfaces__msg__HardwareStatus * data = NULL;

  if (size) {
    data = (irobot_interfaces__msg__HardwareStatus *)allocator.zero_allocate(size, sizeof(irobot_interfaces__msg__HardwareStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = irobot_interfaces__msg__HardwareStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        irobot_interfaces__msg__HardwareStatus__fini(&data[i - 1]);
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
irobot_interfaces__msg__HardwareStatus__Sequence__fini(irobot_interfaces__msg__HardwareStatus__Sequence * array)
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
      irobot_interfaces__msg__HardwareStatus__fini(&array->data[i]);
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

irobot_interfaces__msg__HardwareStatus__Sequence *
irobot_interfaces__msg__HardwareStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  irobot_interfaces__msg__HardwareStatus__Sequence * array = (irobot_interfaces__msg__HardwareStatus__Sequence *)allocator.allocate(sizeof(irobot_interfaces__msg__HardwareStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = irobot_interfaces__msg__HardwareStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
irobot_interfaces__msg__HardwareStatus__Sequence__destroy(irobot_interfaces__msg__HardwareStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    irobot_interfaces__msg__HardwareStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
irobot_interfaces__msg__HardwareStatus__Sequence__are_equal(const irobot_interfaces__msg__HardwareStatus__Sequence * lhs, const irobot_interfaces__msg__HardwareStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!irobot_interfaces__msg__HardwareStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
irobot_interfaces__msg__HardwareStatus__Sequence__copy(
  const irobot_interfaces__msg__HardwareStatus__Sequence * input,
  irobot_interfaces__msg__HardwareStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(irobot_interfaces__msg__HardwareStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    irobot_interfaces__msg__HardwareStatus * data =
      (irobot_interfaces__msg__HardwareStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!irobot_interfaces__msg__HardwareStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          irobot_interfaces__msg__HardwareStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!irobot_interfaces__msg__HardwareStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
