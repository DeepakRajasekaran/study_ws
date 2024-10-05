// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from irobot_interfaces:srv/ComputeRectangleArea.idl
// generated code does not contain a copyright notice

#ifndef IROBOT_INTERFACES__SRV__DETAIL__COMPUTE_RECTANGLE_AREA__STRUCT_H_
#define IROBOT_INTERFACES__SRV__DETAIL__COMPUTE_RECTANGLE_AREA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ComputeRectangleArea in the package irobot_interfaces.
typedef struct irobot_interfaces__srv__ComputeRectangleArea_Request
{
  double length;
  double width;
} irobot_interfaces__srv__ComputeRectangleArea_Request;

// Struct for a sequence of irobot_interfaces__srv__ComputeRectangleArea_Request.
typedef struct irobot_interfaces__srv__ComputeRectangleArea_Request__Sequence
{
  irobot_interfaces__srv__ComputeRectangleArea_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} irobot_interfaces__srv__ComputeRectangleArea_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ComputeRectangleArea in the package irobot_interfaces.
typedef struct irobot_interfaces__srv__ComputeRectangleArea_Response
{
  double area;
} irobot_interfaces__srv__ComputeRectangleArea_Response;

// Struct for a sequence of irobot_interfaces__srv__ComputeRectangleArea_Response.
typedef struct irobot_interfaces__srv__ComputeRectangleArea_Response__Sequence
{
  irobot_interfaces__srv__ComputeRectangleArea_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} irobot_interfaces__srv__ComputeRectangleArea_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IROBOT_INTERFACES__SRV__DETAIL__COMPUTE_RECTANGLE_AREA__STRUCT_H_
