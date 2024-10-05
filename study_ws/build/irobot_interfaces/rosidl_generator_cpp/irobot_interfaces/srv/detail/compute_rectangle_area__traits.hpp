// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from irobot_interfaces:srv/ComputeRectangleArea.idl
// generated code does not contain a copyright notice

#ifndef IROBOT_INTERFACES__SRV__DETAIL__COMPUTE_RECTANGLE_AREA__TRAITS_HPP_
#define IROBOT_INTERFACES__SRV__DETAIL__COMPUTE_RECTANGLE_AREA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "irobot_interfaces/srv/detail/compute_rectangle_area__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace irobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeRectangleArea_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: length
  {
    out << "length: ";
    rosidl_generator_traits::value_to_yaml(msg.length, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeRectangleArea_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: length
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "length: ";
    rosidl_generator_traits::value_to_yaml(msg.length, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeRectangleArea_Request & msg, bool use_flow_style = false)
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

}  // namespace irobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use irobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const irobot_interfaces::srv::ComputeRectangleArea_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  irobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use irobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const irobot_interfaces::srv::ComputeRectangleArea_Request & msg)
{
  return irobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<irobot_interfaces::srv::ComputeRectangleArea_Request>()
{
  return "irobot_interfaces::srv::ComputeRectangleArea_Request";
}

template<>
inline const char * name<irobot_interfaces::srv::ComputeRectangleArea_Request>()
{
  return "irobot_interfaces/srv/ComputeRectangleArea_Request";
}

template<>
struct has_fixed_size<irobot_interfaces::srv::ComputeRectangleArea_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<irobot_interfaces::srv::ComputeRectangleArea_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<irobot_interfaces::srv::ComputeRectangleArea_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace irobot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeRectangleArea_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: area
  {
    out << "area: ";
    rosidl_generator_traits::value_to_yaml(msg.area, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeRectangleArea_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: area
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "area: ";
    rosidl_generator_traits::value_to_yaml(msg.area, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeRectangleArea_Response & msg, bool use_flow_style = false)
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

}  // namespace irobot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use irobot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const irobot_interfaces::srv::ComputeRectangleArea_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  irobot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use irobot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const irobot_interfaces::srv::ComputeRectangleArea_Response & msg)
{
  return irobot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<irobot_interfaces::srv::ComputeRectangleArea_Response>()
{
  return "irobot_interfaces::srv::ComputeRectangleArea_Response";
}

template<>
inline const char * name<irobot_interfaces::srv::ComputeRectangleArea_Response>()
{
  return "irobot_interfaces/srv/ComputeRectangleArea_Response";
}

template<>
struct has_fixed_size<irobot_interfaces::srv::ComputeRectangleArea_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<irobot_interfaces::srv::ComputeRectangleArea_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<irobot_interfaces::srv::ComputeRectangleArea_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<irobot_interfaces::srv::ComputeRectangleArea>()
{
  return "irobot_interfaces::srv::ComputeRectangleArea";
}

template<>
inline const char * name<irobot_interfaces::srv::ComputeRectangleArea>()
{
  return "irobot_interfaces/srv/ComputeRectangleArea";
}

template<>
struct has_fixed_size<irobot_interfaces::srv::ComputeRectangleArea>
  : std::integral_constant<
    bool,
    has_fixed_size<irobot_interfaces::srv::ComputeRectangleArea_Request>::value &&
    has_fixed_size<irobot_interfaces::srv::ComputeRectangleArea_Response>::value
  >
{
};

template<>
struct has_bounded_size<irobot_interfaces::srv::ComputeRectangleArea>
  : std::integral_constant<
    bool,
    has_bounded_size<irobot_interfaces::srv::ComputeRectangleArea_Request>::value &&
    has_bounded_size<irobot_interfaces::srv::ComputeRectangleArea_Response>::value
  >
{
};

template<>
struct is_service<irobot_interfaces::srv::ComputeRectangleArea>
  : std::true_type
{
};

template<>
struct is_service_request<irobot_interfaces::srv::ComputeRectangleArea_Request>
  : std::true_type
{
};

template<>
struct is_service_response<irobot_interfaces::srv::ComputeRectangleArea_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // IROBOT_INTERFACES__SRV__DETAIL__COMPUTE_RECTANGLE_AREA__TRAITS_HPP_
