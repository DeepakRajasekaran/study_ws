// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from irobot_interfaces:msg/HardwareStatus.idl
// generated code does not contain a copyright notice

#ifndef IROBOT_INTERFACES__MSG__DETAIL__HARDWARE_STATUS__BUILDER_HPP_
#define IROBOT_INTERFACES__MSG__DETAIL__HARDWARE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "irobot_interfaces/msg/detail/hardware_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace irobot_interfaces
{

namespace msg
{

namespace builder
{

class Init_HardwareStatus_debug_message
{
public:
  explicit Init_HardwareStatus_debug_message(::irobot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  ::irobot_interfaces::msg::HardwareStatus debug_message(::irobot_interfaces::msg::HardwareStatus::_debug_message_type arg)
  {
    msg_.debug_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::irobot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_motor_status
{
public:
  explicit Init_HardwareStatus_motor_status(::irobot_interfaces::msg::HardwareStatus & msg)
  : msg_(msg)
  {}
  Init_HardwareStatus_debug_message motor_status(::irobot_interfaces::msg::HardwareStatus::_motor_status_type arg)
  {
    msg_.motor_status = std::move(arg);
    return Init_HardwareStatus_debug_message(msg_);
  }

private:
  ::irobot_interfaces::msg::HardwareStatus msg_;
};

class Init_HardwareStatus_temperature
{
public:
  Init_HardwareStatus_temperature()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HardwareStatus_motor_status temperature(::irobot_interfaces::msg::HardwareStatus::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_HardwareStatus_motor_status(msg_);
  }

private:
  ::irobot_interfaces::msg::HardwareStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::irobot_interfaces::msg::HardwareStatus>()
{
  return irobot_interfaces::msg::builder::Init_HardwareStatus_temperature();
}

}  // namespace irobot_interfaces

#endif  // IROBOT_INTERFACES__MSG__DETAIL__HARDWARE_STATUS__BUILDER_HPP_
