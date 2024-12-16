cmake_minimum_required(VERSION 3.8)
project(modbus_client_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Find the libmodbus library
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

include_directories(
  include
  ${LIBMODBUS_INCLUDE_DIRS}
)

add_executable(modbus_tcp_client src/modbus_tcp_client.cpp)

target_link_libraries(modbus_tcp_client
  ${LIBMODBUS_LIBRARIES}
)

ament_target_dependencies(modbus_tcp_client rclcpp)

install(TARGETS
  modbus_tcp_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
