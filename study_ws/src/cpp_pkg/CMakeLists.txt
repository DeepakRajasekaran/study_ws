cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(irobot_interfaces REQUIRED)

#add_executable(cpp_node src/first_node.cpp)
#ament_target_dependencies(cpp_node rclcpp)

add_executable(news_station src/news_station.cpp)
ament_target_dependencies(news_station rclcpp std_msgs)

add_executable(radio src/radio.cpp)
ament_target_dependencies(radio rclcpp std_msgs)

add_executable(NumberPublisher src/NumberPublisher.cpp)
ament_target_dependencies(NumberPublisher rclcpp std_msgs)

add_executable(NumberCounter src/NumberCounter.cpp)
ament_target_dependencies(NumberCounter rclcpp std_msgs std_srvs)

#add_executable(sumServer src/sumServer.cpp)
#ament_target_dependencies(sumServer rclcpp example_interfaces)

#add_executable(client_sumServer_noop src/client_sumServer_noop.cpp)
#ament_target_dependencies(client_sumServer_noop rclcpp example_interfaces)

#add_executable(client_sumServer src/client_sumServer.cpp)
#ament_target_dependencies(client_sumServer rclcpp example_interfaces)

add_executable(HardwareStatusPublisher src/hw_statusPublisher.cpp)
ament_target_dependencies(HardwareStatusPublisher rclcpp irobot_interfaces)

add_executable(findRectangleArea_server src/findRectangleArea_server.cpp)
ament_target_dependencies(findRectangleArea_server rclcpp irobot_interfaces)

add_executable(client_RectangleParam src/client_RectangleParam.cpp)
ament_target_dependencies(client_RectangleParam rclcpp irobot_interfaces)

add_executable(activity004_server src/activity004_server.cpp)
ament_target_dependencies(activity004_server rclcpp irobot_interfaces)

add_executable(activity004_client src/activity004_client.cpp)
ament_target_dependencies(activity004_client rclcpp irobot_interfaces)

install(TARGETS
#  cpp_node
  news_station
  radio
  NumberPublisher
  NumberCounter
#  client_sumServer
#  client_sumServer_noop
#  sumServer
  HardwareStatusPublisher
  findRectangleArea_server
  client_RectangleParam
  # Add the executable name here
  activity004_server
  activity004_client
  DESTINATION lib/${PROJECT_NAME}
) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
