cmake_minimum_required(VERSION 3.8)
project(irobot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  #"msg/HardwareStatus.msg"
  #"msg/LEDStates.msg"
  "msg/Turtleinfo.msg"
  "msg/TurtleArray.msg"
  "srv/KillSwitch.srv"
  #"srv/RectangleParameters.srv"
  #"srv/SetLed.srv"
)

ament_package()
