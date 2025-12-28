cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Counter.msg"
  "msg/Velocity.msg"
  "msg/Odom.msg"
  "msg/EncoderRaw.msg"
  "msg/LimitSwitches.msg"
  "msg/ModeSwitch.msg"
  "msg/BnoReading.msg"
  "msg/ActuatorCmd.msg"
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
