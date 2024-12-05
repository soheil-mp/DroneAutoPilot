cmake_minimum_required(VERSION 3.8)
project(drone_nav_rl_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionStatus.msg"
  "srv/LoadMission.srv"
  "srv/AbortMission.srv"
  "srv/OptimizePath.srv"
  "action/ExecuteMission.action"
  DEPENDENCIES geometry_msgs action_msgs
)

ament_package() 