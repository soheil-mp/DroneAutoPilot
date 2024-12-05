cmake_minimum_required(VERSION 3.8)
project(skypilot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/drone_controller_node.py
  scripts/mission_controller_node.py
  scripts/path_planner_node.py
  scripts/visualization_node.py
  scripts/train_rl_agent.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install web interface
install(DIRECTORY
  web
  DESTINATION share/${PROJECT_NAME}
)

# Install mesh files
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Find test dependencies
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  
  # Add linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add Python tests
  ament_add_pytest_test(${PROJECT_NAME}_pytest tests
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 300
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
  
  # Add launch tests
  add_launch_test(
    tests/fixtures/test_launch.py
    TARGET ${PROJECT_NAME}_launch_test
    TIMEOUT 300
  )
endif()

ament_package() 