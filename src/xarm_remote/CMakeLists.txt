cmake_minimum_required(VERSION 3.8)
project(xarm_remote)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTask.action"
  "action/PickPlace.action"
  "action/XarmTask.action"
  DEPENDENCIES geometry_msgs
)

# C++ executables
add_executable(task_server src/task_server.cpp)
ament_target_dependencies(task_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  control_msgs
  trajectory_msgs
)

add_executable(task_server_number src/task_server_number.cpp)
ament_target_dependencies(task_server_number
  rclcpp
  rclcpp_action
  rclcpp_components
  moveit_ros_planning_interface
)

# Link action interfaces to executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(task_server "${cpp_typesupport_target}")
target_link_libraries(task_server_number "${cpp_typesupport_target}")

# Install C++ executables
install(TARGETS
  task_server
  task_server_number
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python module
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python scripts as executables
install(PROGRAMS
  ${PROJECT_NAME}/alexa_interface.py
  scripts/task_client.py
  scripts/task_client_number.py
  scripts/test_integration.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch

  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
