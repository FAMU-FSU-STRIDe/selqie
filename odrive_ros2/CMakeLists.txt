cmake_minimum_required(VERSION 3.8)
project(odrive_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)

# add executable
add_executable(odrive_can_node src/odrive_can_node.cpp)
ament_target_dependencies(odrive_can_node rclcpp robot_msgs)

add_executable(odrive_cmd_node src/odrive_cmd_node.cpp)
ament_target_dependencies(odrive_cmd_node rclcpp robot_msgs)

# install targets
install(TARGETS
  odrive_can_node
  odrive_cmd_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# build testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
