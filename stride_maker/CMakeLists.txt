cmake_minimum_required(VERSION 3.8)
project(stride_maker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)

include_directories(include)

add_executable(command_velocity_filter_node src/command_velocity_filter_node.cpp)
ament_target_dependencies(command_velocity_filter_node rclcpp geometry_msgs)

add_executable(walk_node src/walk_node.cpp)
ament_target_dependencies(walk_node rclcpp robot_msgs)

add_executable(swim_node src/swim_node.cpp)
ament_target_dependencies(swim_node rclcpp robot_msgs)

add_executable(jump_node src/jump_node.cpp)
ament_target_dependencies(jump_node rclcpp robot_msgs)

add_executable(sink_node src/sink_node.cpp)
ament_target_dependencies(sink_node rclcpp robot_msgs)

add_executable(stand_node src/stand_node.cpp)
ament_target_dependencies(stand_node rclcpp robot_msgs)

install(TARGETS
  command_velocity_filter_node
  walk_node
  swim_node
  jump_node
  sink_node
  stand_node
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
