cmake_minimum_required(VERSION 3.8)
project(stride_generation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(leg_control_msgs REQUIRED)

# include directories
include_directories(include)

# add executables
add_executable(walk_stride_node src/walk_stride_node.cpp)
ament_target_dependencies(walk_stride_node rclcpp std_msgs geometry_msgs leg_control_msgs)

add_executable(jump_stride_node src/jump_stride_node.cpp)
ament_target_dependencies(jump_stride_node rclcpp std_msgs geometry_msgs leg_control_msgs)

add_executable(swim_stride_node src/swim_stride_node.cpp)
ament_target_dependencies(swim_stride_node rclcpp std_msgs geometry_msgs leg_control_msgs)

add_executable(sink_stride_node src/sink_stride_node.cpp)
ament_target_dependencies(sink_stride_node rclcpp std_msgs geometry_msgs leg_control_msgs)

add_executable(stand_stride_node src/stand_stride_node.cpp)
ament_target_dependencies(stand_stride_node rclcpp std_msgs geometry_msgs leg_control_msgs)

install(TARGETS
  walk_stride_node
  jump_stride_node
  swim_stride_node
  sink_stride_node
  stand_stride_node
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
