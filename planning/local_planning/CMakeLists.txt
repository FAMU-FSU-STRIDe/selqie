cmake_minimum_required(VERSION 3.8)
project(local_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sbmpo REQUIRED)

# include directories
include_directories(include)

# add executables
add_executable(walk_planning_node src/walk_planning_node.cpp)
ament_target_dependencies(walk_planning_node rclcpp std_msgs geometry_msgs nav_msgs)
target_link_libraries(walk_planning_node sbmpo::sbmpo)

add_executable(jump_planning_node src/jump_planning_node.cpp)
ament_target_dependencies(jump_planning_node rclcpp std_msgs geometry_msgs nav_msgs)

add_executable(swim_planning_node src/swim_planning_node.cpp)
ament_target_dependencies(swim_planning_node rclcpp std_msgs geometry_msgs nav_msgs)

add_executable(sink_planning_node src/sink_planning_node.cpp)
ament_target_dependencies(sink_planning_node rclcpp std_msgs geometry_msgs nav_msgs)

add_executable(stand_planning_node src/stand_planning_node.cpp)
ament_target_dependencies(stand_planning_node rclcpp std_msgs geometry_msgs nav_msgs)

install(TARGETS
  walk_planning_node
  jump_planning_node
  swim_planning_node
  sink_planning_node
  stand_planning_node
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
