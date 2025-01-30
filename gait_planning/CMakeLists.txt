cmake_minimum_required(VERSION 3.8)
project(gait_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sbmpo REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(gait_planning_node src/gait_planning_node.cpp)
ament_target_dependencies(gait_planning_node rclcpp nav_msgs)
target_link_libraries(gait_planning_node sbmpo::sbmpo)

install(TARGETS
  gait_planning_node
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
