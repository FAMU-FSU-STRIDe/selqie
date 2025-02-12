cmake_minimum_required(VERSION 3.8)
project(local_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sbmpo REQUIRED)

include_directories(include)

add_executable(walking_planner_node src/walking_planner_node.cpp)
ament_target_dependencies(walking_planner_node rclcpp nav_msgs)
target_link_libraries(walking_planner_node sbmpo::sbmpo)

add_executable(swimming_planner_node src/swimming_planner_node.cpp)
ament_target_dependencies(swimming_planner_node rclcpp nav_msgs)

add_executable(jumping_planner_node src/jumping_planner_node.cpp)
ament_target_dependencies(jumping_planner_node rclcpp nav_msgs)

# install targets
install(TARGETS
  walking_planner_node
  swimming_planner_node
  jumping_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
