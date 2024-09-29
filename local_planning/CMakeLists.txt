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
find_package(grid_map_ros REQUIRED)

include_directories(include)

add_executable(walking_planner_node src/walking_planner_node.cpp)
ament_target_dependencies(walking_planner_node rclcpp nav_msgs grid_map_ros)
target_link_libraries(walking_planner_node sbmpo::sbmpo)

add_executable(test_grid_map_node src/test_grid_map_node.cpp)
ament_target_dependencies(test_grid_map_node rclcpp nav_msgs grid_map_ros)

# install targets
install(TARGETS
  walking_planner_node
  test_grid_map_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
