cmake_minimum_required(VERSION 3.8)
project(legged_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_msgs REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# OSQP
find_package(osqp REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_executable(legged_mpc_node src/legged_mpc_node.cpp)
ament_target_dependencies(legged_mpc_node rclcpp nav_msgs robot_msgs)
target_link_libraries(legged_mpc_node Eigen3::Eigen osqp::osqp)

add_executable(body_trajectory_node src/body_trajectory_node.cpp)
ament_target_dependencies(body_trajectory_node rclcpp nav_msgs robot_msgs)

add_executable(foothold_planner_node src/foothold_planner_node.cpp)
ament_target_dependencies(foothold_planner_node rclcpp robot_msgs)
target_link_libraries(foothold_planner_node Eigen3::Eigen)

# install targets
install(TARGETS
  legged_mpc_node
  body_trajectory_node
  foothold_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
