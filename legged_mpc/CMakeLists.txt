cmake_minimum_required(VERSION 3.8)
project(legged_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)

find_package(osqp REQUIRED)

include_directories(include)

add_executable(legged_mpc_node src/legged_mpc_node.cpp)
ament_target_dependencies(legged_mpc_node rclcpp robot_msgs)
target_link_libraries(legged_mpc_node osqp::osqp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
