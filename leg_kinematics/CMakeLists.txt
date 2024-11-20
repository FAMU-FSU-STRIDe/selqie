cmake_minimum_required(VERSION 3.8)
project(leg_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msgs REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# include directories
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

# add library
add_executable(fivebar2d_node src/fivebar2d_node.cpp)
ament_target_dependencies(fivebar2d_node rclcpp robot_msgs)
target_link_libraries(fivebar2d_node Eigen3::Eigen)

# add library
add_executable(unitree_a1_leg_node src/unitree_a1_leg_node.cpp)
ament_target_dependencies(unitree_a1_leg_node rclcpp robot_msgs)
target_link_libraries(unitree_a1_leg_node Eigen3::Eigen)

# add library
add_executable(rr2d_node src/rr2d_node.cpp)
ament_target_dependencies(rr2d_node rclcpp robot_msgs)
target_link_libraries(rr2d_node Eigen3::Eigen)

# add executable
add_executable(leg_trajectory_publisher_node src/leg_trajectory_publisher_node.cpp)
ament_target_dependencies(leg_trajectory_publisher_node rclcpp robot_msgs)

# install targets
install(TARGETS
  fivebar2d_node
  unitree_a1_leg_node
  rr2d_node
  leg_trajectory_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
