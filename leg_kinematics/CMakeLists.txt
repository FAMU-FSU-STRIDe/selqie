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

# add executable
add_executable(fivebar2d_node src/fivebar2d_node.cpp)
ament_target_dependencies(fivebar2d_node rclcpp robot_msgs)
target_link_libraries(fivebar2d_node Eigen3::Eigen)

# install targets
install(TARGETS
fivebar2d_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch and models files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
