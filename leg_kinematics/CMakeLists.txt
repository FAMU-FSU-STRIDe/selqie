cmake_minimum_required(VERSION 3.8)
project(leg_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_msgs REQUIRED)

# Eigen3
find_package(Eigen3 REQUIRED)

# include directories
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

# add executable
add_library(leg_kinematics_component SHARED 
  src/fivebar2d_node.cpp 
  src/unitree_a1_leg_node.cpp
)
ament_target_dependencies(leg_kinematics_component rclcpp rclcpp_components robot_msgs)
target_link_libraries(leg_kinematics_component Eigen3::Eigen)

# declare component
rclcpp_components_register_node(
  leg_kinematics_component 
  PLUGIN "leg_kinematics::FiveBar2DNode"
  EXECUTABLE fivebar2d_node
)

rclcpp_components_register_node(
  leg_kinematics_component 
  PLUGIN "leg_kinematics::UnitreeA1LegNode"
  EXECUTABLE unitree_a1_leg_node
)

# install targets
ament_export_targets(export_leg_kinematics_component)
install(
  TARGETS leg_kinematics_component
  EXPORT export_leg_kinematics_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
