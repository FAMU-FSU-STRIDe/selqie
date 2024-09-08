cmake_minimum_required(VERSION 3.8)
project(odrive_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_msgs REQUIRED)

# add executable
add_library(odrive_ros2_component SHARED src/odrive_can_node.cpp)
ament_target_dependencies(odrive_ros2_component rclcpp rclcpp_components robot_msgs)

# declare component
rclcpp_components_register_node(
  odrive_ros2_component 
  PLUGIN "odrive_ros2::ODriveCanNode"
  EXECUTABLE odrive_can_node
)

# install targets
ament_export_targets(export_odrive_ros2_component)
install(
  TARGETS odrive_ros2_component
  EXPORT export_odrive_ros2_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# build testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
