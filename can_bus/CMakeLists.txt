cmake_minimum_required(VERSION 3.8)
project(can_bus)

# compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_msgs REQUIRED)

# add library
add_library(can_bus_component SHARED src/can_bus_node.cpp)
ament_target_dependencies(can_bus_component rclcpp rclcpp_components robot_msgs)

# declare component
rclcpp_components_register_node(
  can_bus_component 
  PLUGIN "can_bus::CanBusNode"
  EXECUTABLE can_bus_node
)

# install targets
ament_export_targets(export_can_bus_component)
install(
  TARGETS can_bus_component
  EXPORT export_can_bus_component
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
