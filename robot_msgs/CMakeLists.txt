cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanFrame.msg"
  "msg/MotorCommand.msg"
  "msg/MotorEstimate.msg"
  "msg/ODriveConfig.msg"
  "msg/ODriveInfo.msg"
  "msg/LegCommand.msg"
  "msg/LegEstimate.msg"
  "msg/LegTrajectory.msg"
  "msg/BodyTrajectory.msg"
  "msg/FootholdTrajectory.msg"
  "msg/FootholdState.msg"
  "msg/StancePattern.msg"
  "msg/GaitTrajectory.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
