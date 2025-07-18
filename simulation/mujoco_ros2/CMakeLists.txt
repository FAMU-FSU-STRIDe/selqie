cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2)

# Set OpenGL preference to GLVND
set(OpenGL_GL_PREFERENCE "GLVND")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(actuation_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Mujoco dependencies
find_package(mujoco REQUIRED)
find_package(glfw3 3.3 REQUIRED)

# include directories
include_directories(include)

# add executable
add_executable(mujoco_node src/mujoco_node.cpp)
ament_target_dependencies(mujoco_node rclcpp actuation_msgs nav_msgs tf2_ros)
target_link_libraries(mujoco_node mujoco::mujoco glfw)

# install targets
install(TARGETS
  mujoco_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
