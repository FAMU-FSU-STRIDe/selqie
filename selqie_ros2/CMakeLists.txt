cmake_minimum_required(VERSION 3.8)
project(selqie_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/selqie_terminal.py
  scripts/selqie_joystick.py
  scripts/selqie_rviz.py
  scripts/tests/plot_control_data.py
  scripts/tests/plot_depth_data.py
  scripts/tests/plot_imu_data.py
  scripts/tests/test_imu.py
  scripts/tests/test_leg_control.py
  DESTINATION lib/${PROJECT_NAME}
)

# install launch and models files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

install(DIRECTORY trajectories/
  DESTINATION share/${PROJECT_NAME}/trajectories
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
