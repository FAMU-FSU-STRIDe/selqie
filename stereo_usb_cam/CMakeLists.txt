cmake_minimum_required(VERSION 3.8)
project(stereo_usb_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS videoio)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(stereo_usb_cam_component SHARED src/stereo_usb_cam.cpp)
ament_target_dependencies(stereo_usb_cam_component rclcpp rclcpp_components sensor_msgs cv_bridge camera_info_manager image_transport)
target_link_libraries(stereo_usb_cam_component ${OpenCV_LIBS})

rclcpp_components_register_nodes(
  stereo_usb_cam_component 
  PLUGIN "stereo_usb_cam::StereoUsbCam"
  EXECUTABLE stereo_usb_cam_node)

ament_export_targets(export_stereo_usb_cam_component)
install(TARGETS stereo_usb_cam_component
  EXPORT export_stereo_usb_cam_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
