cmake_minimum_required(VERSION 3.8)
project(dx_calibrate_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(charuco_detector
  src/main.cpp
  src/charuco_detector_node.cpp
)

target_include_directories(charuco_detector PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)

ament_target_dependencies(charuco_detector
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)

target_link_libraries(charuco_detector
  ${OpenCV_LIBS}
)

install(TARGETS charuco_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
