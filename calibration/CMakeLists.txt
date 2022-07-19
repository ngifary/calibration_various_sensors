cmake_minimum_required(VERSION 3.8)
project(calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(tinyxml_vendor REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(calibration_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

include_directories(cfg)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_executable(velo2cam_calibration src/velo2cam_calibration.cpp)
ament_target_dependencies(velo2cam_calibration tinyxml_vendor tinyxml2_vendor rclcpp cv_bridge sensor_msgs std_msgs OpenCV pcl_ros image_geometry image_transport message_filters stereo_msgs tf2_geometry_msgs tf2_ros tf2 rcl_interfaces calibration_interfaces)

add_executable(lidar_pattern src/lidar_pattern.cpp)
ament_target_dependencies(lidar_pattern rclcpp sensor_msgs std_msgs pcl_ros tf2_geometry_msgs tf2 calibration_interfaces)

add_executable(mono_qr_pattern src/mono_qr_pattern.cpp)
ament_target_dependencies(mono_qr_pattern rclcpp cv_bridge image_geometry message_filters pcl_ros sensor_msgs std_msgs OpenCV calibration_interfaces)

add_executable(disp_masker src/disp_masker.cpp)
ament_target_dependencies(disp_masker rclcpp cv_bridge message_filters sensor_msgs stereo_msgs OpenCV)

add_executable(stereo_pattern src/stereo_pattern.cpp)
ament_target_dependencies(stereo_pattern rclcpp message_filters pcl_ros sensor_msgs std_msgs calibration_interfaces)

add_executable(plane src/plane.cpp)
ament_target_dependencies(plane rclcpp pcl_ros sensor_msgs)

install(TARGETS
  velo2cam_calibration
  lidar_pattern
  mono_qr_pattern
  disp_masker
  stereo_pattern
  plane
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
