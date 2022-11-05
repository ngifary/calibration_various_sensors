cmake_minimum_required(VERSION 3.8)
project(calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(rosidl_default_generators REQUIRED)

#####################
##  Messages etc.  ##
#####################

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CircleCentroids.msg
  msg/SensorPair.msg
  DEPENDENCIES std_msgs sensor_msgs
)

#############
##  Build  ##
#############

set(node_dependencies
  rclcpp
  pcl_ros
  sensor_msgs
  std_msgs
  stereo_msgs
  image_geometry
  image_transport
  cv_bridge
  OpenCV
  message_filters
  tf2_geometry_msgs
  tf2
  tf2_eigen
  Eigen3
  rmw_implementation
)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

add_executable(calibration_core src/calibration_core.cpp)
ament_target_dependencies(calibration_core ${node_dependencies} std_srvs TinyXML2)

add_executable(lidar_pattern src/lidar_pattern.cpp)
ament_target_dependencies(lidar_pattern ${node_dependencies})

add_executable(mono_qr_pattern src/mono_qr_pattern.cpp)
ament_target_dependencies(mono_qr_pattern ${node_dependencies})

add_executable(disp_masker src/disp_masker.cpp)
ament_target_dependencies(disp_masker ${node_dependencies})

add_executable(stereo_pattern src/stereo_pattern.cpp)
ament_target_dependencies(stereo_pattern ${node_dependencies})

add_executable(plane_segmentation src/plane.cpp)
ament_target_dependencies(plane_segmentation pcl_ros sensor_msgs)

add_executable(overlay_image src/overlay_image.cpp)
ament_target_dependencies(overlay_image ${node_dependencies})

add_executable(my_test src/test.cpp)
ament_target_dependencies(my_test ${node_dependencies})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(stereo_pattern "${cpp_typesupport_target}")
target_link_libraries(mono_qr_pattern "${cpp_typesupport_target}")
target_link_libraries(lidar_pattern "${cpp_typesupport_target}")
target_link_libraries(calibration_core "${cpp_typesupport_target}")

install(TARGETS
  calibration_core
  lidar_pattern
  mono_qr_pattern
  disp_masker
  stereo_pattern
  plane_segmentation
  overlay_image
  my_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME})


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

ament_package()
