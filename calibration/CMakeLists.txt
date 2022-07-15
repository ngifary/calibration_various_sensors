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
find_package(rclcpp_components REQUIRED)
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

add_library(passthrough_component SHARED src/passthrough_component.cpp)
target_compile_definitions(passthrough_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(passthrough_component rclcpp rclcpp_components std_msgs pcl_ros)
rclcpp_components_register_nodes(passthrough_component "composition::PassThrough")
set(node_plugins "${node_plugins}composition::PassThrough;$<TARGET_FILE:passthrough_component>\n")

add_library(extract_indices_component SHARED src/extract_indices_component.cpp)
target_compile_definitions(extract_indices_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(extract_indices_component rclcpp rclcpp_components std_msgs pcl_ros)
rclcpp_components_register_nodes(extract_indices_component "composition::ExtractIndices")
set(node_plugins "${node_plugins}composition::ExtractIndices;$<TARGET_FILE:extract_indices_component>\n")

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

add_executable(manual_composition src/manual_composition.cpp)
target_link_libraries(manual_composition passthrough_component extract_indices_component)
ament_target_dependencies(manual_composition rclcpp)

add_executable(linktime_composition src/linktime_composition.cpp)
set(libs passthrough_component extract_indices_component)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(libs
    "-Wl,--no-as-needed"
    ${libs}
    "-Wl,--as-needed")
endif()
target_link_libraries(linktime_composition ${libs})
ament_target_dependencies(linktime_composition class_loader rclcpp rclcpp_components)

add_executable(dlopen_composition src/dlopen_composition.cpp)
ament_target_dependencies(dlopen_composition class_loader rclcpp rclcpp_components)

install(TARGETS
  passthrough_component
  extract_indices_component
  velo2cam_calibration
  lidar_pattern
  mono_qr_pattern
  disp_masker
  stereo_pattern
  plane
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  manual_composition
  linktime_composition
  dlopen_composition
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
