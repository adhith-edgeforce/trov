cmake_minimum_required(VERSION 3.8)
project(trov)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED) 
find_package(robot_state_publisher REQUIRED)
#find_package(joint_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)


# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

# ============================================
# Obstacle Detector Node
# ============================================
add_executable(obstacle_detector 
  src/obstacle_detector.cpp
)

add_executable(collision_beacon 
  src/collision_beacon.cpp
)

add_executable(hrtk_odom 
  src/hrtk_mosaic_odom.cpp
)

add_executable(rslidar_to_lio 
  src/rslidar_to_lio.cpp
)

add_executable(sensors_health_status 
  src/sensor_health_status.cpp
)

add_executable(failsafe 
  src/failsafe.cpp
)

ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

ament_target_dependencies(collision_beacon
  rclcpp
  sensor_msgs
  pcl_conversions
  visualization_msgs
)

ament_target_dependencies(sensors_health_status
  rclcpp
  sensor_msgs
  std_msgs
)

target_include_directories(collision_beacon PUBLIC
  /opt/ros/humble/include/sensor_msgs
)

target_link_libraries(collision_beacon gpiod)

ament_target_dependencies(hrtk_odom
  rclcpp
  nav2_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

ament_target_dependencies(failsafe
  rclcpp
  geometry_msgs
  nav_msgs
  action_msgs
  std_msgs
)


target_link_libraries(obstacle_detector
  ${PCL_LIBRARIES}
)

target_link_libraries(hrtk_odom
  ${PCL_LIBRARIES}
)


# ── executable ───────────────────────────────────────────────────────────────
add_executable(pointcloud_downsampler_node
  src/pointcloud_downsampler.cpp)
 
target_include_directories(pointcloud_downsampler_node PUBLIC
  ${PCL_INCLUDE_DIRS})
 
target_link_libraries(pointcloud_downsampler_node
  pcl_common
  pcl_filters)
 
ament_target_dependencies(pointcloud_downsampler_node
  rclcpp
  sensor_msgs
  pcl_conversions)
 
# ── install ──────────────────────────────────────────────────────────────────
install(TARGETS pointcloud_downsampler_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(trov_drive_bridge src/trov_drive_bridge.cpp)
ament_target_dependencies(trov_drive_bridge rclcpp geometry_msgs mavros_msgs std_msgs)
install(TARGETS trov_drive_bridge DESTINATION lib/${PROJECT_NAME})


add_executable(bumper_filter_node src/bumper_filter_node.cpp)
ament_target_dependencies(bumper_filter_node
  rclcpp sensor_msgs tf2_ros tf2_sensor_msgs pcl_conversions)
target_link_libraries(bumper_filter_node ${PCL_LIBRARIES})

install(TARGETS bumper_filter_node DESTINATION lib/${PROJECT_NAME})


# ============ BUILD EXECUTABLE ============
add_executable(pointcloud_crop_filter src/pointcloud_crop_filter.cpp)

ament_target_dependencies(pointcloud_crop_filter
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

ament_target_dependencies(rslidar_to_lio
  rclcpp
  sensor_msgs
)

target_link_libraries(pointcloud_crop_filter
  ${PCL_LIBRARIES}
)

install(TARGETS rslidar_to_lio
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS sensors_health_status
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS failsafe
  DESTINATION lib/${PROJECT_NAME}
)

# ============ INSTALL EXECUTABLE ============
install(TARGETS
  pointcloud_crop_filter
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(headlight_controller src/headlight_controller.cpp)
ament_target_dependencies(headlight_controller rclcpp std_msgs mavros_msgs)
install(TARGETS headlight_controller DESTINATION lib/${PROJECT_NAME})


# ============================================
# Install Python Scripts
# ============================================

install(PROGRAMS
  scripts/waypoint_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_follower_poses.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gps_frame_changer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/floodlight.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/delivery_runner.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(battery_monitor src/battery_monitor.cpp)
ament_target_dependencies(battery_monitor rclcpp std_msgs)
target_include_directories(battery_monitor PRIVATE /usr/include)
install(TARGETS battery_monitor DESTINATION lib/${PROJECT_NAME})

add_executable(beeper src/beeper.cpp)
ament_target_dependencies(beeper rclcpp mavros_msgs)
install(TARGETS beeper DESTINATION lib/${PROJECT_NAME})


# ============================================
# Install targets
# ============================================
install(TARGETS
  obstacle_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  collision_beacon
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  hrtk_odom
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install mesh
install(DIRECTORY
  mesh
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF/xacro files
install(DIRECTORY
  description
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

# Install graphs directory
install(DIRECTORY
  graphs
  DESTINATION share/${PROJECT_NAME}/
)

# Install routes directory
install(DIRECTORY
  routes
  DESTINATION share/${PROJECT_NAME}/
)

# ============================================
# Testing
# ============================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
