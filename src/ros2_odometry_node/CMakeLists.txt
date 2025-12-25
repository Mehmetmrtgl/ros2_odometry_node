cmake_minimum_required(VERSION 3.8)
project(ros2_odometry_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include header)

add_executable(ros2_odometry_node
  src/main.cpp
  src/odometry_node.cpp
  
  src/utils/odometry_parameters.cpp
  
  src/mecanum/mecanum_kinematics.cpp
  src/mecanum/mecanum_impl.cpp
  
  src/ackermann/ackermann_kinematics.cpp
  src/ackermann/ackermann_impl.cpp
)

target_include_directories(ros2_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(ros2_odometry_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS ros2_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY header include 
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()