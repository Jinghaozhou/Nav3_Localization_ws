cmake_minimum_required(VERSION 3.16)
project(localization_path_recorder)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(localization_path_recorder_node src/localization_path_recorder_node.cpp)
ament_target_dependencies(localization_path_recorder_node
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

install(TARGETS localization_path_recorder_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
