cmake_minimum_required(VERSION 3.5)
project(neobotix_coordinator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(iras_behaviortree_ros2 REQUIRED)
find_package(iras_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry2 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
#find_package(object_detector_tensorflow_interfaces REQUIRED)

include_directories(
  include 
  ${behaviortree_ros_INCLUDE_DIRS})

 
set(DEPENDENCIES 
  rclcpp
  rclcpp_action
  iras_behaviortree_ros2
  iras_interfaces
  tf2_ros
  tf2_geometry_msgs
  geometry2
  ament_index_cpp
  nav2_msgs
  std_srvs
  #object_detector_tensorflow_interfaces
)

file (GLOB SOURCES_ACTIONS src/actions/*.cpp)
file (GLOB SOURCES_SERVICES src/services/*.cpp)
file (GLOB SOURCES_CONDITIONS src/conditions/*.cpp)
file (GLOB SOURCES_CONTROLS src/controls/*.cpp)
file (GLOB SOURCES_NODES src/nodes/*.cpp)

add_executable(Coordinator
  src/node.cpp
  ${SOURCES_ACTIONS}
  ${SOURCES_SERVICES}
  ${SOURCES_CONDITIONS}
  ${SOURCES_CONTROLS}
  ${SOURCES_NODES})

target_link_libraries(Coordinator
  iras_behaviortree_ros2::behaviortree_ros_lib
)

ament_target_dependencies(Coordinator
  ${DEPENDENCIES})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  behaviors
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  Coordinator
  DESTINATION lib/${PROJECT_NAME})

ament_package()