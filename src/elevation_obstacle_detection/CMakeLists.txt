cmake_minimum_required(VERSION 3.5)
project(elevation_obstacle_detection)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(box_msg REQUIRED)
find_package(dth_messages REQUIRED)
find_package(perception_common REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Build the node
add_executable(elevation_obstacle_detection_node
  src/elevation_obstacle_detection_node.cpp
)

ament_target_dependencies(elevation_obstacle_detection_node
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  pcl_conversions
  box_msg
  dth_messages
  perception_common
)

target_link_libraries(elevation_obstacle_detection_node
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS elevation_obstacle_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
