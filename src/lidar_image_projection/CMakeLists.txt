cmake_minimum_required(VERSION 3.5)
project(lidar_image_projection)

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
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

# Build the node
add_executable(lidar_image_projection_node
  src/lidar_image_projection_node.cpp
)

ament_target_dependencies(lidar_image_projection_node
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
)

target_link_libraries(lidar_image_projection_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS lidar_image_projection_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
