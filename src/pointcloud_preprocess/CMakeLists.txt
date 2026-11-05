cmake_minimum_required(VERSION 3.5)
project(pointcloud_preprocess)

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
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(${PCL_INCLUDE_DIRS})

# Build the node
add_executable(pointcloud_preprocess_node
  src/pointcloud_preprocess_node.cpp
)

ament_target_dependencies(pointcloud_preprocess_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(pointcloud_preprocess_node
  ${PCL_LIBRARIES}
)

# Install
install(TARGETS pointcloud_preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
