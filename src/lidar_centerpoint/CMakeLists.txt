cmake_minimum_required(VERSION 3.14)
project(lidar_centerpoint)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# TODO(lidar_centerpoint): Remove once upgrading to TensorRT 8.5 is complete
add_compile_options(-Wno-deprecated-declarations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(box_msg REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# CUDA
set(CUDA_TOOLKIT_ROOT_DIR "" CACHE PATH "CUDA toolkit root (e.g. /usr/local/cuda)")
find_package(CUDA REQUIRED)

# TensorRT / cuDNN
set(TENSORRT_ROOT "" CACHE PATH "TensorRT install root (contains include/ and lib/)")
find_path(TENSORRT_INCLUDE NvInfer.h
  HINTS ${TENSORRT_ROOT}/include $ENV{TENSORRT_ROOT}/include /usr/include/aarch64-linux-gnu)
find_library(NVINFER nvinfer
  HINTS ${TENSORRT_ROOT}/lib $ENV{TENSORRT_ROOT}/lib /usr/lib/aarch64-linux-gnu)
find_library(NVONNXPARSER nvonnxparser
  HINTS ${TENSORRT_ROOT}/lib $ENV{TENSORRT_ROOT}/lib /usr/lib/aarch64-linux-gnu)
find_library(CUDNN_LIBRARY cudnn
  HINTS ${CUDA_TOOLKIT_ROOT_DIR}/lib64 /usr/lib/aarch64-linux-gnu)
if(NOT NVINFER OR NOT NVONNXPARSER)
  message(FATAL_ERROR "TensorRT (libnvinfer / libnvonnxparser) not found. "
                      "Pass -DTENSORRT_ROOT=/path/to/TensorRT.")
endif()

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -gencode arch=compute_61,code=sm_61
                                        -gencode arch=compute_75,code=sm_75
                                        -gencode arch=compute_86,code=sm_86
                                        -gencode arch=compute_87,code=sm_87
                                        -gencode arch=compute_89,code=sm_89
                                        -gencode arch=compute_90,code=sm_90")

include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

### centerpoint CUDA kernels ###
cuda_add_library(centerpoint_cuda_lib SHARED
  lib/postprocess/circle_nms_kernel.cu
  lib/postprocess/postprocess_kernel.cu
  lib/network/scatter_kernel.cu
  lib/preprocess/preprocess_kernel.cu
)
target_link_libraries(centerpoint_cuda_lib ${CUDA_LIBRARIES})

### centerpoint core library ###
add_library(centerpoint_lib SHARED
  lib/centerpoint_trt.cpp
  lib/network/network_trt.cpp
  lib/network/tensorrt_wrapper.cpp
  lib/preprocess/pointcloud_densification.cpp
  lib/preprocess/voxel_generator.cpp
  lib/utils.cpp
  lib/ros_utils.cpp
)
target_include_directories(centerpoint_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Suppress unknown-pragmas from CUB in CUDA headers
target_include_directories(centerpoint_lib SYSTEM PUBLIC ${CUDA_INCLUDE_DIRS})
target_link_libraries(centerpoint_lib
  centerpoint_cuda_lib
  ${NVINFER}
  ${NVONNXPARSER}
  ${CUDNN_LIBRARY}
  ${CUDA_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(centerpoint_lib
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  box_msg
  tf2
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  pcl_conversions
)

### node executable ###
add_executable(lidar_centerpoint_node src/node.cpp)
target_link_libraries(lidar_centerpoint_node centerpoint_lib)
ament_target_dependencies(lidar_centerpoint_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  box_msg
  tf2
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  pcl_conversions
)

install(TARGETS
  lidar_centerpoint_node
  centerpoint_lib
  centerpoint_cuda_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
