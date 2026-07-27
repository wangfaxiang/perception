cmake_minimum_required(VERSION 3.8)
project(centerpoint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies


# find_package(std_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV 4 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

# if(${arch}     STREQUAL  "aarch64")
#   # set(TENSORRT_ROOT         /root/.kiwi/lib/TensorRT-8.5.3.1-cuda11x)
#   # set(CUDA_TOOLKIT_ROOT_DIR /root/.kiwi/lib/cuda-11.8)
# elseif(${arch} STREQUAL "x86_64")
#   set(TENSORRT_ROOT         /root/.kiwi/lib/TensorRT-8.5.3.1-cuda11x)
#   set(CUDA_TOOLKIT_ROOT_DIR /root/.kiwi/lib/cuda-11.8)
# endif()
set(TENSORRT_ROOT         /home/promote/wfx/tools/TensorRT/TensorRT-8.6.0.12)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-11.8)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -Wextra -Wall -Wno-deprecated-declarations -O3")
# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -gencode arch=compute_87,code=compute_87)
set(TENSORRT_INCLUDE ${TENSORRT_ROOT}/include)
set(TENSORRT_LIB     ${TENSORRT_ROOT}/lib)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(box_msg REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(CUDA REQUIRED)
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  message("Using Debug Mode")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
endif()

include_directories(
  ${pcl_conversions_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE}
  ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libspconv/include
  include
)

link_directories(
  ${TENSORRT_LIB}
  /usr/lib/
)

file(GLOB_RECURSE SOURCE_FILES 
  src/*.cu 
  src/*.cpp
)

cuda_add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${TENSORRT_ROOT}/lib/libnvinfer.so
    /usr/lib/libspconv.so
)



ament_target_dependencies(centerpoint rclcpp std_msgs sensor_msgs box_msg) 

install(TARGETS
  centerpoint 
  DESTINATION lib/${PROJECT_NAME})

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

