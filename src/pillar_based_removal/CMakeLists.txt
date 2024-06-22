cmake_minimum_required(VERSION 3.20)
project(pillar_based_removal LANGUAGES CUDA CXX VERSION 0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(CUDA REQUIRED)
find_package(CUDAToolkit REQUIRED)


find_library(LIBSPCONV NAMES spconv PATHS /usr/local/lib)
if(NOT LIBSPCONV)
  message(FATAL_ERROR "libspconv.so not found")
endif()

add_executable(pillar_based_removal_node 
  src/pillar_based_removal_algo.cu
  src/pillar_based_removal_node.cu
)

target_compile_options(pillar_based_removal_node PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda> -DTV_CUDA)

target_include_directories(pillar_based_removal_node PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  "/root/spconv_cpp/cumm/include"
  "/root/spconv_cpp/spconv/include"
  "/opt/ros/humble/include/pcl_conversions"
)

target_link_libraries(pillar_based_removal_node 
  ${LIBSPCONV}
  ${PCL_LIBRARIES}
  CUDA::cublas
  CUDA::cublasLt
  CUDA::cudart
)

ament_target_dependencies(pillar_based_removal_node 
  rclcpp 
  sensor_msgs
  PCL
  pcl_conversions
)

install(TARGETS pillar_based_removal_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
