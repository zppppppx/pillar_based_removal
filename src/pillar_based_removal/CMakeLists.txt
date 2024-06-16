cmake_minimum_required(VERSION 3.20)
project(pillar_based_removal LANGUAGES CUDA CXX VERSION 0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(CUDA REQUIRED)
find_package(CUDAToolkit REQUIRED)

add_executable(pillar_based_removal_node src/pillar_based_removal_node.cpp src/pillar_based_removal_algo.cu)

find_library(LIBSPCONV NAMES spconv PATHS /usr/local/lib)
if(NOT LIBSPCONV)
  message(FATAL_ERROR "libspconv.so not found")
endif()

target_include_directories(pillar_based_removal_node PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  "/root/spconv_build/cumm/include"
  "/root/spconv_build/spconv/include"
  "/opt/ros/humble/include/pcl_conversions"
)

target_link_libraries(pillar_based_removal_node 
  ${LIBSPCONV}
  ${PCL_LIBRARIES}
  CUDA::cublas
  CUDA::cudart 
  spconv
)

ament_target_dependencies(pillar_based_removal_node 
  rclcpp 
  sensor_msgs
  PCL
  pcl_conversions
  CUDA
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



# cmake_minimum_required(VERSION 3.20)
# project(pillar_based_removal)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# include_directories(
#   include
#   ${PCL_INCLUDE_DIRS}
#   "/root/spconv_build/cumm/include"
#   "/root/spconv_build/spconv/include"
#   "/opt/ros/humble/include/pcl_conversions"
# )

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_conversions REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# add_executable(pillar_based_removal_node 
#   src/pillar_based_removal_node.cpp 
#   src/pillar_based_removal_algo.cpp
#   /root/spconv_build/spconv/src/spconvlib/spconv/csrc/sparse/all/ops_cpu3d/Point2VoxelCPU/Point2VoxelCPU_point_to_voxel.cc
#   /root/spconv_build/spconv/src/spconvlib/spconv/csrc/sparse/all/ops_cpu3d/Point2VoxelCPU/Point2VoxelCPU_Point2VoxelCPU.cc
#   /root/spconv_build/spconv/src/spconvlib/spconv/csrc/sparse/all/ops_cpu3d/Point2VoxelCPU/Point2VoxelCPU_point_to_voxel_static.cc
# )
# target_include_directories(pillar_based_removal_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
#   ${PCL_INCLUDE_DIRS}
#   "/root/spconv_build/cumm/include"
#   "/root/spconv_build/spconv/include"
#   "/opt/ros/humble/include/pcl_conversions"
# )
# target_compile_features(pillar_based_removal_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(pillar_based_removal_node 
#   rclcpp 
#   sensor_msgs
#   PCL
#   pcl_conversions
# )

# install(TARGETS pillar_based_removal_node
#   DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})

# install(DIRECTORY params
#   DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()




