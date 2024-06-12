cmake_minimum_required(VERSION 3.20)
project(pillar_based_removal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  "/root/spconv_build/cumm/include"
  "/root/spconv_build/spconv/include"
  "/opt/ros/humble/include/pcl_conversions"
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pillar_based_removal_node src/pillar_based_removal_node.cpp)
target_include_directories(pillar_based_removal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
  "/root/spconv_build/cumm/include"
  "/root/spconv_build/spconv/include"
  "/opt/ros/humble/include/pcl_conversions"
)
target_compile_features(pillar_based_removal_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pillar_based_removal_node 
  rclcpp 
  sensor_msgs
  PCL
  pcl_conversions
)

install(TARGETS pillar_based_removal_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

















# cmake_minimum_required(VERSION 3.18)
# project(pillar_based_removal)

# # Compiler options
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Set C++ Standard
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Find packages
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(PCL REQUIRED)
# find_package(pcl_conversions REQUIRED)

# # Include directories
# include_directories(
#   ${PCL_INCLUDE_DIRS}
#   "/opt/ros/humble/include/pcl_conversions"
# )

# set(SPCONV_INCLUDE_DIR "/root/spconv_build/spconv/include")
# set(CUMM_INCLUDE_DIR "/root/spconv_build/cumm/include")

# # Find external libraries, assuming they do not have CMake support
# find_library(SPCONV_LIBRARY NAMES spconv HINTS /usr/local/lib)
# # find_path(SPCONV_INCLUDE_DIR NAMES spconv.h HINTS /root/spconv_build/spconv/include)

# # find_path(CUMM_INCLUDE_DIR NAMES tensor.h HINTS /root/spconv_build/cumm/include)

# # Add executable
# add_executable(pillar_based_removal_node src/pillar_based_removal_node.cpp)

# # Include headers and link libraries
# target_include_directories(pillar_based_removal_node PRIVATE
#   include
#   ${SPCONV_INCLUDE_DIR}
#   ${CUMM_INCLUDE_DIR}
# )

# target_link_libraries(pillar_based_removal_node
#   ${SPCONV_LIBRARY}
#   ${PCL_LIBRARIES}
# )

# ament_target_dependencies(pillar_based_removal_node 
#   rclcpp 
#   sensor_msgs
#   PCL
#   pcl_conversions
# )

# # Install targets
# install(TARGETS pillar_based_removal_node
#   DESTINATION lib/${PROJECT_NAME})

# # Linting
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
