cmake_minimum_required(VERSION 3.22)
project(native_adapters)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wdouble-promotion -Wfloat-equal -Wshadow)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(PCL 1.12 REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(dem_msgs REQUIRED)

set(PUB_HEADERS
  include/native_adapters/NavMsgOccupancyGrid_CvMat.hpp
  include/native_adapters/SensorMsgImage_CvMat.hpp
  include/native_adapters/CV.hpp
  include/native_adapters/SensorMsgPointCloud2_PCLPointCloud.hpp
  include/native_adapters/PCL.hpp
  include/native_adapters/PointerDefinesMacro.hpp
  include/native_adapters/DigitalElevationMapMsg_CvMat.hpp
)

add_library(pcl_adapters SHARED
  src/SensorMsgPointCloud2_PCLPointCloud.cpp
)
set_target_properties(pcl_adapters PROPERTIES PUBLIC_HEADER "${PUB_HEADERS}")
target_include_directories(
  pcl_adapters
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  PRIVATE
  ${pcl_conversions_INCLUDE_DIRS}
)

target_link_libraries(pcl_adapters
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  pcl_common
)

add_library(cv_adapters SHARED
  src/NavMsgOccupancyGrid_CvMat.cpp
  src/SensorMsgImage_CvMat.cpp
  src/DigitalElevationMapMsg_CvMat.cpp
)
set_target_properties(cv_adapters PROPERTIES PUBLIC_HEADER "${PUB_HEADERS}")
target_include_directories(
  cv_adapters
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  PRIVATE
  ${cv_bridge_INCLUDE_DIRS}
)

target_link_libraries(cv_adapters
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${dem_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  opencv_core
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS pcl_adapters
  EXPORT export_pcl_adapters
  PUBLIC_HEADER DESTINATION include/native_adapters
  INCLUDES DESTINATION include
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS cv_adapters
  EXPORT export_cv_adapters
  PUBLIC_HEADER DESTINATION include/native_adapters
  INCLUDES DESTINATION include
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_pcl_adapters export_cv_adapters HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  nav_msgs
  pcl_msgs
  dem_msgs
  cv_bridge
  OpenCV
  PCL
  pcl_conversions
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
