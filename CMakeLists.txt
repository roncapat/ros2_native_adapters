cmake_minimum_required(VERSION 3.5)
project(native_adapters)

set(CMAKE_CXX_STANDARD 17)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
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

add_library(NativeAdapters SHARED
  src/NavMsgOccupancyGrid_CvMat.cpp
  src/SensorMsgPointCloud2_PCLPointCloud.cpp
  src/SensorMsgImage_CvMat.cpp
  src/DigitalElevationMapMsg_CvMat.cpp)
set_target_properties(NativeAdapters PROPERTIES PUBLIC_HEADER "${PUB_HEADERS}")
target_include_directories(NativeAdapters PUBLIC include)

target_include_directories(NativeAdapters PRIVATE
  ${pcl_conversions_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

target_link_libraries(NativeAdapters
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${dem_msgs_TARGETS}
  ${cv_bridge_TARGETS}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

install(
  TARGETS NativeAdapters
  PUBLIC_HEADER DESTINATION include/native_adapters
  INCLUDES DESTINATION include
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#ament_export_targets(NativeAdapters)
ament_export_include_directories(include ${PCL_INCLUDE_DIRS})
ament_export_libraries(NativeAdapters)
ament_export_dependencies(rclcpp rclcpp_components std_msgs sensor_msgs nav_msgs OpenCV PCL pcl_conversions pcl_msgs dem_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
