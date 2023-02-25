cmake_minimum_required(VERSION 3.5)
project(native_adapters)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

set(PUB_HEADERS
  include/native_adapters/OpenCV.hpp
  include/native_adapters/PCL_2.hpp
)

add_library(NativeAdapters SHARED src/OpenCV.cpp src/PCL_2.cpp)
set_target_properties(NativeAdapters PROPERTIES PUBLIC_HEADER "${PUB_HEADERS}")
target_include_directories(NativeAdapters PUBLIC include)

target_include_directories(NativeAdapters PRIVATE 
  ${PCL_INCLUDE_DIRS} 
  ${pcl_conversions_INCLUDE_DIRS}
)
target_link_libraries(NativeAdapters
  rclcpp::rclcpp 
  ${rclcpp_components_TARGETS} 
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${OpenCV_LIBS})

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
ament_export_dependencies(rclcpp rclcpp_components std_msgs sensor_msgs nav_msgs OpenCV PCL pcl_conversions)

ament_package()
