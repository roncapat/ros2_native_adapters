cmake_minimum_required(VERSION 3.5)
project(native_adapters)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

install(
  DIRECTORY include/native_adapters
  DESTINATION include
)

ament_export_include_directories(include ${PCL_INCLUDE_DIRS})
ament_export_dependencies(std_msgs sensor_msgs nav_msgs OpenCV PCL pcl_conversions)

ament_package()
