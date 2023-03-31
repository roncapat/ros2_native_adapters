#include <native_adapters/SensorMsgPointCloud2_PCLPointCloud.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

StampedPointCloud_PCL::StampedPointCloud_PCL(const StampedPointCloud_PCL &other){
  std::cerr << "[PointCloud2 Adapter] Copy constructor called" << std::endl;
  //raise(SIGTRAP);
  this->header = other.header;
  this->cloud = other.cloud;
}

StampedPointCloud_PCL::StampedPointCloud_PCL(StampedPointCloud_PCL &&other){
  std::cerr << "[PointCloud2 Adapter] Move constructor called" << std::endl;
  //raise(SIGTRAP);
  this->header = std::move(other.header);
  this->cloud = std::move(other.cloud);
}

StampedPointCloud_PCL & StampedPointCloud_PCL::operator=(const StampedPointCloud_PCL & other){
  std::cerr << "[PointCloud2 Adapter] Assignment operator called" << std::endl;
  //raise(SIGTRAP);
  if (this == &other) return *this; 
  this->header = other.header;
  this->cloud = other.cloud;
  return *this;
}

uint32_t StampedPointCloud_PCL::width() const {
  return std::visit([](auto&& cloud){return cloud.width;}, cloud);
}

uint32_t StampedPointCloud_PCL::height() const {
  return std::visit([](auto&& cloud){return cloud.height;}, cloud);
}

bool StampedPointCloud_PCL::has_colors() const {
  return std::visit([](auto&& cloud) {
    using T = typename std::decay_t<decltype(cloud)>::PointType;
    return pcl::traits::has_color_v<T>;
  }, cloud);
}

template <typename PointT>
void process_message(const sensor_msgs::msg::PointCloud2 & source, StampedPointCloud_PCL & destination){
  destination.cloud = std::move(pcl::PointCloud<PointT>());
  pcl::fromROSMsg(source, std::get<pcl::PointCloud<PointT>>(destination.cloud));
}

void rclcpp::TypeAdapter<StampedPointCloud_PCL, sensor_msgs::msg::PointCloud2>::convert_to_ros_message (const StampedPointCloud_PCL & source, sensor_msgs::msg::PointCloud2 & destination){
  std::cerr << "[PointCloud2 Adapter] Conversion to message" << std::endl;
  //raise(SIGTRAP);

  std::visit([&](auto&& cloud){pcl::toROSMsg(cloud, destination);}, source.cloud);
  destination.header = source.header;
}

void rclcpp::TypeAdapter<StampedPointCloud_PCL, sensor_msgs::msg::PointCloud2>::convert_to_custom (const sensor_msgs::msg::PointCloud2 & source, StampedPointCloud_PCL & destination){
  std::cerr << "[PointCloud2 Adapter] Conversion from message" << std::endl;
  //raise(SIGTRAP);
  
  bool rgb = false;
  bool rgba = false;
  bool intensity = false;
  bool normals = false;
  bool label = false;
  for (auto f : source.fields) {
    if (f.name == "rgb") {
      rgb = true;
    }
    if (f.name == "rgba") {
      rgba = true;
    }
    if (f.name == "intensity") {
      intensity = true;
    }
    if (f.name == "normal_x") {
      normals = true;
    }
    if (f.name == "label") {
      label = true;
    }
  }

  if (rgba) {
    process_message<pcl::PointXYZRGBA>(source, destination);
  } else if (rgb) {
    if (normals) {
      process_message<pcl::PointXYZRGBNormal>(source, destination);
    } else if (label) {
      process_message<pcl::PointXYZRGBL>(source, destination);
    } else {
      process_message<pcl::PointXYZRGB>(source, destination);
    }
  } else if (intensity) {
    if (normals) {
      process_message<pcl::PointXYZINormal>(source, destination);
    } else {
      process_message<pcl::PointXYZI>(source, destination);
    }
  } else if (label) {
    if (normals) {
      process_message<pcl::PointNormal>(source, destination);
    } else {
      process_message<pcl::PointXYZ>(source, destination);
    }
  } else {
    if (normals) {
      process_message<pcl::PointXYZLNormal>(source, destination);
    } else {
      process_message<pcl::PointXYZL>(source, destination);
    }
  }

  destination.header = source.header;
}

