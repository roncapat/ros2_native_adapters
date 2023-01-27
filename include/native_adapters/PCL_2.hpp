#ifndef TRAVERSABILITY_TOOLKIT__ADAPTERS__PCL_2
#define TRAVERSABILITY_TOOLKIT__ADAPTERS__PCL_2

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

//#include <variant>

struct StampedPointCloud2 {
  std_msgs::msg::Header header;
  std::variant<
    pcl::PointCloud<pcl::PointXYZ>,
    pcl::PointCloud<pcl::PointXYZL>,
    pcl::PointCloud<pcl::PointXYZI>,
    pcl::PointCloud<pcl::PointXYZRGBA>,
    pcl::PointCloud<pcl::PointXYZRGB>,
    pcl::PointCloud<pcl::PointXYZRGBL>,
    pcl::PointCloud<pcl::PointNormal>,
    pcl::PointCloud<pcl::PointXYZRGBNormal>,
    pcl::PointCloud<pcl::PointXYZLNormal>,
    pcl::PointCloud<pcl::PointXYZINormal>
  > cloud;

  StampedPointCloud2() = default;

  StampedPointCloud2(const StampedPointCloud2 &other){
    std::cerr << "[PointCloud2 Adapter] Copy constructor called" << std::endl;
    //raise(SIGTRAP);
    this->header = other.header;
    this->cloud = other.cloud;
  }

  StampedPointCloud2(StampedPointCloud2 &&other){
    std::cerr << "[PointCloud2 Adapter] Move constructor called" << std::endl;
    //raise(SIGTRAP);
    this->header = std::move(other.header);
    this->cloud = std::move(other.cloud);
  }

  StampedPointCloud2 & operator=(const StampedPointCloud2 & other){
    std::cerr << "[PointCloud2 Adapter] Assignment operator called" << std::endl;
    //raise(SIGTRAP);
    if (this == &other) return *this; 
    this->header = other.header;
    this->cloud = other.cloud;
  }

  uint32_t width() const {
    return std::visit([](auto&& cloud){return cloud.width;}, cloud);
  }

  uint32_t height() const {
    return std::visit([](auto&& cloud){return cloud.height;}, cloud);
  }

  bool has_colors() const {
    return std::visit([](auto&& cloud) {
      using T = typename std::decay_t<decltype(cloud)>::PointType;
      return pcl::traits::has_color_v<T>;
    }, cloud);
  }
};

template<>
struct rclcpp::TypeAdapter<StampedPointCloud2, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = StampedPointCloud2;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message (const custom_type & source, ros_message_type & destination){
    std::cerr << "[PointCloud2 Adapter] Conversion to message" << std::endl;
    //raise(SIGTRAP);

    std::visit([&](auto&& cloud){pcl::toROSMsg(cloud, destination);}, source.cloud);
    destination.header = source.header;
  }

  static void convert_to_custom (const ros_message_type & source, custom_type & destination){
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

  template <typename PointT>
  static void process_message(const ros_message_type & source, custom_type & destination){
    destination.cloud = pcl::PointCloud<PointT>();
    pcl::fromROSMsg(source, std::get<pcl::PointCloud<PointT>>(destination.cloud));
  }
};

#endif // TRAVERSABILITY_TOOLKIT__ADAPTERS__PCL_2