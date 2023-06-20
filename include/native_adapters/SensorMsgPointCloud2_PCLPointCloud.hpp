/*
Copyright 2023 Patrick Roncagliolo
*/

#ifndef NATIVE_ADAPTERS__SENSORMSGPOINTCLOUD2_PCLPOINTCLOUD_HPP_
#define NATIVE_ADAPTERS__SENSORMSGPOINTCLOUD2_PCLPOINTCLOUD_HPP_

#include <pcl/common/common.h>

#include <variant>

#include <rclcpp/type_adapter.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <native_adapters/PointerDefinesMacro.hpp>

struct StampedPointCloud_PCL
{
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

  StampedPointCloud_PCL() = default;
  StampedPointCloud_PCL(const StampedPointCloud_PCL & other);
  StampedPointCloud_PCL(StampedPointCloud_PCL && other);
  StampedPointCloud_PCL & operator=(const StampedPointCloud_PCL & other);

  DEFINE_MSG_POINTERS(StampedPointCloud_PCL)

  uint32_t width() const;
  uint32_t height() const;
  bool has_colors() const;
};

template<>
struct rclcpp::TypeAdapter<StampedPointCloud_PCL, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = StampedPointCloud_PCL;
  using ros_message_type = sensor_msgs::msg::PointCloud2;
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

#endif  // NATIVE_ADAPTERS__SENSORMSGPOINTCLOUD2_PCLPOINTCLOUD_HPP_
