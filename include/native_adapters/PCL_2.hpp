#ifndef TRAVERSABILITY_TOOLKIT__ADAPTERS__PCL_2
#define TRAVERSABILITY_TOOLKIT__ADAPTERS__PCL_2

#include <rclcpp/type_adapter.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/common.h>

#include <variant>

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
  StampedPointCloud2(const StampedPointCloud2 &other);
  StampedPointCloud2(StampedPointCloud2 &&other);
  StampedPointCloud2 & operator=(const StampedPointCloud2 & other);

  uint32_t width() const;
  uint32_t height() const;
  bool has_colors() const;
};

template<>
struct rclcpp::TypeAdapter<StampedPointCloud2, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = StampedPointCloud2;
  using ros_message_type = sensor_msgs::msg::PointCloud2;
  static void convert_to_ros_message (const custom_type & source, ros_message_type & destination);  
  static void convert_to_custom (const ros_message_type & source, custom_type & destination);
};

#endif // TRAVERSABILITY_TOOLKIT__ADAPTERS__PCL_2