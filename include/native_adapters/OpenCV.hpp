#ifndef TRAVERSABILITY_TOOLKIT__ADAPTERS__OCV
#define TRAVERSABILITY_TOOLKIT__ADAPTERS__OCV

#include <rclcpp/type_adapter.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <opencv2/core/mat.hpp>

struct StampedCvMat {
  std_msgs::msg::Header header;
  nav_msgs::msg::MapMetaData info;
  cv::Mat mat;

  static const uint8_t UNKNOWN = 0;
  static const uint8_t FREE = 1;
  static const uint8_t MIN_COST = 2;
  static const uint8_t OBSTACLE = 101;

  StampedCvMat() = default;
  StampedCvMat(const StampedCvMat &other);
  StampedCvMat(StampedCvMat &&other);
  StampedCvMat & operator=(const StampedCvMat & other);
};

template<>
struct rclcpp::TypeAdapter<StampedCvMat, nav_msgs::msg::OccupancyGrid>{
  using is_specialized = std::true_type;
  using custom_type = StampedCvMat;
  using ros_message_type = nav_msgs::msg::OccupancyGrid;
  static void convert_to_ros_message (const custom_type & source, ros_message_type & destination);
  static void convert_to_custom (const ros_message_type & source, custom_type & destination);
};

#endif // TRAVERSABILITY_TOOLKIT__ADAPTERS__OCV