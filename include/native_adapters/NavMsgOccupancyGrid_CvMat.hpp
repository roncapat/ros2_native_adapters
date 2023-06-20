/*
Copyright 2023 Patrick Roncagliolo
*/

#ifndef NATIVE_ADAPTERS__NAVMSGOCCUPANCYGRID_CVMAT_HPP_
#define NATIVE_ADAPTERS__NAVMSGOCCUPANCYGRID_CVMAT_HPP_
#include <rclcpp/type_adapter.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <opencv2/core/mat.hpp>
#include <native_adapters/PointerDefinesMacro.hpp>

struct StampedOccupancyGrid_CV
{
  std_msgs::msg::Header header;
  nav_msgs::msg::MapMetaData info;
  cv::Mat mat;

  static const uint8_t UNKNOWN;
  static const uint8_t FREE;
  static const uint8_t MIN_COST;
  static const uint8_t OBSTACLE;

  StampedOccupancyGrid_CV() = default;
  StampedOccupancyGrid_CV(const StampedOccupancyGrid_CV & other);
  StampedOccupancyGrid_CV(StampedOccupancyGrid_CV && other);
  StampedOccupancyGrid_CV & operator=(const StampedOccupancyGrid_CV & other);

  DEFINE_MSG_POINTERS(StampedOccupancyGrid_CV)
};

template<>
struct rclcpp::TypeAdapter<StampedOccupancyGrid_CV, nav_msgs::msg::OccupancyGrid>
{
  using is_specialized = std::true_type;
  using custom_type = StampedOccupancyGrid_CV;
  using ros_message_type = nav_msgs::msg::OccupancyGrid;
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

#endif  // NATIVE_ADAPTERS__NAVMSGOCCUPANCYGRID_CVMAT_HPP_
