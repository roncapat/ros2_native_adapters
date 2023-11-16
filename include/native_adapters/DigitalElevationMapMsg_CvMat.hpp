/*
Copyright 2023 Patrick Roncagliolo, Antonino Bongiovanni
*/

#ifndef NATIVE_ADAPTERS__SENSORMSGIMAGE_CVMAT_HPP_
#define NATIVE_ADAPTERS__SENSORMSGIMAGE_CVMAT_HPP_
#include <rclcpp/type_adapter.hpp>
#include <dem_msgs/msg/digital_elevation_map.hpp>
#include <opencv2/core/mat.hpp>
#include <native_adapters/PointerDefinesMacro.hpp>


struct StampedDigitalElevationMap_CV
{
  std_msgs::msg::Header header;
  nav_msgs::msg::MapMetaData info;
  cv::Mat mat;

  StampedDigitalElevationMap_CV() = default;
  StampedDigitalElevationMap_CV(const StampedDigitalElevationMap_CV & other);
  StampedDigitalElevationMap_CV(StampedDigitalElevationMap_CV && other);
  StampedDigitalElevationMap_CV & operator=(const StampedDigitalElevationMap_CV & other);

  DEFINE_MSG_POINTERS(StampedDigitalElevationMap_CV)
};

template<>
struct rclcpp::TypeAdapter<StampedDigitalElevationMap_CV, dem_msgs::msg::DigitalElevationMap>
{
  using is_specialized = std::true_type;
  using custom_type = StampedDigitalElevationMap_CV;
  using ros_message_type = dem_msgs::msg::DigitalElevationMap;
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};


#endif  // NATIVE_ADAPTERS__DIGITALELEVATIONMAP_CVMAT_HPP_
