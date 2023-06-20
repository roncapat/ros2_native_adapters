/*
Copyright 2023 Patrick Roncagliolo, Antonino Bongiovanni
*/

#ifndef NATIVE_ADAPTERS__SENSORMSGIMAGE_CVMAT_HPP_
#define NATIVE_ADAPTERS__SENSORMSGIMAGE_CVMAT_HPP_
#include <rclcpp/type_adapter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core/mat.hpp>
#include <native_adapters/PointerDefinesMacro.hpp>

struct StampedImage_CV
{
  std_msgs::msg::Header header;
  cv::Mat mat;

  StampedImage_CV() = default;
  StampedImage_CV(const StampedImage_CV & other);
  StampedImage_CV(StampedImage_CV && other);
  StampedImage_CV & operator=(const StampedImage_CV & other);

  DEFINE_MSG_POINTERS(StampedImage_CV)
};

template<>
struct rclcpp::TypeAdapter<StampedImage_CV, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = StampedImage_CV;
  using ros_message_type = sensor_msgs::msg::Image;
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};


#endif  // NATIVE_ADAPTERS__SENSORMSGIMAGE_CVMAT_HPP_
