/*
Copyright 2023 Patrick Roncagliolo, Antonino Bongiovanni
*/

#include <native_adapters/DigitalElevationMapMsg_CvMat.hpp>
#include <rclcpp/logging.hpp>

StampedDigitalElevationMap_CV::StampedDigitalElevationMap_CV(
  const StampedDigitalElevationMap_CV & other)
{
  //RCLCPP_WARN(rclcpp::get_logger("DEM Adapter"), "Copy constructor called");
  this->header = other.header;
  this->info = other.info;
  this->mat = other.mat.clone();
}

StampedDigitalElevationMap_CV::StampedDigitalElevationMap_CV(StampedDigitalElevationMap_CV && other)
{
  //RCLCPP_WARN(rclcpp::get_logger("DEM Adapter"), "Move constructor called");
  this->header = std::move(other.header);
  this->info = std::move(other.info);
  this->mat = std::move(other.mat);
}

StampedDigitalElevationMap_CV & StampedDigitalElevationMap_CV::operator=(
  const StampedDigitalElevationMap_CV & other)
{
  //RCLCPP_WARN(rclcpp::get_logger("DEM Adapter"), "Assignment operator called");
  if (this == &other) {return *this;}
  this->header = other.header;
  this->info = other.info;
  this->mat = other.mat.clone();
  return *this;
}

void rclcpp::TypeAdapter<StampedDigitalElevationMap_CV,
  dem_msgs::msg::DigitalElevationMap>::convert_to_ros_message(
  const custom_type & source, ros_message_type & destination)
{
  //RCLCPP_WARN(rclcpp::get_logger("DEM Adapter"), "Conversion to message");

  destination.data.resize(source.mat.rows * source.mat.cols);
  for (int y = 0; y < source.mat.rows; ++y) {
    for (int x = 0; x < source.mat.cols; ++x) {
      destination.data[y * source.mat.cols + x] = source.mat.at<float>(y, x) - 1;
    }
  }

  destination.header = source.header;
  destination.info = source.info;
}

void rclcpp::TypeAdapter<StampedDigitalElevationMap_CV,
  dem_msgs::msg::DigitalElevationMap>::convert_to_custom(
  const ros_message_type & source, custom_type & destination)
{
  //RCLCPP_WARN(rclcpp::get_logger("DEM Adapter"), "Conversion from message");

  destination.mat =
    cv::Mat(source.info.height, source.info.width, CV_MAKETYPE(cv::DataType<float>::type, 1));
  for (unsigned int y = 0; y < source.info.height; ++y) {
    for (unsigned int x = 0; x < source.info.width; ++x) {
      destination.mat.at<float>(y, x) = source.data[y * source.info.width + x] + 1;
    }
  }
  destination.header = source.header;
  destination.info = source.info;
}
