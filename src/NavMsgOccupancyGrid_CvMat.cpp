/*
Copyright 2023 Patrick Roncagliolo
*/

#include <native_adapters/NavMsgOccupancyGrid_CvMat.hpp>
#include <rclcpp/logging.hpp>

const uint8_t StampedOccupancyGrid_CV::UNKNOWN = 0;
const uint8_t StampedOccupancyGrid_CV::FREE = 1;
const uint8_t StampedOccupancyGrid_CV::MIN_COST = 2;
const uint8_t StampedOccupancyGrid_CV::OBSTACLE = 101;

StampedOccupancyGrid_CV::StampedOccupancyGrid_CV(const StampedOccupancyGrid_CV & other)
{
  RCLCPP_WARN(rclcpp::get_logger("OccupancyGrid Adapter"), "Copy constructor called");
  // raise(SIGTRAP);
  this->header = other.header;
  this->info = other.info;
  this->mat = other.mat.clone();
}

StampedOccupancyGrid_CV::StampedOccupancyGrid_CV(StampedOccupancyGrid_CV && other)
{
  RCLCPP_WARN(rclcpp::get_logger("OccupancyGrid Adapter"), "Move constructor called");
  // raise(SIGTRAP);
  this->header = std::move(other.header);
  this->info = std::move(other.info);
  this->mat = std::move(other.mat);
}

StampedOccupancyGrid_CV & StampedOccupancyGrid_CV::operator=(const StampedOccupancyGrid_CV & other)
{
  RCLCPP_WARN(rclcpp::get_logger("OccupancyGrid Adapter"), "Assignment constructor called");
  // raise(SIGTRAP);
  if (this == &other) {return *this;}
  this->header = other.header;
  this->info = other.info;
  this->mat = other.mat.clone();
  return *this;
}

void rclcpp::TypeAdapter<StampedOccupancyGrid_CV,
  nav_msgs::msg::OccupancyGrid>::convert_to_ros_message(
  const StampedOccupancyGrid_CV & source,
  nav_msgs::msg::OccupancyGrid & destination)
{
  RCLCPP_WARN(rclcpp::get_logger("OccupancyGrid Adapter"), "Conversion to message");
  // raise(SIGTRAP);

  destination.data.resize(source.mat.rows * source.mat.cols);
  for (int y = 0; y < source.mat.rows; ++y) {
    for (int x = 0; x < source.mat.cols; ++x) {
      destination.data[y * source.mat.cols + x] =
        static_cast<uint8_t>(source.mat.at<uint8_t>(y, x) - 1);
    }
  }
  destination.header = source.header;
  destination.info = source.info;
}

void rclcpp::TypeAdapter<StampedOccupancyGrid_CV, nav_msgs::msg::OccupancyGrid>::convert_to_custom(
  const nav_msgs::msg::OccupancyGrid & source, StampedOccupancyGrid_CV & destination)
{
  RCLCPP_WARN(rclcpp::get_logger("OccupancyGrid Adapter"), "Conversion from message");
  // raise(SIGTRAP);

  destination.mat =
    cv::Mat(source.info.height, source.info.width, CV_MAKETYPE(cv::DataType<uint8_t>::type, 1));
  for (unsigned int y = 0; y < source.info.height; ++y) {
    for (unsigned int x = 0; x < source.info.width; ++x) {
      destination.mat.at<uint8_t>(y, x) = 
        static_cast<uint8_t>(source.data[y * source.info.width + x] + 1);
    }
  }
  destination.header = source.header;
  destination.info = source.info;
}
