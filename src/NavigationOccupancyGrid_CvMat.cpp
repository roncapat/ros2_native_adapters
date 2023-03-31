#include "native_adapters/NavigationOccupancyGrid_CvMat.hpp"
#include <iostream>

StampedOccupancyGrid_CV::StampedOccupancyGrid_CV(const StampedOccupancyGrid_CV &other){      
  std::cerr << "[OccupancyGrid Adapter] Copy constructor called" << std::endl;
  //raise(SIGTRAP);
  this->header = other.header;
  this->info = other.info;
  this->mat = other.mat.clone();
}

StampedOccupancyGrid_CV::StampedOccupancyGrid_CV(StampedOccupancyGrid_CV &&other){
  std::cerr << "[OccupancyGrid Adapter] Move constructor called" << std::endl;
  //raise(SIGTRAP);
  this->header = std::move(other.header);
  this->info = std::move(other.info);
  this->mat = std::move(other.mat);
}

StampedOccupancyGrid_CV & StampedOccupancyGrid_CV::operator=(const StampedOccupancyGrid_CV & other){
  std::cerr << "[OccupancyGrid Adapter] Assignment operator called" << std::endl;
  //raise(SIGTRAP);
  if (this == &other) return *this; 
  this->header = other.header;
  this->info = other.info;
  this->mat = other.mat.clone();
  return *this;
}
/*
template<>
struct rclcpp::TypeAdapter<StampedCvMat, nav_msgs::msg::OccupancyGrid>
{
  using is_specialized = std::true_type;
  using custom_type = StampedCvMat;
  using ros_message_type = nav_msgs::msg::OccupancyGrid;

  static void convert_to_ros_message (const custom_type & source, ros_message_type & destination){
    std::cerr << "[OccupancyGrid Adapter] Conversion to message" << std::endl;
    //raise(SIGTRAP);

    destination.data.resize(source.mat.rows * source.mat.cols);
    for(int y=0; y<source.mat.rows; ++y){
        for(int x=0; x<source.mat.cols; ++x){
            destination.data[y*source.mat.cols+x] = source.mat.at<uint8_t>(y,x) - 1;
        }
    }
    destination.header = source.header;
    destination.info = source.info;
  }

  static void convert_to_custom (const ros_message_type & source, custom_type & destination){
    std::cerr << "[OccupancyGrid Adapter] Conversion from message" << std::endl;
    //raise(SIGTRAP);

    destination.mat = cv::Mat (source.info.height, source.info.width, CV_MAKETYPE(cv::DataType<uint8_t>::type, 1));
    for(unsigned int y=0; y<source.info.height; ++y){
        for(unsigned int x=0; x<source.info.width; ++x){
            destination.mat.at<uint8_t>(y,x) = source.data[y*source.info.width+x] + 1;
        }
    }
    destination.header = source.header;
    destination.info = source.info;
  }
};
*/

void rclcpp::TypeAdapter<StampedOccupancyGrid_CV, nav_msgs::msg::OccupancyGrid>::convert_to_ros_message (const StampedOccupancyGrid_CV & source, nav_msgs::msg::OccupancyGrid & destination){
  std::cerr << "[OccupancyGrid Adapter] Conversion to message" << std::endl;
  //raise(SIGTRAP);

  destination.data.resize(source.mat.rows * source.mat.cols);
  for(int y=0; y<source.mat.rows; ++y){
      for(int x=0; x<source.mat.cols; ++x){
          destination.data[y*source.mat.cols+x] = source.mat.at<uint8_t>(y,x) - 1;
      }
  }
  destination.header = source.header;
  destination.info = source.info;
}

void rclcpp::TypeAdapter<StampedOccupancyGrid_CV, nav_msgs::msg::OccupancyGrid>::convert_to_custom (const nav_msgs::msg::OccupancyGrid & source, StampedOccupancyGrid_CV & destination){
  std::cerr << "[OccupancyGrid Adapter] Conversion from message" << std::endl;
  //raise(SIGTRAP);

  destination.mat = cv::Mat (source.info.height, source.info.width, CV_MAKETYPE(cv::DataType<uint8_t>::type, 1));
  for(unsigned int y=0; y<source.info.height; ++y){
      for(unsigned int x=0; x<source.info.width; ++x){
          destination.mat.at<uint8_t>(y,x) = source.data[y*source.info.width+x] + 1;
      }
  }
  destination.header = source.header;
  destination.info = source.info;
}