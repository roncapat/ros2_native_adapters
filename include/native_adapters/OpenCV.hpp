#ifndef TRAVERSABILITY_TOOLKIT__ADAPTERS__OCV
#define TRAVERSABILITY_TOOLKIT__ADAPTERS__OCV

#include "rclcpp/type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <opencv2/core/mat.hpp>

//#include <signal.h>

struct StampedCvMat {
    std_msgs::msg::Header header;
    nav_msgs::msg::MapMetaData info;
    cv::Mat mat;

    static const uint8_t UNKNOWN = 0;
    static const uint8_t MIN_COST = 2;
    static const uint8_t OBSTACLE = 101;

    StampedCvMat() = default;

    StampedCvMat(const StampedCvMat &other){      
      std::cerr << "[OccupancyGrid Adapter] Copy constructor called" << std::endl;
      //raise(SIGTRAP);
      this->header = other.header;
      this->info = other.info;
      this->mat = other.mat.clone();
    }

    StampedCvMat(StampedCvMat &&other){
      std::cerr << "[OccupancyGrid Adapter] Move constructor called" << std::endl;
      //raise(SIGTRAP);
      this->header = std::move(other.header);
      this->info = std::move(other.info);
      this->mat = std::move(other.mat);
    }

    StampedCvMat & operator=(const StampedCvMat & other){
      std::cerr << "[OccupancyGrid Adapter] Assignment operator called" << std::endl;
      //raise(SIGTRAP);
      if (this == &other) return *this; 
      this->header = other.header;
      this->info = other.info;
      this->mat = other.mat.clone();
    }
};

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

#endif // TRAVERSABILITY_TOOLKIT__ADAPTERS__OCV