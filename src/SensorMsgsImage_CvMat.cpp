#include "native_adapters/SensorMsgsImage_CvMat.hpp"

#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <iostream>

StampedImage_CV::StampedImage_CV(const StampedImage_CV &other) {
    std::cerr << "[SensorMsgsImage_CvMat Adapter] Copy constructor called" << std::endl;
    this->header = other.header;
    this->mat = other.mat.clone();
}

StampedImage_CV::StampedImage_CV(StampedImage_CV &&other) {
    std::cerr << "[SensorMsgsImage_CvMat Adapter] Move constructor called" << std::endl;
    this->header = std::move(other.header);
    this->mat = std::move(other.mat);
}

StampedImage_CV & StampedImage_CV::operator=(const StampedImage_CV &other) {
    std::cerr << "[SensorMsgsImage_CvMat Adapter] Assignment operator called" << std::endl;
    if (this == &other) return *this;

    this->header = other.header;
    this->mat = other.mat.clone();
    return *this;
}

void rclcpp::TypeAdapter<StampedImage_CV, sensor_msgs::msg::Image>::convert_to_ros_message (const custom_type & source, ros_message_type & destination){
    
    std::cerr << "[SensorMsgsImage_CvMat Adapter] Conversion to message" << std::endl;

    int cv_type {source.mat.type()};
    std::string encoding {};
    switch (cv_type)
    {
    case CV_8UC1:
        encoding = sensor_msgs::image_encodings::MONO8;
        break;
    case CV_8UC3:
        encoding = sensor_msgs::image_encodings::BGR8;
        break;
    case CV_16UC1:
        encoding = sensor_msgs::image_encodings::MONO16;
        break;
    case CV_16UC3:
        encoding = sensor_msgs::image_encodings::BGR16;
        break;
    case CV_8UC4:
        encoding = sensor_msgs::image_encodings::BGRA8;
        break;
    case CV_16UC4:
        encoding = sensor_msgs::image_encodings::BGRA16;
        break;
    default:
        throw std::invalid_argument("Unsupported image type.");
        break;
    }

    destination.header = source.header;
    destination = *cv_bridge::CvImage(source.header, encoding, source.mat).toImageMsg();
}

void rclcpp::TypeAdapter<StampedImage_CV, sensor_msgs::msg::Image>::convert_to_custom (const ros_message_type & source, custom_type & destination){
    
    std::cerr << "[SensorMsgsImage_CvMat Adapter] Conversion from message" << std::endl;

    destination.header = source.header;
    destination.mat = std::move(cv_bridge::toCvCopy(source, source.encoding)->image);
}
