#include <native_adapters/SensorMsgImage_CvMat.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/logging.hpp>

StampedImage_CV::StampedImage_CV(const StampedImage_CV &other) {
    RCLCPP_WARN(rclcpp::get_logger("Image Adapter"), "Copy constructor called");
    this->header = other.header;
    this->mat = other.mat.clone();
}

StampedImage_CV::StampedImage_CV(StampedImage_CV &&other) {
    RCLCPP_WARN(rclcpp::get_logger("Image Adapter"), "Move constructor called");
    this->header = std::move(other.header);
    this->mat = std::move(other.mat);
}

StampedImage_CV & StampedImage_CV::operator=(const StampedImage_CV &other) {
    RCLCPP_WARN(rclcpp::get_logger("Image Adapter"), "Assignment operator called");
    if (this == &other) return *this;

    this->header = other.header;
    this->mat = other.mat.clone();
    return *this;
}

void rclcpp::TypeAdapter<StampedImage_CV, sensor_msgs::msg::Image>::convert_to_ros_message (const custom_type & source, ros_message_type & destination){
    RCLCPP_WARN(rclcpp::get_logger("Image Adapter"), "Conversion to message");

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
    case CV_32FC1:
        encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        break;
    case CV_32FC2:
        encoding = sensor_msgs::image_encodings::TYPE_32FC2;
        break;
    case CV_32FC3:
        encoding = sensor_msgs::image_encodings::TYPE_32FC3;
        break;
    case CV_32FC4:
        encoding = sensor_msgs::image_encodings::TYPE_32FC4;
        break;
    case CV_64FC1:
        encoding = sensor_msgs::image_encodings::TYPE_64FC1;
        break;
    case CV_64FC2:
        encoding = sensor_msgs::image_encodings::TYPE_64FC2;
        break;
    case CV_64FC3:
        encoding = sensor_msgs::image_encodings::TYPE_64FC3;
        break;
    case CV_64FC4:
        encoding = sensor_msgs::image_encodings::TYPE_64FC4;
        break;
    default:
        throw std::invalid_argument("Unsupported image type.");
        break;
    }

    destination.header = source.header;
    destination = *cv_bridge::CvImage(source.header, encoding, source.mat).toImageMsg();
}

void rclcpp::TypeAdapter<StampedImage_CV, sensor_msgs::msg::Image>::convert_to_custom (const ros_message_type & source, custom_type & destination){
    RCLCPP_WARN(rclcpp::get_logger("Image Adapter"), "Conversion from message");

    destination.header = source.header;
    destination.mat = std::move(cv_bridge::toCvCopy(source, source.encoding)->image);
}
