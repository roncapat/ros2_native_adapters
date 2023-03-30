#ifndef TRAVERSABILITY_TOOLKIT__ADAPTERS__SENSORMSGIMAGE_CVMAT
#define TRAVERSABILITY_TOOLKIT__ADAPTERS__SENSORMSGIMAGE_CVMAT

#include <rclcpp/type_adapter.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/mat.hpp>

struct CustomCvMat {

  std_msgs::msg::Header header;
  cv::Mat mat;

  CustomCvMat() = default;
  CustomCvMat(const CustomCvMat &other);
  CustomCvMat(CustomCvMat &&other);
  CustomCvMat & operator=(const CustomCvMat & other);

};

template<>
struct rclcpp::TypeAdapter<CustomCvMat, sensor_msgs::msg::Image>{
  using is_specialized = std::true_type;
  using custom_type = CustomCvMat;
  using ros_message_type = sensor_msgs::msg::Image;
  static void convert_to_ros_message (const custom_type & source, ros_message_type & destination);
  static void convert_to_custom (const ros_message_type & source, custom_type & destination);
};


#endif