#ifndef TRAVERSABILITY_TOOLKIT__ADAPTERS__StampedImage_CV
#define TRAVERSABILITY_TOOLKIT__ADAPTERS__StampedImage_CV

#include <rclcpp/type_adapter.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/mat.hpp>

struct StampedImage_CV {

  std_msgs::msg::Header header;
  cv::Mat mat;

  StampedImage_CV() = default;
  StampedImage_CV(const StampedImage_CV &other);
  StampedImage_CV(StampedImage_CV &&other);
  StampedImage_CV & operator=(const StampedImage_CV & other);

};

template<>
struct rclcpp::TypeAdapter<StampedImage_CV, sensor_msgs::msg::Image>{
  using is_specialized = std::true_type;
  using custom_type = StampedImage_CV;
  using ros_message_type = sensor_msgs::msg::Image;
  static void convert_to_ros_message (const custom_type & source, ros_message_type & destination);
  static void convert_to_custom (const ros_message_type & source, custom_type & destination);
};


#endif // TRAVERSABILITY_TOOLKIT__ADAPTERS__StampedImage_CV