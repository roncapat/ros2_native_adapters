# ROS2 Native Adapters

This package provides the following ROS2 type adaptations:

| Adapted Message | Native wrapper | Underlying native class | Header |
|-|-|-|-|
| sensor_msgs/PointCloud2 | StampedPointCloud_PCL | pcl::PointCloud\<PointT\> |CV.hpp|
| sensor_msgs/Image | StampedImage_CV | cv::Mat |CV.hpp|
| nav_msgs/OccupancyGrid | StampedOccupancyGrid_CV | cv::Mat |PCL.hpp|

## Wrapper philosophy
Each wrapper class includes at least an std_msgs::Header attribute and a native handle.

## Examples
See [here](https://github.com/roncapat/ros2-native-adapters-examples).
