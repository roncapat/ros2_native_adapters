# ROS2 Native Adapters

This package provides the following ROS2 type adaptations:

| Adapted Message | Native wrapper | Underlying native class | Header |
|-|-|-|-|
| sensor_msgs/PointCloud2 | StampedPointCloud | pcl::PointCloud\<PointT\> |OpenCV.hpp|
| nav_msgs/OccupancyGrid | StampedCvMat | cv::Mat |PCL_2.hpp|

## Wrapper philosophy
Each wrapper class includes at least an std_msgs::Header attribute and a native handle.