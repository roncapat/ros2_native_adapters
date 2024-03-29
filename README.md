# ROS2 Native Adapters

[![Ubuntu 22.04 Humble Build](https://github.com/roncapat/ros2_native_adapters/actions/workflows/humble.yml/badge.svg)](https://github.com/roncapat/ros2_native_adapters/actions/workflows/humble.yml)
[![Ubuntu 22.04 Iron Build](https://github.com/roncapat/ros2_native_adapters/actions/workflows/iron.yml/badge.svg)](https://github.com/roncapat/ros2_native_adapters/actions/workflows/iron.yml)
[![Ubuntu 22.04 Rolling Build](https://github.com/roncapat/ros2_native_adapters/actions/workflows/rolling.yml/badge.svg?branch=iron)](https://github.com/roncapat/ros2_native_adapters/actions/workflows/rolling.yml)

This package provides the following ROS2 type adaptations:

| Adapted Message | Native wrapper | Underlying native class | Header | CMake target |
|-|-|-|-|-|
| sensor_msgs/PointCloud2 | StampedPointCloud_PCL | pcl::PointCloud\<PointT\> |PCL.hpp| native_adapters::pcl_adapters |
| sensor_msgs/Image | StampedImage_CV | cv::Mat |CV.hpp| native_adapters::cv_adapters |
| nav_msgs/OccupancyGrid | StampedOccupancyGrid_CV | cv::Mat |CV.hpp| native_adapters::cv_adapters |
| dem_msgs/DigitalElevationMap | StampedDigitalElevationMap_CV | cv::Mat |CV.hpp| native_adapters::cv_adapters |

## Wrapper philosophy
Each wrapper class includes at least an `std_msgs::Header` attribute and a native handle.

## Examples
See [here](https://github.com/roncapat/ros2-native-adapters-examples).

## Pre-commit
This project uses [pre-commit](https://pre-commit.com/).  On Ubuntu, install it with:
```
sudo python3 -m pip install pre-commit
```
Then, enter the repository root and run :
```
pre-commit install
```
Now every time a commit is issued, a number of automated checks are done on code.
