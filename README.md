# Frontier_toolbox_ROS2
These packages are created to study various exploration algorithms using Nav2 and slam_toolbox with both a narrowed FOV and full FOV lidar. These packages should allow the user to quickly and easily bring up an exploration robot in a simulated or real environment using the provided nodes and libraries in this repository.

![Example_pic1](frontier_exp_cpp/images/frontier_demo.gif)![Example_pic2](frontier_exp_cpp/images/frontier_demo2.gif)

## Overview
This package was created to study the performance of various frontier exploration algorithms paired wit slam_toolbox and Nav2 with both 360 degree and narrowed Field of View (FOV) on the LaserScan. This narrow FOV simulated a 2D lidar sensor that may be "damaged" or "occluded" during usage. Users can re-run my tests in simulation or on their own differential drive robots if they would like. 

For those using this package some provided features include:
- Flatten a 3D to 2D lidar scan
- Systematically and dynamically corrupt lidar data
- Utilize my frontier generation library for other frontier exploration methods
- Utilize my clustering library to cluster frontiers with DBSCAN
- Test frontier algorithms provided in this package.

## Portfolio Post
To read more about this project, please visit my [portfolio website post!](https://schelbert197.github.io/portfolio/portfolio_featured/frontier/)

## Package Dependencies
- ROS2 Humble
- Gazebo Classic
- slam_toolbox
- Nav2 (for nav_msgs)
- Robotis packages (clone from humble-devel)
    - turtlebot3
    - turtlebot3_msgs
    - turtlebot3_simulations
- PCL (Point Cloud Library)
    - pcl_conversions
    - pcl_ros (ros-humble-pcl-ros)
- libpcap-dev (to eliminate warn message)
