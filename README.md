# Lidar Obstacle Detection

## Steps

The steps of the obstacle detection project are as follows:

1. Understand the what is the road and what are the obstacles. 
This is achieved by using the Planar Segmentation method which uses 
the Random Sample Consensus (RANSAC) algorithm.

2. After figuring out which points belong to the road and which do not, 
we go ahead cluster points within a given tolerance which do not belong to the road. 

3. Finally we draw 3D bounding boxes around the clustered points.

<img src="./media/ObstacleDetectionFPS.gif" width="700" height="400" />

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

## Installation

### Linux Ubuntu 16

Install PCL, C++

The link here is very helpful,
https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

A few updates to the instructions above were needed.

* libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

* BUILD_visualization needed to be manually turned on, this link shows you how to do that,
http://www.pointclouds.org/documentation/tutorials/building_pcl.php
