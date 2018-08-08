#ifndef CAMERACENTERED_SYNC_H
#define CAMERACENTERED_SYNC_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

// PCL stuff
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV stuff
#include <opencv2/opencv.hpp>

// Basic stuff
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <ctime>
#include <math.h>

// Boost
#include <boost/format.hpp>

using namespace message_filters;

class CameraCenteredSync
{
public:
	explicit CameraCenteredSync();
	~CameraCenteredSync();

protected:
private:	
	message_filters::Subscriber<sensor_msgs::Image> Camera;
	message_filters::Subscriber<nav_msgs::Odometry> GPS;
	message_filters::Cache<snav_msgs::Odometry> cache_GPS;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_1;
    message_filters::Cache<sensor_msgs::PointCloud2> cache_L1;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_2;
    message_filters::Cache<sensor_msgs::PointCloud2> cache_L2;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_3;
    message_filters::Cache<sensor_msgs::PointCloud2> cache_L3;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_4;
    message_filters::Cache<sensor_msgs::PointCloud2> cache_L4;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_5;
    message_filters::Cache<sensor_msgs::PointCloud2> cache_L5;
};

#endif