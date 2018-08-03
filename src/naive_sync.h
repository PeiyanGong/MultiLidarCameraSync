/*
	This is a naive implementation of Lidar-Camera synchronization code. Since CAN bus and GPS are not ready yet, there 
	is no linear composition of pointcloud motion distortion. 

	6/29/2018
	Vince Gong 
	pygong@umich.edu
*/
// ROS stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

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

// Boost
#include <boost/format.hpp>

ros::Publisher pub_velo;


void write_Lidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, const std::string name);

void callback_sync(const sensor_msgs::ImageConstPtr& Camera_msg,
				const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_01,
              	const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_02,
                const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_03, 
                const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04, 
                const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_05);
// void callback_sync(const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_03, 
//                 const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04, 
//                 const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_05);