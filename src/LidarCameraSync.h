#ifndef LIDARCAMERA_SYNC_H
#define LIDARCAMERA_SYNC_H

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

class LidarCameraSync
{
public:
	explicit LidarCameraSync(const int& lidarNums = 1, const int lidar_Hz = 10, const bool _correct_distortion = false);
	//~LidarCameraSync();

	void callback_1lidar(const sensor_msgs::ImageConstPtr& Camera_msg,
                                    const nav_msgs::Odometry::ConstPtr& GPS_msg, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04);
	void callback_3lidar(const sensor_msgs::ImageConstPtr& Camera_msg,
                                    const nav_msgs::Odometry::ConstPtr& GPS_msg,
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_03, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_05);
	void callback_5lidar(const sensor_msgs::ImageConstPtr& Camera_msg,
                                    const nav_msgs::Odometry::ConstPtr& GPS_msg,
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_01,
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_02,
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_03, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_05);
protected:
	void parse_Calibration();
	void write_Lidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& ptrCloud);
	void write_camera(const sensor_msgs::ImageConstPtr& Camera_msg);
	void transform_pointcloud();
	void update_variables(const sensor_msgs::ImageConstPtr& Camera_msg,
                                        const nav_msgs::Odometry::ConstPtr& GPS_msg, 
                                        const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04);

private:
	void _transform_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, const Eigen::Matrix4f& cal_Matrix);
	void _transform_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud);
	void _undistort_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud);
	inline double angle_diff(double leftAngle, double rightAngle);
	inline double angle_diff_2pi(double leftAngle, double rightAngle);
	inline double wrap_to_pi(double angle);
	inline double wrap_to_2pi(double angle);

	ros::NodeHandle nh;
	ros::Publisher pub_pointcloud;
	ros::Publisher pub_pointcloud_wo_correction;

	message_filters::Subscriber<sensor_msgs::Image> Camera;

    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_1;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_2;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_3;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_4;
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_5;

    Subscriber<nav_msgs::Odometry> GPS;

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy1;
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry, sensor_msgs::PointCloud2, 
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy3;
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry, sensor_msgs::PointCloud2, 
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy5;

    typedef Synchronizer<MySyncPolicy1> Sync1;
    typedef Synchronizer<MySyncPolicy3> Sync3;
    typedef Synchronizer<MySyncPolicy5> Sync5;

    boost::shared_ptr<Sync1> sync1;
    boost::shared_ptr<Sync3> sync3;
    boost::shared_ptr<Sync5> sync5;

    bool correct_distortion;
    bool calib_parsed;
	int firstSeq;
	int lidar_number;
	size_t frameNums;

	std::string imageDir;
	std::string lidarDir;
	std::string calibDir;

	std::clock_t start;

	// std::vector<Eigen::Matrix4f> cal_Matrix;
	Eigen::Matrix4f cal_Matrix_01;
	Eigen::Matrix4f cal_Matrix_02;
	Eigen::Matrix4f cal_Matrix_03;
	Eigen::Matrix4f cal_Matrix_05;

	cv_bridge::CvImagePtr cv_ptr;

	pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_1;
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_2;
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_3;
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_4;
	pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_5;

	sensor_msgs::PointCloud2 output;
	cv::Mat Image;

	Eigen::MatrixXf _tmp;

	// for undistortion
	double lidar_period;
	double last_stamp;
	double current_stamp;
	double linear_x;
	double linear_y;
	// double linear_z;
	double last_pose_x;
	double last_pose_y;
	// double last_pose_z;
	double pose_x;
	double pose_y;
	// double pose_z;
	double last_theta;
	double theta;
	// double _deltaT;
};

#endif //LOAM_LASERMAPPING_H