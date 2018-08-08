#include "CameraCenteredSync.h"

using namespace message_filters;

CameraCenteredSync::CameraCenteredSync(const int& lidarNums)
								:firstSeq(-1),
                                calib_parsed(false),
                                ptrCloud_4(new pcl::PointCloud<pcl::PointXYZI>()),
                                ptrCloud_3(new pcl::PointCloud<pcl::PointXYZI>()),
                                ptrCloud_5(new pcl::PointCloud<pcl::PointXYZI>()),
                                ptrCloud_1(new pcl::PointCloud<pcl::PointXYZI>()),
                                ptrCloud_2(new pcl::PointCloud<pcl::PointXYZI>()),                              
                                imageDir("/home/vince/Documents/Mcity/DensoData/Images/"),
                                lidarDir("/home/vince/Documents/Mcity/DensoData/Pointcloud/") //need current directory
{
	lidar_number = lidarNums;
    Camera.subscribe(nh, "/image_raw", 5);
    GPS.subscribe(nh, "/odom", 5);
    Lidar_4.subscribe(nh,"/pointcloud04",10);
    if (lidarNums == 3) {

    } else if( lidarNums == 5) {
        

    } else if(lidarNums == 1){

        sync1.reset(new Sync1(MySyncPolicy1(10), Camera, GPS, Lidar_4));   
        sync1->registerCallback(boost::bind(&LidarCameraSync::callback_1lidar, this, _1, _2, _3));
        
    } else ROS_ERROR("Invalid lidar number, please input 1, 3 or 5");

    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse", 1);
    pub_pointcloud_wo_correction = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse_wo_correction", 1);
}
