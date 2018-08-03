#include "LidarCameraSync.h"

using namespace message_filters;

LidarCameraSync::LidarCameraSync(const int& lidarNums, const int lidar_Hz, const bool _correct_distortion)
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
    correct_distortion = _correct_distortion;
    lidar_period = 1.0/lidar_Hz;
    lidar_number = lidarNums;
    Camera.subscribe(nh, "/image_raw", 5);
    GPS.subscribe(nh, "/odom", 5);
    Lidar_4.subscribe(nh,"/pointcloud04",10);

    // parse_Calibration();

    if (lidarNums == 3) {

        Lidar_3.subscribe(nh,"/pointcloud03",10);
        Lidar_5.subscribe(nh,"/pointcloud05",10);
        sync3.reset(new Sync3(MySyncPolicy3(10), Camera, GPS, Lidar_3, Lidar_4, Lidar_5));   
        sync3->registerCallback(boost::bind(&LidarCameraSync::callback_3lidar, this, _1, _2, _3, _4, _5));

    } else if( lidarNums == 5) {
        
        Lidar_3.subscribe(nh,"/pointcloud03",10);
        Lidar_5.subscribe(nh,"/pointcloud05",10);
        Lidar_1.subscribe(nh,"/pointcloud01",10);
        Lidar_2.subscribe(nh,"/pointcloud02",10);
        sync5.reset(new Sync5(MySyncPolicy5(10), Camera, GPS, Lidar_1, Lidar_2, Lidar_3, Lidar_4, Lidar_5));   
        sync5->registerCallback(boost::bind(&LidarCameraSync::callback_5lidar, this, _1, _2, _3, _4, _5, _6, _7));

    } else if(lidarNums == 1){

        sync1.reset(new Sync1(MySyncPolicy1(10), Camera, GPS, Lidar_4));   
        sync1->registerCallback(boost::bind(&LidarCameraSync::callback_1lidar, this, _1, _2, _3));
        
    } else ROS_ERROR("Invalid lidar number, please input 1, 3 or 5");

    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse", 1);
    pub_pointcloud_wo_correction = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse_wo_correction", 1);

}


void LidarCameraSync::callback_1lidar(const sensor_msgs::ImageConstPtr& Camera_msg,
                                    const nav_msgs::Odometry::ConstPtr& GPS_msg, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04)
{
    update_variables(Camera_msg, GPS_msg, Lidar_msg_04);

    ROS_INFO_STREAM("Camera_msg received at " << Camera_msg->header.stamp<<" Seq: "<<Camera_msg->header.seq);
    ROS_INFO_STREAM("GPS_msg received at " << GPS_msg->header.stamp<<" Seq: "<<GPS_msg->header.seq);
    ROS_INFO_STREAM("Lidar_msg_04 received at " << Lidar_msg_04->header.stamp<<" Seq: "<<Lidar_msg_04->header.seq);

    pcl::fromROSMsg(*Lidar_msg_04,*ptrCloud_4);
    // write_camera(Camera_msg);
    // write_Lidar(ptrCloud_4);
    pcl::toROSMsg(*ptrCloud_4, output);
    pub_pointcloud_wo_correction.publish(output);

    transform_pointcloud();

    pcl::toROSMsg(*ptrCloud_4, output);
    pub_pointcloud.publish(output);

    ROS_INFO_STREAM("Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms, Frames: "<<((Camera_msg->header.seq) - firstSeq+1));
}
void LidarCameraSync::callback_3lidar(const sensor_msgs::ImageConstPtr& Camera_msg,
                                    const nav_msgs::Odometry::ConstPtr& GPS_msg,
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_03, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_05)
{
    // start = std::clock();
    // if(firstSeq == -1)firstSeq = Camera_msg->header.seq;
    // frameNums = Camera_msg->header.seq - firstSeq + 1;
    update_variables(Camera_msg, GPS_msg, Lidar_msg_04);

    if (!calib_parsed){

        cal_Matrix_03<<0.940413,   0.0348746,   -0.338241,   -0.473188,
                    -0.031771,    0.999387,   0.0147095,   0.0110943,
                    0.338547, -0.00308675,    0.940944,  -0.114471, 
                    0.0,           0.0,          0.0,           1.0;
        cal_Matrix_05<<0.958359,  0.0174144,   0.285036,   0.471749,
                    -0.0100379,   0.999576, -0.0273196, -0.0179415,
                    -0.285391,  0.0233208 ,  0.958127, -0.0886846,
                    0.0,          0.0 ,         0.0,          1.0; 
        calib_parsed = true;
    }
    ROS_INFO_STREAM("Camera_msg received at " << Camera_msg->header.stamp<<" Seq: "<<Camera_msg->header.seq);
    ROS_INFO_STREAM("GPS_msg received at " << GPS_msg->header.stamp<<" Seq: "<<GPS_msg->header.seq);
    ROS_INFO_STREAM("Lidar_msg_03 received at " << Lidar_msg_03->header.stamp<<" Seq: "<<Lidar_msg_03->header.seq);
    ROS_INFO_STREAM("Lidar_msg_04 received at " << Lidar_msg_04->header.stamp<<" Seq: "<<Lidar_msg_04->header.seq);
    ROS_INFO_STREAM("Lidar_msg_05 received at " << Lidar_msg_04->header.stamp<<" Seq: "<<Lidar_msg_05->header.seq);

    pcl::fromROSMsg(*Lidar_msg_03,*ptrCloud_3);
    pcl::fromROSMsg(*Lidar_msg_04,*ptrCloud_4);
    pcl::fromROSMsg(*Lidar_msg_05,*ptrCloud_5);

    transform_pointcloud();

    *ptrCloud_4 += *ptrCloud_3;
    *ptrCloud_4 += *ptrCloud_5;

    // write_camera(Camera_msg);
    // write_Lidar(ptrCloud_4);

    pcl::toROSMsg(*ptrCloud_4, output);
    pub_pointcloud.publish(output);

    ROS_INFO_STREAM("Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms, Frames: "<< frameNums);

}
void LidarCameraSync::callback_5lidar(const sensor_msgs::ImageConstPtr& Camera_msg,
                                    const nav_msgs::Odometry::ConstPtr& GPS_msg,
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_01,
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_02,
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_03, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_05)
{
    // start = std::clock();
    // if(firstSeq == -1)firstSeq = Camera_msg->header.seq;
    // frameNums = Camera_msg->header.seq - firstSeq + 1;
    // // Eigen::Affine3f _CL_1 = pcl::getTransformation(0,0,0,0,0,-1.5708);
    update_variables(Camera_msg, GPS_msg, Lidar_msg_04);
    if (!calib_parsed){

        cal_Matrix_01<<-0.07775,  0.971848, -0.222408,  0.813054,
                    -0.966705, -0.128043, -0.221559,   2.22258,
                    -0.243798,  0.197776,  0.949445,  -1.24362,
                    0,          0,          0,          1;
        cal_Matrix_02<<0.412873,  0.878207,  0.241429, -0.787354,
                    -0.871087,   0.45816, -0.176906,   2.20536,
                    -0.265974, -0.137267,  0.954158,  -1.25976,
                        0,         0,         0,         1;
        cal_Matrix_03<<0.940413,   0.0348746,   -0.338241,   -0.473188,
                    -0.031771,    0.999387,   0.0147095,   0.0110943,
                    0.338547, -0.00308675,    0.940944,  -0.114471, 
                    0.0,           0.0,          0.0,           1.0;
        cal_Matrix_05<<0.958359,  0.0174144,   0.285036,   0.471749,
                    -0.0100379,   0.999576, -0.0273196, -0.0179415,
                    -0.285391,  0.0233208 ,  0.958127, -0.0886846,
                    0.0,          0.0 ,         0.0,          1.0; 
        calib_parsed = true;
    }
    

    // Topic Info
    ROS_INFO_STREAM("Camera_msg received at " << Camera_msg->header.stamp.toSec()<<" Seq: "<<Camera_msg->header.seq);
    ROS_INFO_STREAM("GPS_msg received at " << GPS_msg->header.stamp.toSec()<<" Seq: "<<GPS_msg->header.seq);
    ROS_INFO_STREAM("Lidar_msg_01 received at " << Lidar_msg_01->header.stamp<<" Seq: "<<Lidar_msg_03->header.seq);
    ROS_INFO_STREAM("Lidar_msg_02 received at " << Lidar_msg_02->header.stamp<<" Seq: "<<Lidar_msg_04->header.seq);
    ROS_INFO_STREAM("Lidar_msg_03 received at " << Lidar_msg_03->header.stamp<<" Seq: "<<Lidar_msg_03->header.seq);
    ROS_INFO_STREAM("Lidar_msg_04 received at " << Lidar_msg_04->header.stamp<<" Seq: "<<Lidar_msg_04->header.seq);
    ROS_INFO_STREAM("Lidar_msg_05 received at " << Lidar_msg_04->header.stamp<<" Seq: "<<Lidar_msg_05->header.seq);
    // Record first seq
    // Process Pointcloud

    pcl::fromROSMsg(*Lidar_msg_01,*ptrCloud_1);
    pcl::fromROSMsg(*Lidar_msg_02,*ptrCloud_2);
    pcl::fromROSMsg(*Lidar_msg_03,*ptrCloud_3);
    pcl::fromROSMsg(*Lidar_msg_04,*ptrCloud_4);
    pcl::fromROSMsg(*Lidar_msg_05,*ptrCloud_5);

    transform_pointcloud();

    // Correct distortion


    //ROS_INFO_STREAM("Before appending ptcloud: Lidar_3 size: "<<ptrCloud_3->size()<<" Lidar_4 size: "<<ptrCloud_4->size()<<" Lidar_5 size: "<<ptrCloud_5->size());
    *ptrCloud_4 += *ptrCloud_3;
    *ptrCloud_4 += *ptrCloud_5;
    *ptrCloud_4 += *ptrCloud_1;
    *ptrCloud_4 += *ptrCloud_2;

    // ptrCloud_4 become the main pointcloud

    // Try to correct z axis
    // pcl::transformPointCloud (*ptrCloud_4, *ptrCloud_4, _CL_1);
    // write_camera(Camera_msg);
    // write_Lidar(ptrCloud_4);

    pcl::toROSMsg(*ptrCloud_4, output);
    pub_pointcloud.publish(output);

    ROS_INFO_STREAM("Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms, Frames: "<<((Camera_msg->header.seq) - firstSeq+1));
}
void LidarCameraSync::parse_Calibration(){

    calib_parsed = true;
}

void LidarCameraSync::write_Lidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr& ptrCloud)
{
    boost::format dst_file(lidarDir + "%06d.bin");
    dst_file % frameNums;
    _tmp = ptrCloud->getMatrixXfMap(8,8,0);
    Eigen::MatrixXf tmp(4,_tmp.cols());
    tmp << _tmp.row(0),_tmp.row(1),_tmp.row(2),(_tmp.row(4)/255.0);
    int nrow = tmp.rows();
    int ncol = tmp.cols();

    float data[nrow][ncol];
    Eigen::Map<Eigen::MatrixXf>(&data[0][0], nrow, ncol) = tmp;

    FILE * stream;
    stream = fopen(dst_file.str().c_str(),"wb");
    fwrite(data,sizeof(float),4*tmp.cols(),stream);
    fclose(stream);
}
void LidarCameraSync::write_camera(const sensor_msgs::ImageConstPtr& Camera_msg)
{
    try
    {
        cv_ptr=cv_bridge::toCvCopy(Camera_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
        return;
    }

    Image = cv_ptr->image;
    //Convert image into .jpg file
    boost::format fmt(imageDir + "%06d.jpg");
    fmt % (Camera_msg->header.seq - firstSeq);
    cv::imwrite(fmt.str(), Image);
}

void LidarCameraSync::transform_pointcloud(){

    // Things need to be done:
    // 1. solve the linear motion and z angular motion of car (done)
    // 2. get the z angle of first point and last point. (done)
    // 3. do transfermation on lidar 1,2,3,5 (done)
    // 4. solve the ratio of each point according to z angle (done)
    // 5. correct rotation
    // 6. correct translation
    // 7. update xy
    
    if (lidar_number == 1)
    {
        _transform_pointcloud(ptrCloud_4);
    } else if (lidar_number == 3){
        _transform_pointcloud(ptrCloud_4);
        _transform_pointcloud(ptrCloud_3, cal_Matrix_03);
        _transform_pointcloud(ptrCloud_5, cal_Matrix_05);
    } else if (lidar_number == 5){
        _transform_pointcloud(ptrCloud_4);
        _transform_pointcloud(ptrCloud_3, cal_Matrix_03);
        _transform_pointcloud(ptrCloud_5, cal_Matrix_05);
        _transform_pointcloud(ptrCloud_1, cal_Matrix_01);
        _transform_pointcloud(ptrCloud_2, cal_Matrix_02);
    }

    last_pose_x = pose_x;
    last_pose_y = pose_y;

    last_theta = theta;
    last_stamp = current_stamp;
}
void LidarCameraSync::_transform_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, const Eigen::Matrix4f& cal_Matrix){

    // transfer the pointcloud to lidar_4 frame
    pcl::transformPointCloud (*ptrCloud, *ptrCloud, cal_Matrix);
    // extra: filter out the point cloud on car;
    if (correct_distortion)
    {
        _undistort_pointcloud(ptrCloud);
    }
    
}
void LidarCameraSync::_transform_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud){

    if (correct_distortion)
    {
        _undistort_pointcloud(ptrCloud);
    }
}
void LidarCameraSync::_undistort_pointcloud(pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud){

    // do nothing at the first frame
    if(frameNums == 1) return;
    else{
        double _delta_t = current_stamp - last_stamp;
        double __ratio = lidar_period/_delta_t;
        double _delta_x = (pose_x - last_pose_x) * __ratio;
        double _delta_y = (pose_y - last_pose_y) * __ratio;


        double _delta_theta = angle_diff(theta , last_theta) * __ratio; //in the interval [-pi,+pi] radians

        double _theta_start = atan2((*ptrCloud)[0].y,(*ptrCloud)[0].x); //in the interval [-pi,+pi] radians
        double _theta_end = atan2((*ptrCloud)[ptrCloud->size()-1].y,(*ptrCloud)[ptrCloud->size()-1].x);
        double _full_range = angle_diff_2pi(_theta_end, _theta_start);
        if (_full_range < 1.0) // sometime the lidar point cloud range is over 2pi 
        {
            _full_range = 2.0 * M_PI + _full_range;
        }
        // std::cout << "_delta_x: "<<_delta_x<<" _delta_y: "<<_delta_y<<" _delta_theta: "<<_delta_theta <<std::endl;
        // std::cout << "_theta_start: "<<_theta_start<<" _theta_end: "<<_theta_end <<std::endl;
        for (int i = 0; i < ptrCloud->size(); ++i)
        {
            double _theta_point = atan2((*ptrCloud)[i].y,(*ptrCloud)[i].x);
            double _range_xy = sqrt(((*ptrCloud)[i].x*(*ptrCloud)[i].x)+((*ptrCloud)[i].y*(*ptrCloud)[i].y));
            double _part_range = angle_diff_2pi(_theta_point, _theta_start);
            if (i > ptrCloud->size()/1.33 && _part_range < 1.0){
                _part_range = 2.0 * M_PI + _part_range;
            }
            double _ratio = (_part_range/_full_range);

            // if (i == 0 || i == (ptrCloud->size()-1)/2||i == ptrCloud->size()-1){
            //     std::cout <<"_ratio: "<< _ratio<< " _theta_point: "<<_theta_point<<" _range_xy: "<<_range_xy<<std::endl;
            //     std::cout <<"old x: "<< (*ptrCloud)[i].x << " old y: "<<(*ptrCloud)[i].y<<std::endl;
            // }
            _theta_point = _theta_point - _delta_theta*_ratio;
            (*ptrCloud)[i].x = cos(_theta_point)*_range_xy - _delta_x*_ratio;
            (*ptrCloud)[i].y = sin(_theta_point)*_range_xy - _delta_y*_ratio;
            // if (i == 0 || i == (ptrCloud->size()-1)/2||i == ptrCloud->size()-1){
            //     std::cout <<"new x: "<< (*ptrCloud)[i].x << " new y: "<<(*ptrCloud)[i].y<<" _theta_point: "<<_theta_point<<std::endl;
            // }
        }
    }
}
void LidarCameraSync::update_variables(const sensor_msgs::ImageConstPtr& Camera_msg,
                                        const nav_msgs::Odometry::ConstPtr& GPS_msg, 
                                        const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04)
{

    start = std::clock();
    current_stamp = double(Lidar_msg_04->header.stamp.sec) + double(Lidar_msg_04->header.stamp.nsec)*1e-9;

    pose_x = GPS_msg->pose.pose.position.z;
    pose_y = GPS_msg->pose.pose.position.x;

    double q1 = GPS_msg->pose.pose.orientation.x;
    double q2 = GPS_msg->pose.pose.orientation.y;
    double q3 = GPS_msg->pose.pose.orientation.z;
    double q0 = GPS_msg->pose.pose.orientation.w;

    theta = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)); // rotation on z axis

    if(firstSeq == -1){
        firstSeq = Camera_msg->header.seq;
        last_stamp = current_stamp;
        last_pose_x = pose_x;
        last_pose_y = pose_y;
        // last_pose_z = pose_z;
        last_theta = theta;
    }
    frameNums = Camera_msg->header.seq - firstSeq + 1;
}

inline double LidarCameraSync::angle_diff(double leftAngle, double rightAngle){

    double diff = leftAngle - rightAngle;
    if(fabs(diff) > M_PI)
    {
        diff -= (diff > 0) ? M_PI*2 : M_PI*-2;
    }

    return diff;
}
inline double LidarCameraSync::angle_diff_2pi(double leftAngle, double rightAngle){

    double diff = angle_diff(leftAngle,rightAngle);
    diff = wrap_to_2pi(diff);

    return diff;
}
inline double LidarCameraSync::wrap_to_pi(double angle){

    if(angle < -M_PI)
    {
        for(; angle < -M_PI; angle += 2.0*M_PI);
    }
    else if(angle > M_PI)
    {
        for(; angle > M_PI; angle -= 2.0*M_PI);
    }

    return angle;
}
inline double LidarCameraSync::wrap_to_2pi(double angle)
{
    if(angle < 0)
    {
        for(; angle < 0; angle += 2.0*M_PI);
    }
    else if(angle > 2*M_PI)
    {
        for(; angle > 2*M_PI; angle -= 2.0*M_PI);
    }

    return angle;
}