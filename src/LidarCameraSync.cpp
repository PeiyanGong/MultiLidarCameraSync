#include "LidarCameraSync.h"

using namespace message_filters;

LidarCameraSync::LidarCameraSync(const int& lidarNums, const int lidar_Hz, const bool _correct_distortion, const bool publish_bbox)
								:firstSeq(-1),
                                calib_parsed(false),
                                ptrCloud_4(new pcl::PointCloud<pcl::PointXYZI>()),
                                ptrCloud_3(new pcl::PointCloud<pcl::PointXYZI>()),
                                ptrCloud_5(new pcl::PointCloud<pcl::PointXYZI>()),
                                ptrCloud_1(new pcl::PointCloud<pcl::PointXYZI>()),
                                ptrCloud_2(new pcl::PointCloud<pcl::PointXYZI>()),
                                predDir("/home/vince/Documents/Mcity/Sync/data/"),                              
                                imageDir("/home/vince/Documents/Mcity/DensoData/Images_unrect/"),
                                lidarDir("/home/vince/Documents/Mcity/DensoData/Pointcloud_unrect/") //need current directory
{
    publish_3dbbox = publish_bbox;
    correct_distortion = _correct_distortion;
    lidar_period = 1.0/lidar_Hz;
    lidar_number = lidarNums;
    Camera.subscribe(nh, "/image_raw", 5);
    GPS.subscribe(nh, "/odom", 5);
    Lidar_4.subscribe(nh,"/pointcloud04",10);
    std::vector<std::vector<int>> tmp_map{{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
    edge_map = tmp_map;
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

        // sync1.reset(new Sync1(MySyncPolicy1(10), Camera, GPS, Lidar_4)); 
        sync1.reset(new Sync1(MySyncPolicy1(10), Camera, Lidar_4));  
        sync1->registerCallback(boost::bind(&LidarCameraSync::callback_1lidar, this, _1, _2));
        
    } else ROS_ERROR("Invalid lidar number, please input 1, 3 or 5");
    if (publish_3dbbox)
    {
        pred_bbox = nh.advertise<visualization_msgs::Marker>("/pred_bbox",1);
    }
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse", 1);
    pub_pointcloud_wo_correction = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse_wo_correction", 1);

}


void LidarCameraSync::callback_1lidar(const sensor_msgs::ImageConstPtr& Camera_msg,
                                    // const sensor_msgs::ImuConstPtr& GPS_msg, 
                                    const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04)
{
    start = std::clock();
    current_stamp = double(Lidar_msg_04->header.stamp.sec) + double(Lidar_msg_04->header.stamp.nsec)*1e-9;

    // pose_x = GPS_msg->pose.pose.position.z;
    // pose_y = GPS_msg->pose.pose.position.x;

    // double q1 = GPS_msg->pose.pose.orientation.x;
    // double q2 = GPS_msg->pose.pose.orientation.y;
    // double q3 = GPS_msg->pose.pose.orientation.z;
    // double q0 = GPS_msg->pose.pose.orientation.w;

    // theta = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)); // rotation on z axis

    if(firstSeq == -1){
        firstSeq = Camera_msg->header.seq;
        frameNums = -1;
        last_stamp = current_stamp;
        last_pose_x = pose_x;
        last_pose_y = pose_y;
        // last_pose_z = pose_z;
        last_theta = theta;
    }
    if (publish_3dbbox)
    {
        frameNums++;
    }
    if (frameNums == 177) frameNums =181; // lost 5 frame ptrcloud 
    if (!calib_parsed){

        cal_Matrix_LC<<7.533745e-03, -9.999714e-01, -6.166020e-04, -4.069766e-03,
                    1.480249e-02, 7.280733e-04, -9.998902e-01, -7.631618e-02,
                     9.998621e-01, 7.523790e-03, 1.480755e-02, -2.717806e-01,
                     0.0,           0.0,          0.0,           1.0;
        cal_Matrix_CL_R_rec<<9.999239e-01, 9.837760e-03, -7.445048e-03,
                         -9.869795e-03, 9.999421e-01, -4.278459e-03,
                          7.402527e-03, 4.351614e-03, 9.999631e-01;
        // R_rect_00: 9.999239e-01 9.837760e-03 -7.445048e-03 -9.869795e-03 9.999421e-01 -4.278459e-03 7.402527e-03 4.351614e-03 9.999631e-01
        Eigen::Matrix4f tmp = cal_Matrix_LC.inverse();
        cal_Matrix_CL_R = tmp.block<3,3>(0,0);
        cal_Matrix_CL_t <<tmp(0,3),tmp(1,3),tmp(2,3);
        calib_parsed = true;
    }

    ROS_INFO_STREAM("Camera_msg received at " << Camera_msg->header.stamp<<" Seq: "<<Camera_msg->header.seq);
    // ROS_INFO_STREAM("GPS_msg received at " << GPS_msg->header.stamp<<" Seq: "<<GPS_msg->header.seq);
    ROS_INFO_STREAM("Lidar_msg_04 received at " << Lidar_msg_04->header.stamp<<" Seq: "<<Lidar_msg_04->header.seq);

    pcl::fromROSMsg(*Lidar_msg_04,*ptrCloud_4);
    if (publish_3dbbox){
        publish_prediction();
    } else {
        write_camera(Camera_msg);
        write_Lidar(ptrCloud_4);   
    }
    pcl::toROSMsg(*ptrCloud_4, output);
    pub_pointcloud_wo_correction.publish(output);

    transform_pointcloud();

    pcl::toROSMsg(*ptrCloud_4, output);
    pub_pointcloud.publish(output);

    ROS_INFO_STREAM("Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms, Frames: "<<frameNums);
}
void LidarCameraSync::callback_3lidar(const sensor_msgs::ImageConstPtr& Camera_msg,
                                    const sensor_msgs::ImuConstPtr& GPS_msg,
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
        cal_Matrix_LC<<0.99757, -0.0689411, -0.00999908, -0.126208,
                    -0.00379972, 0.0894736, -0.995982, -0.726174,
                     0.0695588, 0.9936, 0.0889943, -0.664635,
                     0.0,           0.0,          0.0,           1.0;
        Eigen::Matrix4f tmp = cal_Matrix_LC.inverse();
        cal_Matrix_CL_R = tmp.block<3,3>(0,0);
        cal_Matrix_CL_t <<tmp(0,3),tmp(1,3),tmp(2,3);
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

    if (publish_3dbbox){
        publish_prediction();
    } else {
        write_camera(Camera_msg);
        write_Lidar(ptrCloud_4);   
    }
    
    pcl::toROSMsg(*ptrCloud_4, output);
    pub_pointcloud.publish(output);

    ROS_INFO_STREAM("Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms, Frames: "<< frameNums);

}
void LidarCameraSync::callback_5lidar(const sensor_msgs::ImageConstPtr& Camera_msg,
                                    const sensor_msgs::ImuConstPtr& GPS_msg,
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
        cal_Matrix_LC<<0.99757, -0.0689411, -0.00999908, -0.126208,
                    -0.00379972, 0.0894736, -0.995982, -0.726174,
                     0.0695588, 0.9936, 0.0889943, -0.664635,
                     0.0,           0.0,          0.0,           1.0;
        Eigen::Matrix4f tmp = cal_Matrix_LC.inverse();
        cal_Matrix_CL_R = tmp.block<3,3>(0,0);
        cal_Matrix_CL_t <<tmp(0,3),tmp(1,3),tmp(2,3);
        calib_parsed = true;
    }
    

    // Topic Info
    ROS_INFO_STREAM("Camera_msg received at " << Camera_msg->header.stamp<<" Seq: "<<Camera_msg->header.seq);
    ROS_INFO_STREAM("GPS_msg received at " << GPS_msg->header.stamp<<" Seq: "<<GPS_msg->header.seq);
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
    if (publish_3dbbox){
        publish_prediction();
    } else {
        write_camera(Camera_msg);
        write_Lidar(ptrCloud_4);   
    }

    pcl::toROSMsg(*ptrCloud_4, output);
    pub_pointcloud.publish(output);

    ROS_INFO_STREAM("Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms, Frames: "<<((Camera_msg->header.seq) - firstSeq+1));
}
void LidarCameraSync::parse_Calibration(){ // I am too lazy to write this...

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
    fmt % frameNums;
    cv::imwrite(fmt.str(), Image);
}

void LidarCameraSync::publish_prediction(){
    boost::format fmt(predDir + "%06d.txt");
    fmt % frameNums;
    std::vector<std::array<float,16>> prediction = read_prediction(fmt.str());
    // do things to publish markers
    // markers format: [0] class, [1].[2],[3] don't care, [4],[5],[6],[7] 2d bbox xmin, ymin, xman, ymax
    // [8] height [9] width [10] length [11][12][13] location xyz [14] rotation-y [15] score (ratio)
    if (prediction.empty())return;
    visualization_msgs::Marker bbox;
    bbox.header.frame_id = "velo_link";
    bbox.header.stamp = ros::Time::now();
    bbox.ns = "prediction";
    bbox.lifetime = ros::Duration(0.11);
    bbox.id = 0;
    bbox.type = visualization_msgs::Marker::LINE_LIST;
    bbox.action = visualization_msgs::Marker::ADD;
    bbox.scale.x = 0.1;
    bbox.color.g = 1.0;
    bbox.color.a = 1.0;
    for (auto it = prediction.begin(); it != prediction.end(); it++){ // for each 3d bbox
        if (it->empty()) continue; // it is not supposed to be empty but who knows.
        Eigen::Matrix3f R;
        Eigen::MatrixXf corners(3,8);
        float roty = (*it)[14];
        float h = (*it)[8];
        float w = (*it)[9];
        float l = (*it)[10];
        float x = (*it)[11];
        float y = (*it)[12];
        float z = (*it)[13];
        R = Eigen::AngleAxisf(roty,  Eigen::Vector3f::UnitY());
        corners<<l/2.0, l/2.0, -l/2.0, -l/2.0, l/2.0, l/2.0, -l/2.0, -l/2.0,
                0.0, 0.0, 0.0, 0.0, -h, -h, -h, -h,
                w/2.0, -w/2.0, -w/2.0, w/2.0, w/2.0, -w/2.0, -w/2.0, w/2.0;
        // std::cout<<"corners after init: "<<corners(0,0)<<std::endl;
        corners = R*corners;
        // std::cout<<"corners after R: "<<corners(0,0)<<std::endl;
        // Add translation x,y,z
        corners.row(0) += x*Eigen::MatrixXf::Ones(1,8);
        corners.row(1) += y*Eigen::MatrixXf::Ones(1,8);
        corners.row(2) += z*Eigen::MatrixXf::Ones(1,8);
        // std::cout<<"corners after translation: "<<corners(0,0)<<std::endl;
        corners = cal_Matrix_CL_R_rec.inverse() * corners;

        // From reference frame to velodyne frame
        corners = cal_Matrix_CL_R * corners;
        // std::cout<<"corners after CL_R: "<<corners(0,0)<<std::endl;
        // Add translation cal_Matrix_CL_t
        corners.row(0) += cal_Matrix_CL_t(0)*Eigen::MatrixXf::Ones(1,8);
        corners.row(1) += cal_Matrix_CL_t(1)*Eigen::MatrixXf::Ones(1,8);
        corners.row(2) += cal_Matrix_CL_t(2)*Eigen::MatrixXf::Ones(1,8);
        // std::cout<<"corners after CL_t: "<<corners(0,0)<<std::endl;
        // Add 12 edges
        // 0-1,1-2,2-3,3-0//4-5,5-6,6-7,7-4//0-4,1-5,2-6,3-7//
        for (int i = 0; i<12;i++){ // for each edges
            geometry_msgs::Point tmp_p1;
            geometry_msgs::Point tmp_p2;
            tmp_p1.x = corners(0,edge_map[i][0]);
            tmp_p1.y = corners(1,edge_map[i][0]);
            tmp_p1.z = corners(2,edge_map[i][0]);
            tmp_p2.x = corners(0,edge_map[i][1]);
            tmp_p2.y = corners(1,edge_map[i][1]);
            tmp_p2.z = corners(2,edge_map[i][1]);
            bbox.points.push_back(tmp_p1);
            bbox.points.push_back(tmp_p2);
        }
        
    }
    pred_bbox.publish(bbox);
    
}
std::vector<std::array<float,16>> LidarCameraSync::read_prediction(std::string name){
    std::string line;
    std::vector<std::array<float,16>> result;
    std::ifstream myfile(name);
    // std::cout<<"name: "<<name<<std::endl;
    if (myfile.is_open()) {
        while (std::getline(myfile,line)){
            std::array<float, 16> tmp;
            // cout << line << '\n';
            // from line to array
            std::istringstream iss(line);
            for (int i = 0;i < 16; i++){
                std::string tmp_string;
                iss >> tmp_string;
                if(i == 0){
                    if (tmp_string == "Car") tmp[i] = 1;
                    else if (tmp_string == "Pedestrian") tmp[i] = 2;
                    else tmp[i] = 0;
                }else{
                    tmp[i] = std::stof(tmp_string);
                }
                // std::cout<<tmp[i]<<" ";
            }
            // std::cout<<std::endl;
            result.push_back(tmp);
        }
    myfile.close();
    }
    else ROS_ERROR("Unable to open prediction file: %s ",name.c_str()); 
    return result;
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
    // if(frameNums == 0) return;
    // else{
    //     double _delta_t = current_stamp - last_stamp;
    //     double __ratio = lidar_period/_delta_t;
    //     double _delta_x = (pose_x - last_pose_x) * __ratio;
    //     double _delta_y = (pose_y - last_pose_y) * __ratio;


    //     double _delta_theta = angle_diff(theta , last_theta) * __ratio; //in the interval [-pi,+pi] radians

    //     double _theta_start = atan2((*ptrCloud)[0].y,(*ptrCloud)[0].x); //in the interval [-pi,+pi] radians
    //     double _theta_end = atan2((*ptrCloud)[ptrCloud->size()-1].y,(*ptrCloud)[ptrCloud->size()-1].x);
    //     double _full_range = angle_diff_2pi(_theta_end, _theta_start);
    //     if (_full_range < 1.0) // sometime the lidar point cloud range is over 2pi 
    //     {
    //         _full_range = 2.0 * M_PI + _full_range;
    //     }
    //     // std::cout << "_delta_x: "<<_delta_x<<" _delta_y: "<<_delta_y<<" _delta_theta: "<<_delta_theta <<std::endl;
    //     // std::cout << "_theta_start: "<<_theta_start<<" _theta_end: "<<_theta_end <<std::endl;
    //     for (int i = 0; i < ptrCloud->size(); ++i)
    //     {
    //         double _theta_point = atan2((*ptrCloud)[i].y,(*ptrCloud)[i].x);
    //         double _range_xy = sqrt(((*ptrCloud)[i].x*(*ptrCloud)[i].x)+((*ptrCloud)[i].y*(*ptrCloud)[i].y));
    //         double _part_range = angle_diff_2pi(_theta_point, _theta_start);
    //         if (i > ptrCloud->size()/1.33 && _part_range < 1.0){
    //             _part_range = 2.0 * M_PI + _part_range;
    //         }
    //         double _ratio = (_part_range/_full_range);

    //         // if (i == 0 || i == (ptrCloud->size()-1)/2||i == ptrCloud->size()-1){
    //         //     std::cout <<"_ratio: "<< _ratio<< " _theta_point: "<<_theta_point<<" _range_xy: "<<_range_xy<<std::endl;
    //         //     std::cout <<"old x: "<< (*ptrCloud)[i].x << " old y: "<<(*ptrCloud)[i].y<<std::endl;
    //         // }
    //         _theta_point = _theta_point - _delta_theta*_ratio;
    //         (*ptrCloud)[i].x = cos(_theta_point)*_range_xy - _delta_x*_ratio;
    //         (*ptrCloud)[i].y = sin(_theta_point)*_range_xy - _delta_y*_ratio;
    //         // if (i == 0 || i == (ptrCloud->size()-1)/2||i == ptrCloud->size()-1){
    //         //     std::cout <<"new x: "<< (*ptrCloud)[i].x << " new y: "<<(*ptrCloud)[i].y<<" _theta_point: "<<_theta_point<<std::endl;
    //         // }
    //     }
    // }
}
void LidarCameraSync::update_variables(const sensor_msgs::ImageConstPtr& Camera_msg,
                                        const sensor_msgs::ImuConstPtr& GPS_msg, 
                                        const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04)
{

    start = std::clock();
    current_stamp = double(Lidar_msg_04->header.stamp.sec) + double(Lidar_msg_04->header.stamp.nsec)*1e-9;

    // pose_x = GPS_msg->pose.pose.position.z;
    // pose_y = GPS_msg->pose.pose.position.x;

    // double q1 = GPS_msg->pose.pose.orientation.x;
    // double q2 = GPS_msg->pose.pose.orientation.y;
    // double q3 = GPS_msg->pose.pose.orientation.z;
    // double q0 = GPS_msg->pose.pose.orientation.w;

    // theta = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3)); // rotation on z axis

    if(firstSeq == -1){
        firstSeq = Camera_msg->header.seq;
        frameNums = -1;
        last_stamp = current_stamp;
        last_pose_x = pose_x;
        last_pose_y = pose_y;
        // last_pose_z = pose_z;
        last_theta = theta;
    }
    if (publish_3dbbox)
    {
        frameNums++;
    }else{
        frameNums = Camera_msg->header.seq - firstSeq;
    }
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