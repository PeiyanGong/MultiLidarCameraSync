/*
    This is a naive implementation of Lidar-Camera synchronization code. Since CAN bus and GPS are not ready yet, there 
    is no linear composition of pointcloud motion distortion. Camera is about 4Hz and lidars are 10Hz

    6/29/2018
    Vince Gong 
    pygong@umich.edu
*/
#include "naive_sync.h"

//#define PI 3.14159265

int firstSeq = -1; // I know, I know, this is ugly, I will put everything in a class after the demo.

std::string imageDir ="/home/vince/Documents/Mcity/DensoData/Images_1/";
std::string lidarDir04 ="/home/vince/Documents/Mcity/DensoData/Pointcloud04_1/"; 
std::string lidarDir ="/home/vince/Documents/Mcity/DensoData/Pointcloud_1/"; 

void write_Lidar(const pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud, const std::string name){

	Eigen::MatrixXf _temp = ptrCloud->getMatrixXfMap(8,8,0);
    Eigen::MatrixXf temp(4,_temp.cols());
    temp<<_temp.row(0),_temp.row(1),_temp.row(2),(_temp.row(4)/255.0);
    int nrow = temp.rows();
    int ncol = temp.cols();
    //ROS_INFO_STREAM("Output Matrix size: "<< temp.size()<<" row: "<< nrow<< " col: "<<ncol);

	float data[nrow][ncol];
	Eigen::Map<Eigen::MatrixXf>(&data[0][0], nrow, ncol) = temp;

	FILE * stream;
	stream = fopen(name.c_str(),"wb");
	fwrite(data,sizeof(float),4*temp.cols(),stream);
	fclose(stream);

}

// void callback_sync(const sensor_msgs::ImageConstPtr& Camera_msg,
//                         const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_03, 
//                         const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04, 
//                         const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_05){
void callback_sync(const sensor_msgs::ImageConstPtr& Camera_msg,
                      const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_01,
                      const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_02,
                      const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_03, 
                      const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04, 
                      const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_05){
// void callback_sync(     const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_03, 
//                         const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04, 
//                         const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_05){
    // Timer
    std::clock_t start;
    start = std::clock();
    Eigen::Matrix4f Cal_Matrix_01 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Cal_Matrix_02 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Cal_Matrix_03 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Cal_Matrix_05 = Eigen::Matrix4f::Identity();
    Eigen::Affine3f _CL_1 = pcl::getTransformation(0,0,0,0,0,-1.5708);
   //  Eigen::Affine3f _CL_2 = pcl::getTransformation(0,0,0,0,0,1.5708);
   //  // Eigen::Affine3f _CL_1 = pcl::getTransformation(0,0,0,1.57,0,0);
   //  // Eigen::Affine3f _CL_2 = pcl::getTransformation(0,0,0,0,-0.029,0);
   //  // Eigen::Matrix4f CL_1 = _CL_1.matrix();
   //  // Eigen::Matrix4f CL_2 = _CL_2.matrix();
   //  // std::cout<<CL_1<<std::endl;
   //   //      1           0           0           0
   //   //  0 0.000796274          -1           0
   //   // -0           1 0.000796274           0
   //   //  0           0           0           1
   //  // std::cout<<CL_2<<std::endl;
   //   //       0.99958         -0 -0.0289959          0
   //   //         0          1         -0          0
   //   // 0.0289959          0    0.99958          0
   //   //         0          0          0          1

   //  // std::cout<<CL_2*CL_1<<std::endl; // 
   //  Eigen::Affine3f _CL = pcl::getTransformation(0,0,0,1.57,-0.029,0);
   //  Eigen::Matrix4f CL = _CL.matrix();
   //  // std::cout<<CL<<std::endl;
   // //       0.99958   -0.0289959 -2.30887e-05            0
   // //         0  0.000796274           -1            0
   // // 0.0289959     0.999579  0.000795939            0
   // //         0            0            0            1

   //  Eigen::Matrix4f RB;
   //  RB <<0.99915,0.00994418,-0.0399946,-0.126208,
   //      -0.0063695,0.996053,0.088533,-0.726174,
   //      0.0407171,-0.0882031,0.99527,-0.664635,
   //      0,0,0,1;
   //  CL = CL*_CL_2.matrix();
   //  RB = RB * CL; // RB * CL_2 * CL_1
   //  std::cout<<RB<<std::endl;

// RB with z axis correction:
 // -0.0689447    -0.99757 -0.00999908   -0.126208
 //  0.0894736   0.0037994   -0.995982   -0.726174
 //     0.9936  -0.0695624   0.0889943   -0.664635
 //          0           0           0           1


// RB without z axis correction:
//         0.99757  -0.0689411 -0.00999908   -0.126208
// -0.00379972   0.0894736   -0.995982   -0.726174
//   0.0695588      0.9936   0.0889943   -0.664635
//           0           0           0           1

    // Eigen::Affine3f TL03 = pcl::getTransformation(0.4683,0.0046,0.1409,0.0098,0.2877,0.0210);
    // Eigen::Affine3f TR05 = pcl::getTransformation(-0.4597,0.0087,0.0848,-0.002399,-0.2571,0.0035);

    // Cal_Matrix_03 = TL03.matrix().inverse();
    // Cal_Matrix_05 = TR05.matrix().inverse();

//    0.958688   0.0201354   -0.283748   -0.409066
//  -0.0182174     0.99979  0.00939706  0.00260812
//    0.283877 -0.00383971    0.958853   -0.268024
//          -0          -0          -0           1
//    0.967126  0.00338495    0.254277    0.422995
// -0.00288998    0.999993 -0.00232015 -0.00983171
//   -0.254283  0.00150902    0.967129    -0.19892
//          -0          -0          -0           1

    // std::cout<<Cal_Matrix_03<<std::endl;
    // std::cout<<Cal_Matrix_05<<std::endl;


    // TR 05
  // transform_rx = -0.002399;
  // transform_ry = -0.2571;
  // transform_rz = 0.0035;
  // transform_tx = -0.4597;
  // transform_ty = 0.0087;
  // transform_tz = 0.0848;	// For ns3 to ns4
    // TL 03    
  // transform_rx = 0.0098;
  // transform_ry = 0.2877;
  // transform_rz = 0.0210;
  // transform_tx = 0.4683;
  // transform_ty = 0.0046;
  // transform_tz = 0.1409;
//   Cal_Matrix_03 = 
//     Cal_Matrix_03<<0.944936,  0.0217603,  -0.326531,  -0.455621,
// -0.0138125,   0.999549,  0.0266392,  0.0180255,
//   0.326963, -0.0206622,   0.944811, -0.0503514, 
//     0.0,           0.0,          0.0,           1.0;
    Cal_Matrix_01<<-0.07775,  0.971848, -0.222408,  0.813054,
      -0.966705, -0.128043, -0.221559,   2.22258,
      -0.243798,  0.197776,  0.949445,  -1.24362,
             0,          0,          0,          1;
  Cal_Matrix_02<<0.412873,  0.878207,  0.241429, -0.787354,
    -0.871087,   0.45816, -0.176906,   2.20536,
    -0.265974, -0.137267,  0.954158,  -1.25976,
            0,         0,         0,         1;
	Cal_Matrix_03<<0.940413,   0.0348746,   -0.338241,   -0.473188,
	-0.031771,    0.999387,   0.0147095,   0.0110943,
	0.338547, -0.00308675,    0.940944,  -0.114471, 
	0.0,           0.0,          0.0,           1.0;
	// For ns5 to ns4 
//     Cal_Matrix_05<<   0.954769,     0.00478,    0.29731,    0.480566,
// -0.00665779,    0.999964,  0.00530383,  0.0066282,
//   -0.297274, -0.00704337,    0.954766,  0.00189982,
//     0.0,          0.0 ,         0.0,          1.0;
	Cal_Matrix_05<<0.958359,  0.0174144,   0.285036,   0.471749,
	-0.0100379,   0.999576, -0.0273196, -0.0179415,
	-0.285391,  0.0233208 ,  0.958127, -0.0886846,
	0.0,          0.0 ,         0.0,          1.0; 

    // Topic Info
    ROS_INFO_STREAM("Camera_msg received at " << Camera_msg->header.stamp.toSec()<<" Seq: "<<Camera_msg->header.seq);
    ROS_INFO_STREAM("Lidar_msg_01 received at " << Lidar_msg_01->header.stamp<<" Seq: "<<Lidar_msg_03->header.seq);
    ROS_INFO_STREAM("Lidar_msg_02 received at " << Lidar_msg_02->header.stamp<<" Seq: "<<Lidar_msg_04->header.seq);
    ROS_INFO_STREAM("Lidar_msg_03 received at " << Lidar_msg_03->header.stamp<<" Seq: "<<Lidar_msg_03->header.seq);
    ROS_INFO_STREAM("Lidar_msg_04 received at " << Lidar_msg_04->header.stamp<<" Seq: "<<Lidar_msg_04->header.seq);
    ROS_INFO_STREAM("Lidar_msg_05 received at " << Lidar_msg_04->header.stamp<<" Seq: "<<Lidar_msg_05->header.seq);
    // Record first seq
    if(firstSeq == -1)firstSeq = Camera_msg->header.seq;
    //if(firstSeq == -1)firstSeq = Lidar_msg_04->header.seq;
    // Process Pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_3(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_4(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrCloud_5(new pcl::PointCloud<pcl::PointXYZI>);

    //sensor_msgs::PointCloud2 _Lidar_msg_03 = *Lidar_msg_03;
    //sensor_msgs::PointCloud2 _Lidar_msg_04 = *Lidar_msg_04;
    //sensor_msgs::PointCloud2 _Lidar_msg_05 = *Lidar_msg_05;

    //_Lidar_msg_03.fields[3].name = "intensities";
    //_Lidar_msg_04.fields[3].name = "intensities";
    //_Lidar_msg_05.fields[3].name = "intensities";
    //ROS_INFO_STREAM(_Lidar_msg_03.fields[0].name);
    //ROS_INFO_STREAM(_Lidar_msg_03.fields[1].name);
    //ROS_INFO_STREAM(_Lidar_msg_03.fields[2].name);
    //ROS_INFO_STREAM(_Lidar_msg_03.fields[3].name);

    pcl::fromROSMsg(*Lidar_msg_01,*ptrCloud_1);
    pcl::fromROSMsg(*Lidar_msg_02,*ptrCloud_2);
    pcl::fromROSMsg(*Lidar_msg_03,*ptrCloud_3);
    pcl::fromROSMsg(*Lidar_msg_04,*ptrCloud_4);
    pcl::fromROSMsg(*Lidar_msg_05,*ptrCloud_5);

    pcl::transformPointCloud (*ptrCloud_1, *ptrCloud_1, Cal_Matrix_01);
    pcl::transformPointCloud (*ptrCloud_2, *ptrCloud_2, Cal_Matrix_02);
    pcl::transformPointCloud (*ptrCloud_3, *ptrCloud_3, Cal_Matrix_03);
    pcl::transformPointCloud (*ptrCloud_5, *ptrCloud_5, Cal_Matrix_05);

    boost::format dst_file_04(lidarDir04 + "%06d.bin");
    dst_file_04 % (Camera_msg->header.seq - firstSeq);

    write_Lidar(ptrCloud_4,dst_file_04.str());
    //ROS_INFO_STREAM("Before appending ptcloud: Lidar_3 size: "<<ptrCloud_3->size()<<" Lidar_4 size: "<<ptrCloud_4->size()<<" Lidar_5 size: "<<ptrCloud_5->size());
    *ptrCloud_4 += *ptrCloud_3;
    *ptrCloud_4 += *ptrCloud_5;
    *ptrCloud_4 += *ptrCloud_1;
    *ptrCloud_4 += *ptrCloud_2;
    //ROS_INFO_STREAM("After appending ptcloud: Total size: "<<ptrCloud_4->size());
    // ptrCloud_4 become the main pointcloud

    // Try to correct z axis
    pcl::transformPointCloud (*ptrCloud_4, *ptrCloud_4, _CL_1);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr=cv_bridge::toCvCopy(Camera_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
        return;
    }

    cv::Mat Image = cv_ptr->image;
    //Convert image into .jpg file
    boost::format fmt(imageDir + "%06d.jpg");
    fmt % (Camera_msg->header.seq - firstSeq);
    cv::imwrite(fmt.str(), Image);
    // Convert ptrCloud_4 into .bin and save

    // Eigen::MatrixXf _temp = ptrCloud_4->getMatrixXfMap(8,8,0);
    // //Eigen::MatrixXf _temp = ptrCloud_4->getMatrixXfMap();
    // Eigen::MatrixXf temp(4,_temp.cols());
    // temp<<_temp.row(0),_temp.row(1),_temp.row(2),(_temp.row(4)/255.0);
    // int nrow = temp.rows();
    // int ncol = temp.cols();
    // ROS_INFO_STREAM("Output Matrix size: "<< temp.size()<<" row: "<< nrow<< " col: "<<ncol);
    //ROS_INFO_STREAM(temp.col(0)); //(x,y,z,intensity)
    //ROS_INFO_STREAM(temp.row(3));
    //ROS_INFO_STREAM(_temp.col(0)); // (x,y,z,0,intensity,nonsense,nonsense)
    //ROS_INFO_STREAM(" PointXYZI size: "<<sizeof(pcl::PointXYZI));
    //ROS_INFO_STREAM((*ptrCloud_4)[0].x<<" "<<(*ptrCloud_4)[0].y<<" "<<(*ptrCloud_4)[0].z<<" "<<(*ptrCloud_4)[0].intensity);
    boost::format dst_file(lidarDir + "%06d.bin");
    dst_file % (Camera_msg->header.seq - firstSeq);
    write_Lidar(ptrCloud_4,dst_file.str());

 //    float data[nrow][ncol];
	// Eigen::Map<Eigen::MatrixXf>(&data[0][0], nrow, ncol) = temp;

	// FILE * stream;
	// stream = fopen(dst_file.str().c_str(),"wb");
	// fwrite(data,sizeof(float),4*temp.cols(),stream);
	// fclose(stream);

	sensor_msgs::PointCloud2 output;
	pcl::toROSMsg(*ptrCloud_4, output);
	pub_velo.publish(output);

    ROS_INFO_STREAM("Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms, Frames: "<<((Camera_msg->header.seq) - firstSeq+1));
};

int main (int argc, char** argv)
{
  // Initialize ROS
    ros::init (argc, argv, "navie_sync");
    ros::NodeHandle nh;

    // Create ROS subscribers for the input point cloud
    message_filters::Subscriber<sensor_msgs::Image> Camera(nh, "pg_15307894/image_raw", 5);
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_1(nh, "ns1/velodyne_points", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_2(nh, "ns2/velodyne_points", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_3(nh, "ns3/velodyne_points", 10);  
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_4(nh, "ns4/velodyne_points", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> Lidar_5(nh, "ns5/velodyne_points", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2,
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), Camera, Lidar_1, Lidar_2, Lidar_3, Lidar_4, Lidar_5);
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), Lidar_3, Lidar_4, Lidar_5);
    sync.registerCallback(boost::bind(&callback_sync, _1, _2, _3, _4, _5, _6));
    // sync.registerCallback(boost::bind(&callback_sync, _1, _2, _3));

    pub_velo = nh.advertise<sensor_msgs::PointCloud2>("/vlp_fuse", 1);

    ros::spin();
    // Spin
}