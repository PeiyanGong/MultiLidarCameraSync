#include <ros/ros.h>
#include "LidarCameraSync.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "synchronizer");

  int lidar_nums, lidar_hz;
  bool correct_distortion;
  ros::param::get("/naive_sync/num_of_lidar", lidar_nums);
  ros::param::get("/naive_sync/lidar_hz", lidar_hz);
  ros::param::get("/naive_sync/correct_distortion", correct_distortion);
  // std::cout << lidar_nums <<std::endl;
  // try{
  LidarCameraSync synchronizer(lidar_nums, lidar_hz, correct_distortion);
  // } 
  // catch(const char* error)
  // {
  // 	std::cerr<<"Error: "<<error<<std::endl;
  // }
  

  ros::spin();

}
