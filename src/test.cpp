#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
using namespace message_filters;


class Node
{

public:

    Node(int i)
{
    tmp = i;
    sub1.subscribe(node, "/pg_15307894/image_raw", 5);
    sub2.subscribe(node, "/ns4/velodyne_points", 10);
    sync.reset(new Sync(MySyncPolicy(10), sub1, sub2));   
    sync->registerCallback(boost::bind(&Node::callback, this, _1, _2));


}

    void callback(const sensor_msgs::ImageConstPtr& Camera_msg, const sensor_msgs::PointCloud2ConstPtr& Lidar_msg_04)
{
         std::cout << "Synchronization successful" << std::endl;
}  

private:
    int tmp;
    ros::NodeHandle node;
    // ros::Publisher pub;

    message_filters::Subscriber<sensor_msgs::Image> sub1;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub2;
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "synchronizer");

    Node synchronizer(1);

    ros::spin();

}