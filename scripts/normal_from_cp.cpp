// Tutorial from : http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>

void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    ROS_INFO("inside callback");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normal_cp_sub");
    ros::NodeHandle node_normal;
    // Velodyne VLP-16 produce 300 000 points/sec into the topic /points
    ros::Rate loop_rate(10);
    ros::Subscriber sub;
    sub = node_normal.subscribe("points", 1, callback);
    ros::spin();
}