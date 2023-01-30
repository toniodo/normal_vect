// Tutorial from : http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>

void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    // do stuff with temp_cloud here
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normal_cp_sub");
    ros::NodeHandle node_normal;
    // Velodyne VLP-16 produce 300 000 points/sec into the topic /points
    ros::Rate loop_rate(10);
    ros::Subscriber sub;
    sub = node_normal.subscribe("points", 1, cloud_cb);
    ros::spin();
}