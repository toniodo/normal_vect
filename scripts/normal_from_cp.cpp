#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/foreach.hpp>
#include <normal_vect/pub_sub_cp.h>

// Estimate normals
pcl::PointCloud<pcl::PointNormal>::Ptr compute_normal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    // Use 50 nearest neighboor
    ne.setKSearch(50);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    return cloud_with_normals;
}

// Visualize normals
void visu(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    float r, g = 255, b;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 100, 0.4f);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, r, g, b);
    viewer.addPointCloud(cloud, color, "cloud2");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

template <>
void PublisherSubscriber<pcl::PointCloud<pcl::PointNormal>, sensor_msgs::PointCloud2>::subscriberCallback(const sensor_msgs::PointCloud2::ConstPtr &input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    // do stuff with temp_cloud here
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals = compute_normal(temp_cloud);
    // visu(temp_cloud, normals);
    publisherObject.publish(*cloud_normals);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "normal_cp_sub");
    // Velodyne VLP-16 produce 300 000 points/sec into the topic /points
    PublisherSubscriber<pcl::PointCloud<pcl::PointNormal>, sensor_msgs::PointCloud2> pub_sub("normal_cp", "points", 5);
    ros::spin();
    return 0;
}