#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <normal_vect/cp_callback.hpp>

// Estimate normals
pcl::PointCloud<pcl::Normal>::Ptr compute_normal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // Use 50 nearest neighboor
    ne.setKSearch(50);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    return normals;
}

// Visualize normals
void visu(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::visualization::PCLVisualizer &viewer)
{
    float r, g = 255, b;
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, r, g, b);
    viewer.updatePointCloud(cloud, color, "cloud2");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 100, 0.4f);
}

void process_cb(const sensor_msgs::PointCloud2 &input, pcl::visualization::PCLVisualizer &viewer)
{
    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::fromROSMsg(input, temp_cloud);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_ptr=boost::make_shared(temp_cloud);
    // do stuff with temp_cloud here
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals = compute_normal(temp_cloud);
    visu(temp_cloud, normals, viewer);
}

void Cp_callback::cloud_cb(const sensor_msgs::PointCloud2 &input)
{
    data = input;
}

int main(int argc, char **argv)
{
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    Cp_callback cloud_point;
    ros::init(argc, argv, "normal_cp_sub");
    while (ros::ok())
    {
        ros::NodeHandle node_normal;
        // Velodyne VLP-16 produce 300 000 points/sec into the topic /points
        ros::Rate loop_rate(1);
        ros::Subscriber sub;
        sub = node_normal.subscribe("points", 100, &Cp_callback::cloud_cb, &cloud_point);
        process_cb(cloud_point.data, viewer);
        loop_rate.sleep();
    }
    return 0;
}