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
pcl::PointCloud<pcl::Normal>::Ptr compute_normal(const pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &cloud)
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

void process_cb(const sensor_msgs::PointCloud2 &input, pcl::visualization::PCLVisualizer &viewer)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    // do stuff with temp_cloud here
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normals = compute_normal(temp_cloud);
    ROS_INFO("End of callback");
    // visu(temp_cloud, normals);
}

int main(int argc, char **argv)
{
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    Cp_callback cloud_point;
    ros::init(argc, argv, "normal_cp_sub");
    ros::NodeHandle node_normal;
    // Velodyne VLP-16 produce 300 000 points/sec into the topic /points
    ros::Subscriber sub;
    sub = node_normal.subscribe("points", 5, cloud_cb);
    ros::spin();
    return 0;
}