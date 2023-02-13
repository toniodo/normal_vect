#ifndef NORMAL_LAYER_H_
#define NORMAL_LAYER_H_
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace normal_layer_namespace
{

  class NormalLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
  {
  public:
    NormalLayer();

    void onInitialize();

    // To set up the area of the layer in which costs will be updated
    void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x,
                      double *max_y);

    // Update costs according to the angle of the normals
    void updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

    bool isDiscretized()
    {
      return true;
    }

    void matchSize();

    // Callback to get normal from Lidar
    void normalCallback(const sensor_msgs::PointCloud2::ConstPtr &input);

    void imuCallback(const sensor_msgs::Imu::ConstPtr &input);

    pcl::PointCloud<pcl::PointNormal>::Ptr computeNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    ros::Subscriber normal_sub;

    ros::Subscriber imu_sub;

  private:
    pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_normals;

    geometry_msgs::Vector3 orientation_imu;

    ros::NodeHandle nh;

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  };
}
#endif
