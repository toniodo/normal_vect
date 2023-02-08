#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <normal_vect/pub_sub_cp.h>
#include <normal_vect/normal_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(normal_layer_namespace::NormalLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace normal_layer_namespace
{

    NormalLayer::NormalLayer()
    {
        cloud_normals = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    }

    void NormalLayer::onInitialize()
    {
        current_ = true;
        default_value_ = NO_INFORMATION;
        matchSize();

        normal_sub = nh.subscribe("normal_cp", 5, &NormalLayer::normalCallback, this);

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &NormalLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    // TODO create call that update cloud_normals
    void NormalLayer::normalCallback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &input)
    {
        cloud_normals = input;
    }

    void NormalLayer::matchSize()
    {
        Costmap2D *master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
    }

    void NormalLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    void NormalLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                   double *min_y, double *max_x, double *max_y)
    {
        if (!enabled_)
            return;

        double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
        unsigned int mx;
        unsigned int my;
        if (worldToMap(mark_x, mark_y, mx, my))
        {
            setCost(mx, my, LETHAL_OBSTACLE);
        }

        *min_x = std::min(*min_x, mark_x);
        *min_y = std::min(*min_y, mark_y);
        *max_x = std::max(*max_x, mark_x);
        *max_y = std::max(*max_y, mark_y);
    }

    void NormalLayer::updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
            return;
        float maxAngleDeg = 30;
        float thresh = std::cos(maxAngleDeg * M_PI / 180.0);
        unsigned int mx;
        unsigned int my;
        Eigen::Vector3f dir(0.0, 0.0, 1.0);
        pcl::PointIndicesPtr indices = pcl::PointIndicesPtr(new pcl::PointIndices);
        for (auto i = 0; i < cloud_normals->size(); ++i)
        {
            const auto &normal = cloud_normals->points[i].getNormalVector3fMap();
            // To correct errors in direction we use absolute value
            if (abs(normal.dot(dir)) <= thresh)
            {
                master_grid.worldToMap(cloud_normals->points[i].x, cloud_normals->points[i].y, mx, my);
                if (mx >= min_i && mx <= max_i && my >= min_j && my <= max_j)
                {
                    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
                }
            }
        }
    }
} // end namespace

/*
template <>
void PublisherSubscriber<costmap_2d::Costmap2D, pcl::PointCloud<pcl::PointNormal>>::subscriberCallback(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &input)
{
    // Set a threshold angle
    float maxAngleDeg = 30;
    // Normal to the horizon
    Eigen::Vector3d dir(0.0, 0.0, 1.0);
    // do stuff with temp_cloud here
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_layer_namespace::NormalLayer new_layer;
    new_layer.onInitialize();

    publisherObject.publish(new_layer);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "costmap_threshold");
    // Velodyne VLP-16 produce 300 000 points/sec into the topic /points
    PublisherSubscriber<costmap_2d::Costmap2D, pcl::PointCloud<pcl::PointNormal>> pub_sub("costmap_normal", "normal_of_cp", 5);

    ros::spin();
    return 0;
}*/