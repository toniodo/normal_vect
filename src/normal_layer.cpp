#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h> // Use to accelerate performances
// #include <pcl/filters/statistical_outlier_removal.h> // Filter to remove outliers
#include <normal_vect/normal_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(normal_layer_namespace::NormalLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace normal_layer_namespace
{

    NormalLayer::NormalLayer()
    {
        // Init the Cloud point of normals
        cloud_normals = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    }

    void NormalLayer::onInitialize()
    {
        current_ = true;
        default_value_ = NO_INFORMATION;
        matchSize();

        std::string topic_lidar;
        nh.getParam("topic_lidar", topic_lidar);
        std::string topic_imu;
        nh.getParam("topic_imu", topic_imu);

        // Set subscribers
        normal_sub = nh.subscribe(topic_lidar, 5, &NormalLayer::normalCallback, this);
        imu_sub = nh.subscribe(topic_imu, 5, &NormalLayer::imuCallback, this);

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &NormalLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void NormalLayer::normalCallback(const sensor_msgs::PointCloud2::ConstPtr &input)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        // // Create the filtering object from the converted cloud
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // int nb_neighboor;
        // nh.getParam("nb_neighboor", nb_neighboor);
        // sor.setInputCloud(temp_cloud);
        // sor.setMeanK(nb_neighboor);
        // sor.setStddevMulThresh(0.25);
        // sor.filter(*cloud_filtered);

        // // Compute normals from the filtered cloud point
        // cloud_normals = computeNormal(cloud_filtered);

        // Compute normals from the converted cloud
        cloud_normals = computeNormal(temp_cloud);
    }

    void NormalLayer::imuCallback(const sensor_msgs::Imu::ConstPtr &input)
    {
        orientation_imu = input->orientation;
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

        // Get parameters from launch file
        double height_local;
        double width_local;
        nh.getParam("height_local", height_local);
        nh.getParam("width_local", width_local);

        // Set bounds according to the size of the local costmap
        *min_x = mark_x - height_local / 2;
        *min_y = mark_y - width_local / 2;
        *max_x = mark_x + height_local / 2;
        *max_y = mark_y + width_local / 2;
    }

    void NormalLayer::updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
            return;

        float maxAngleDeg;
        nh.getParam("angle_threshold", maxAngleDeg);
        float thresh = std::cos(maxAngleDeg * M_PI / 180.0);
        unsigned int mx;
        unsigned int my;

        // The direction of normal of the base according to the frame reference of the IMU

        Eigen::Vector3f dir(orientation_imu.x, orientation_imu.y, orientation_imu.z);
        dir.normalize();
        for (auto i = 0; i < cloud_normals->size(); ++i)
        {
            const auto &normal = cloud_normals->points[i].getNormalVector3fMap();
            /*
            To correct errors in direction we use absolute value and check the condition according to the maximum angle.
            We also check if the point fits to the map of master_grid and to the updated bounds.
            */
            if (abs(normal.dot(dir)) <= thresh && master_grid.worldToMap(cloud_normals->points[i].x, cloud_normals->points[i].y, mx, my) && mx >= min_i && mx <= max_i && my >= min_j && my <= max_j)
            {
                master_grid.setCost(mx, my, LETHAL_OBSTACLE);
            }
        }
    }

    // Estimate normals
    pcl::PointCloud<pcl::PointNormal>::Ptr NormalLayer::computeNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        // Use of a Kd tree method to find the nearest neighboors
        ne.setSearchMethod(tree);

        int nb_neighboor;
        nh.getParam("nb_neighboor", nb_neighboor);
        int nb_thread;
        nh.getParam("nb_thread", nb_thread);
        ne.setNumberOfThreads(nb_thread);
        ne.setKSearch(nb_neighboor);

        ne.setInputCloud(cloud);
        ne.compute(*normals);
        pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
        return cloud_with_normals;
    }

} // end namespace
