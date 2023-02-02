#pragma once
#include <sensor_msgs/PointCloud2.h>

class Cp_callback
{
public:
    sensor_msgs::PointCloud2 data;
    void cloud_cb(const sensor_msgs::PointCloud2 &input);
};