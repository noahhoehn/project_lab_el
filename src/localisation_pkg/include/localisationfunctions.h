#ifndef LOCALISATIONFUNCTIONS_H
#define LOCALISATIONFUNCTIONS_H

#pragma once

#include <ros/ros.h>
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <velodyne_pointcloud/point_types.h>
#include <vector>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "math.h"

class LocalisationFunctions
{
public:
    LocalisationFunctions(){}

    sensor_msgs::PointCloud2 filterPcl(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
        pcl::PCLPointCloud2 cloud_filtered;

        pcl_conversions::toPCL(*cloud_msg, *cloud2);

        for (int i=0; i<cloud2->)

        return temp_cloud;
    }

private:
};


#endif // LOCALISATIONFUNCTIONS_H
