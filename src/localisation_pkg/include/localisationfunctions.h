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

    sensor_msgs::PointCloud2 filterPointcloud(const sensor_msgs::PointCloud2 cloud_msg)
    {
        // Create pointer on new PCL PointCloud2
        pcl::PCLPointCloud2 *cloud2 = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud2);
        //Create new PCL Pointcloud2
        pcl::PCLPointCloud2 cloud_filtered;
        //Write Cloud Message (sensor_msgs/PointCloud2) to cloud2
        pcl_conversions::toPCL(cloud_msg, *cloud2);

        //Create Passthrough filter for PCL PointCloud2
        pcl::PassThrough<pcl::PCLPointCloud2> filter;

        //Set Filter Parameters (Input Cloud, Filter Name, Upper and lower filter limit)
        filter.setInputCloud(cloudPtr);
        filter.setFilterFieldName("intensity");
        filter.setFilterLimits(150.0, 250.0);
        // Apply filter -> cloud_filtered is the filtered PCL PointCloud2
        filter.filter(cloud_filtered);

        // Create sensor_msgs/PointCloud2
        sensor_msgs::PointCloud2 cloudy_filt;
        // Convert PCL PointCloud2 to sensor_msgs/PointCloud2
        pcl_conversions::moveFromPCL(cloud_filtered, cloudy_filt);

        //Return filtered sensor_msgs/PointCloud2
        return cloudy_filt;
    }

private:
};


#endif // LOCALISATIONFUNCTIONS_H
