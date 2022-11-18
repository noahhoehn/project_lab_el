#ifndef TEST_POINTCLOUD_H
#define TEST_POINTCLOUD_H

#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
//#include "msg/test_pcl.h"
#include <sensor_msgs/PointCloud.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <vector>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "math.h"

tf::TransformListener *listener;

std::vector <sensor_msgs::PointCloud> velodyne;
//std::vector <float> intensity;

//localisation_pkg::test_msg TestMsg;

std::vector <geometry_msgs::Point32>transform_points(std::vector <geometry_msgs::Point32> input,std::string koordinatensystem);

#endif // TEST_POINTCLOUD_H
