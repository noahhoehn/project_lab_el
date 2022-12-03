#include <ros/ros.h>
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
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
#include "localisation_pkg/test_pcl.h"
#include "localisationfunctions.h"
#include "math.h"

/**
 * @brief The LocalisationNode class
 */
class LocalisationNode
{
public:
    using localisationType = LocalisationFunctions;
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    LocalisationNode(const float samplingTimeArg):
        samplingTime(samplingTimeArg)
    {
        velodynePointsSub = node.subscribe("/velodyne_points", 1, &LocalisationNode::LidarCallback, this);

        filtered2PointsPub = node.advertise<sensor_msgs::PointCloud2>("/filt_points", 1);

        filtered1PointsPub = node.advertise<sensor_msgs::PointCloud>("/filt_pointcloud1", 1);

        intensityPub = node.advertise<localisation_pkg::test_pcl>("/intensity", 1);
    }

    void step()
    {
        filtered2PointsPub.publish(pointcloud_2);
        filtered1PointsPub.publish(pointcloud_1);
    }

private:
    ros::NodeHandle node { "~" }; /**< The ROS node handle. */
    ros::Subscriber velodynePointsSub;
    ros::Publisher filtered2PointsPub;
    ros::Publisher filtered1PointsPub;
    ros::Publisher intensityPub;
    const float samplingTime;
    localisation_pkg::test_pcl intensityArray;
    sensor_msgs::PointCloud pointcloud_1;
    sensor_msgs::PointCloud2 pointcloud_2;
 //   PointCloud::Ptr pcl_1 (new PointCloud);
    localisationType localisation;


    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pointcloud_2 = localisation.filterPointcloud(*msg);
        sensor_msgs::convertPointCloud2ToPointCloud(pointcloud_2, pointcloud_1);
    }

};

int main(int argc, char **argv)
{
    const float samplingTime = 100e-3F; // the constant sample time [ s ]
    ros::init(argc, argv, "carctrl_node"); // initialize ROS
    LocalisationNode node(samplingTime);
    // define sampling rate as the inverse of the sample time
    ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));
    // loop while ROS is running
    while (ros::ok()) {
        // call the method step() of the SineNode instance node
        node.step();
        // pass control to ROS for background tasks
        ros::spinOnce();
        // wait for next sampling point
        // neighbor sampling points have a time distance of 100ms
        loopRate.sleep();
    }
    // return success
    return EXIT_SUCCESS;
}
