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

        //filtered2PointsPub = node.advertise<sensor_msgs::PointCloud2>("/filt_points", 1);

        dataPclPub = node.advertise<sensor_msgs::PointCloud>("/pclFiltered", 1);

        //intensityPub = node.advertise<localisation_pkg::test_pcl>("/intensity", 1);

        clusterPointsPub = node.advertise<localisation_pkg::test_pcl>("/clusteredPoints",1);

        pointsPub = node.advertise<localisation_pkg::test_pcl>("/justPoints",1);


    }

    void step()
    {
        //filtered2PointsPub.publish(dataPcl2);
        dataPclPub.publish(dataPcl);

        clusterPointsPub.publish(msgClusteredPoints);

        pointsPub.publish(msgPoints);
    }

private:
    ros::NodeHandle node { "~" }; /**< The ROS node handle. */
    ros::Subscriber velodynePointsSub;
    //ros::Publisher filtered2PointsPub;
    ros::Publisher dataPclPub;
    //ros::Publisher intensityPub;
    ros::Publisher clusterPointsPub;
    ros::Publisher pointsPub;
    const float samplingTime = 0.0F;
    localisation_pkg::test_pcl intensityArray;
    sensor_msgs::PointCloud dataPcl;
    sensor_msgs::PointCloud2 dataPcl2;
    std::vector<geometry_msgs::Point32> points;
    std::vector<geometry_msgs::Point32> clusteredPoints;
    localisation_pkg::test_pcl msgClusteredPoints;
    localisation_pkg::test_pcl msgPoints;
    localisationType localisation;



    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        dataPcl2 = localisation.filterPointCloud(*msg);
        sensor_msgs::convertPointCloud2ToPointCloud(dataPcl2, dataPcl);
        points = localisation.convertPcl2toVector(dataPcl2);
        clusteredPoints = localisation.clusterPointCloud(points);
        msgClusteredPoints.points = clusteredPoints;
        msgPoints.points = points;
    }
};

int main(int argc, char **argv)
{
    const float samplingTime = 100e-3F; // the constant sample time [ s ]
    ros::init(argc, argv, "localisation_node"); // initialize ROS
    LocalisationNode node(samplingTime);
    // define sampling rate as the inverse of the sample time
    ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));
    // loop while ROS is running
    while (ros::ok()) {
        node.step();
        ros::spinOnce();
        loopRate.sleep();
    }
    // return success
    return EXIT_SUCCESS;
}
