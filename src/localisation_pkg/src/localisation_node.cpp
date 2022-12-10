#include <ros/ros.h>
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
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
#include "localisation_pkg/trianglesList.h"
#include "localisationfunctions.h"
#include "math.h"
#include "geometry_msgs/Polygon.h"

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

        reflectorPosesSub = node.subscribe("/gazebo/model_states", 1, &LocalisationNode::ModelStatesCallback, this);

        dataPclPub = node.advertise<sensor_msgs::PointCloud>("/pclFiltered", 1);

        clusterPointsPub = node.advertise<localisation_pkg::test_pcl>("/clusteredPoints",1);

        pointsPub = node.advertise<localisation_pkg::test_pcl>("/justPoints",1);

        trianglesPub = node.advertise<localisation_pkg::trianglesList>("/trianglesList",1);

        mapReflectorPointsPub = node.advertise<localisation_pkg::test_pcl>("/mapReflectorPointsPub",1);

        mapTrianglesPub = node.advertise<localisation_pkg::trianglesList>("/mapTriangles",1);


    }

    void step()
    {

        dataPclPub.publish(dataPcl);

        clusterPointsPub.publish(msgClusteredPoints);

        pointsPub.publish(msgPoints);

        trianglesPub.publish(msgTriangles);

        mapReflectorPointsPub.publish(msgMapReflectorPointsList);

        mapTrianglesPub.publish(msgMapTriangles);

    }

private:
    ros::NodeHandle node { "~" }; /**< The ROS node handle. */
    const float samplingTime = 0.0F;

    ros::Subscriber velodynePointsSub;
    ros::Subscriber reflectorPosesSub;

    ros::Publisher dataPclPub;
    ros::Publisher clusterPointsPub;
    ros::Publisher pointsPub;
    ros::Publisher trianglesPub;
    ros::Publisher mapTrianglesPub;
    ros::Publisher mapReflectorPointsPub;

    sensor_msgs::PointCloud dataPcl;
    sensor_msgs::PointCloud2 dataPcl2;
    std::vector<geometry_msgs::Point32> points;
    std::vector<geometry_msgs::Point32> clusteredPoints;
    localisation_pkg::test_pcl msgClusteredPoints;
    localisation_pkg::test_pcl msgPoints;
    localisation_pkg::trianglesList msgTriangles;
    localisationType localisation;
    std::vector<geometry_msgs::Polygon> trianglesList;
    std::vector<geometry_msgs::Polygon> mapTrianglesList;
    localisation_pkg::trianglesList msgMapTriangles;
    std::vector<geometry_msgs::Point32> mapReflectorPointsList;
    localisation_pkg::test_pcl msgMapReflectorPointsList;



    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        dataPcl2 = localisation.filterPointCloud(*msg);
        sensor_msgs::convertPointCloud2ToPointCloud(dataPcl2, dataPcl);
        points = localisation.convertPcl2toVector(dataPcl2);
        clusteredPoints = localisation.clusterPointCloud(points);
        trianglesList = localisation.findTriangles(clusteredPoints);
        msgClusteredPoints.points = clusteredPoints;
        msgPoints.points = points;
        msgTriangles.triangles = trianglesList;
    }

    void ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {

      mapReflectorPointsList = localisation.getReflectorPositions(*msg);
      msgMapReflectorPointsList.points = mapReflectorPointsList;

      mapTrianglesList = localisation.findTriangles(mapReflectorPointsList);
      msgMapTriangles.triangles = mapTrianglesList;

      //reflectorPosesSub.shutdown();

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
