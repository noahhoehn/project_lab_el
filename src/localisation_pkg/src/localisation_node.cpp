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
#include "localisation_pkg/pointList.h"
#include "localisation_pkg/reflector.h"
#include "localisation_pkg/reflectorList.h"
#include "localisation_pkg/triangle.h"
#include "localisation_pkg/trianglesList.h"
#include "localisationfunctions.h"
#include "math.h"
#include "geometry_msgs/Polygon.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "localisation_pkg/trianglePair.h"
#include "localisation_pkg/trianglePairList.h"
#include "localisation_pkg/triangleSide.h"
#include "localisation_pkg/triangleSideList.h"
#include "localisation_pkg/calcTriangle.h"
#include "localisation_pkg/calcTriangleList.h"
#include "localisation_pkg/reflectorPair.h"
#include "localisation_pkg/calcPosition.h"

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
        clusterCentroidsPub = node.advertise<localisation_pkg::pointList>("/clusterCentroids",1);
        filteredPointsPub = node.advertise<localisation_pkg::pointList>("/filteredPoints",1);
        mapReflectorListPub = node.advertise<localisation_pkg::reflectorList>("/mapReflectorList",1);
        mapTrianglesPub = node.advertise<localisation_pkg::trianglesList>("/mapTriangles",1);
        lidarTrianglesPub = node.advertise<localisation_pkg::trianglesList>("/lidarTriangles",1);
        lidarUsableTrianglesPub = node.advertise<localisation_pkg::trianglesList>("/lidarUsableTriangles",1);
        trianglePairsPub = node.advertise<localisation_pkg::trianglePairList>("/trianglePairs",1);
        calcTrianglesPub = node.advertise<localisation_pkg::calcTriangleList>("/calcTriangles",1);
        calcPosTrianglesPub = node.advertise<localisation_pkg::calcTriangleList>("/calcPosTriangles",1);
        calcPositionPub = node.advertise<localisation_pkg::calcPosition>("/calcPosition",1);
    }

    void step()
    {
        dataPclPub.publish(dataPcl);
        clusterCentroidsPub.publish(clusterCentroids);
        filteredPointsPub.publish(filteredPoints);
        mapReflectorListPub.publish(mapReflectorList);
        mapTrianglesPub.publish(mapTriangles);
        lidarTrianglesPub.publish(lidarTriangles);
        lidarUsableTrianglesPub.publish(lidarUsableTriangles);
        trianglePairsPub.publish(trianglePairs);
        calcTrianglesPub.publish(calcTriangles);
        calcPosTrianglesPub.publish(calcPosTriangles);
        calcPositionPub.publish(calcPosition);
    }

private:

    localisationType localisation;

    ros::NodeHandle node { "~" }; /**< The ROS node handle. */
    const float samplingTime = 0.0F;

    ros::Subscriber velodynePointsSub;
    ros::Subscriber reflectorPosesSub;

    ros::Publisher dataPclPub;
    ros::Publisher clusterCentroidsPub;
    ros::Publisher filteredPointsPub;
    ros::Publisher lidarTrianglesPub;
    ros::Publisher mapTrianglesPub;
    ros::Publisher mapReflectorListPub;
    ros::Publisher lidarUsableTrianglesPub;
    ros::Publisher trianglePairsPub;
    ros::Publisher calcTrianglesPub;
    ros::Publisher calcPosTrianglesPub;
    ros::Publisher calcPositionPub;

    bool gotMapTriangles = false;
    sensor_msgs::PointCloud dataPcl;
    sensor_msgs::PointCloud2 dataPcl2;
    geometry_msgs::Point32 gazeboLidarPos;
    localisation_pkg::pointList filteredPoints;
    localisation_pkg::pointList clusterCentroids;
    localisation_pkg::reflectorList clusterCentroidsLabeled;
    localisation_pkg::trianglesList lidarTriangles;
    localisation_pkg::trianglesList lidarUsableTriangles;
    localisation_pkg::trianglesList mapTriangles;
    localisation_pkg::reflectorList mapReflectorList;
    localisation_pkg::trianglePairList trianglePairs;
    localisation_pkg::calcTriangleList calcTriangles;
    localisation_pkg::calcTriangleList calcPosTriangles;
    localisation_pkg::calcPosition calcPosition;


    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        dataPcl2 = localisation.filterPointCloud(*msg);
        sensor_msgs::convertPointCloud2ToPointCloud(dataPcl2, dataPcl);
        filteredPoints = localisation.convertPcl2toVector(dataPcl2);
        clusterCentroids = localisation.clusterPointCloud(filteredPoints);
        clusterCentroidsLabeled = localisation.labelClusterCentroids(clusterCentroids);
        lidarTriangles = localisation.findTriangles(clusterCentroidsLabeled);
        lidarUsableTriangles = localisation.findUsableTriangles(lidarTriangles);
        trianglePairs = localisation.findTrianglePairs(mapTriangles, lidarUsableTriangles);
        calcTriangles = localisation.getCalcTriangelsList(trianglePairs);
        calcPosTriangles = localisation.getPosTriangelsList(calcTriangles);
        calcPosition = localisation.getMeanPosition(calcPosTriangles, gazeboLidarPos);
    }

    void ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
      gazeboLidarPos = localisation.getGazeboLidarPos(*msg);

      if (!gotMapTriangles)
      {
          mapReflectorList = localisation.getReflectorPositions(*msg);
          mapTriangles = localisation.findTriangles(mapReflectorList);
          gotMapTriangles = true;
      }

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
