#include <ros/ros.h>
#include <stdio.h>
#include "localisationfunctions.h"
#include "parameters.h"

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
      //Subscriptions
      velodynePointsSub = node.subscribe("/velodyne_points", 1, &LocalisationNode::LidarCallback, this);
      reflectorPosesSub = node.subscribe("/gazebo/model_states", 1, &LocalisationNode::ModelStatesCallback, this);

      //Advertisements
      dataPclPub = node.advertise<sensor_msgs::PointCloud>("/pclFiltered", 1);
      filteredPointsPub = node.advertise<localisation_pkg::pointList>("/filteredPoints",1);
      clusterCentroidsPub = node.advertise<localisation_pkg::pointList>("/clusterCentroids",1);
      mapReflectorListPub = node.advertise<localisation_pkg::reflectorList>("/mapReflectorList",1);
      mapTrianglesPub = node.advertise<localisation_pkg::trianglesList>("/mapTriangles",1);
      mapCompareTrianglesPub = node.advertise<localisation_pkg::trianglesList>("/mapCompareTriangles",1);
      lidarTrianglesPub = node.advertise<localisation_pkg::trianglesList>("/lidarTriangles",1);
      lidarUsableTrianglesPub = node.advertise<localisation_pkg::trianglesList>("/lidarUsableTriangles",1);
      trianglePairsPub = node.advertise<localisation_pkg::trianglePairList>("/trianglePairs",1);
      calcTrianglesPub = node.advertise<localisation_pkg::calcTriangleList>("/calcTriangles",1);
      calcPosTrianglesPub = node.advertise<localisation_pkg::calcTriangleList>("/calcPosTriangles",1);
      calcPositionPub = node.advertise<localisation_pkg::calcPosition>("/calcPosition",1);
      gazeboLidarPosPub = node.advertise<geometry_msgs::Point32>("/gazeboLidarPos",1);
    }

    void step()
    {
      //Localisation
      if (dataAvailable == true)//only do calculation if data is available (otherwise error in filter function, especially needed for first calculation)
      {
        localisationFinished = false; //block lidar-callback function from overwriting data
        dataPcl2Filtered = localisation.filterPointCloud(dataPcl2); //filter point cloud
        sensor_msgs::convertPointCloud2ToPointCloud(dataPcl2Filtered, dataPcl); //convertion from PCL2 to PCL
        filteredPoints = localisation.convertPcl2toVector(dataPcl2Filtered);  //convertion from PCL to Vector
        clusterCentroids = localisation.clusterPointCloud(filteredPoints, minPointsForCluster, maxDistanceClusterPoints); //cluster Vector of Points
        lidarTriangles = localisation.findTriangles(clusterCentroids);  //build triangles between cluster centroids/LiDAR reflectors
        lidarUsableTriangles = localisation.findUsableTriangles(lidarTriangles);  //filter triangles on usability (LiDAR in Triangle)
        mapCompareTriangles = localisation.findMapCompareTriangles (mapTriangles, gazeboLidarPos, gnssAreaRadiusThreshold); //filter map triangles whether in area of LiDAR
        trianglePairs = localisation.findTrianglePairs(mapCompareTriangles, lidarUsableTriangles, triangleMatchingTolerance); //match triangles
        calcTriangles = localisation.getCalcTriangelsList(trianglePairs); //match Reflectors of triangle pairs
        calcPosTriangles = localisation.getPosTriangelsList(calcTriangles); //calculate location of lidar for each triangle pair
        calcPosition = localisation.getMeanPosition(calcPosTriangles, gazeboLidarPos);  //calculate median of positions
        localisationFinished = true;  //unblock lidar-callback function from overwriting data
      }

      //Publish Debug Data
      dataPclPub.publish(dataPcl);
      clusterCentroidsPub.publish(clusterCentroids);
      filteredPointsPub.publish(filteredPoints);
      mapReflectorListPub.publish(mapReflectorList);
      mapTrianglesPub.publish(mapTriangles);
      mapCompareTrianglesPub.publish(mapCompareTriangles);
      lidarTrianglesPub.publish(lidarTriangles);
      lidarUsableTrianglesPub.publish(lidarUsableTriangles);
      trianglePairsPub.publish(trianglePairs);
      calcTrianglesPub.publish(calcTriangles);
      calcPosTrianglesPub.publish(calcPosTriangles);

      //Publish calculated Position and actual Position
      calcPositionPub.publish(calcPosition);
      gazeboLidarPosPub.publish(gazeboLidarPos);
    }

private:

    //class Type
    localisationType localisation;

    //decleration node and sampling time
    ros::NodeHandle node { "~" }; /**< The ROS node handle. */
    const float samplingTime = 0.0F;

    //declaration Subscriber and Publisher
    ros::Subscriber velodynePointsSub;
    ros::Subscriber reflectorPosesSub;
    ros::Publisher dataPclPub;
    ros::Publisher clusterCentroidsPub;
    ros::Publisher filteredPointsPub;
    ros::Publisher lidarTrianglesPub;
    ros::Publisher mapTrianglesPub;
    ros::Publisher mapCompareTrianglesPub;
    ros::Publisher mapReflectorListPub;
    ros::Publisher lidarUsableTrianglesPub;
    ros::Publisher trianglePairsPub;
    ros::Publisher calcTrianglesPub;
    ros::Publisher calcPosTrianglesPub;
    ros::Publisher calcPositionPub;
    ros::Publisher gazeboLidarPosPub;

    //declaration query bools
    bool gotMapTriangles = false;
    bool localisationFinished = true;
    bool dataAvailable = false;

    //declaration localisation variables
    sensor_msgs::PointCloud dataPcl;
    sensor_msgs::PointCloud2 dataPcl2;
    sensor_msgs::PointCloud2 dataPcl2Filtered;
    geometry_msgs::Point32 gazeboLidarPos;
    localisation_pkg::pointList filteredPoints;
    localisation_pkg::reflectorList clusterCentroids;
    localisation_pkg::trianglesList lidarTriangles;
    localisation_pkg::trianglesList lidarUsableTriangles;
    localisation_pkg::trianglesList mapTriangles;
    localisation_pkg::trianglesList mapCompareTriangles;
    localisation_pkg::reflectorList mapReflectorList;
    localisation_pkg::trianglePairList trianglePairs;
    localisation_pkg::calcTriangleList calcTriangles;
    localisation_pkg::calcTriangleList calcPosTriangles;
    localisation_pkg::calcPosition calcPosition;


    void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
      //if localisation algorithm still running, don't overwrite data
      if (localisationFinished == true)
      {
        dataPcl2 = *msg;
        dataAvailable = true;
      }
    }

    void ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
      gazeboLidarPos = localisation.getGazeboLidarPos(*msg);

      //just check once for map triangles
      if (!gotMapTriangles)
      {
          mapReflectorList = localisation.getReflectorPositions(*msg);
          mapTriangles = localisation.findTriangles(mapReflectorList);
          mapTriangles = localisation.filterMapTriangles(mapTriangles, maxTriangleSideLength);
          gotMapTriangles = true;
      }
    }

};

int main(int argc, char **argv)
{
    const float samplingTime = 10e-3F; // the constant sample time [ s ]
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
