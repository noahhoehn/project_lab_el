#ifndef LOCALISATIONFUNCTIONS_H
#define LOCALISATIONFUNCTIONS_H

#pragma once

//#include <ros/ros.h>
#include <stdio.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
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
#include <numeric>
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
#include "cmath"
#include <algorithm>
#include "utility"
//created messages
#include "localisation_pkg/pointList.h"
#include "localisation_pkg/reflector.h"
#include "localisation_pkg/reflectorList.h"
#include "localisation_pkg/triangle.h"
#include "localisation_pkg/trianglesList.h"
#include "localisation_pkg/trianglePair.h"
#include "localisation_pkg/trianglePairList.h"
#include "localisation_pkg/triangleSide.h"
#include "localisation_pkg/triangleSideList.h"
#include "localisation_pkg/calcTriangle.h"
#include "localisation_pkg/calcTriangleList.h"
#include "localisation_pkg/reflectorPair.h"
#include "localisation_pkg/calcPosition.h"

/**
 * @brief The LocalisationFunctions class
 */
class LocalisationFunctions
{
public:
    LocalisationFunctions(){}

    //-----------------------------filter and conversion--------------------------------//
    /**
     * @brief filterPointCloud filters pointCloud data by intensity
     * @param msgCloud2
     * @return msgCloud2Filtered
     */
    sensor_msgs::PointCloud2 filterPointCloud(const sensor_msgs::PointCloud2 msgCloud2)
    {
        // Create pointer on new PCL PointCloud2
        pcl::PCLPointCloud2 *pclCloud2 = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr pclCloud2Ptr(pclCloud2);
        //Create new PCL Pointcloud2
        pcl::PCLPointCloud2 pclCloud2Filtered;
        //Convert Cloud Message (sensor_msgs/PointCloud2) to PCLCloud2
        pcl_conversions::toPCL(msgCloud2, *pclCloud2);

        //Create Passthrough filter for PCL PointCloud2
        pcl::PassThrough<pcl::PCLPointCloud2> filter;

        //Set Filter Parameters (Input Cloud, Filter Name, Upper and lower filter limit)
        filter.setInputCloud(pclCloud2Ptr);
        filter.setFilterFieldName("intensity");
        filter.setFilterLimits(150.0, 255.0);
        // Apply filter -> cloud_filtered is the filtered PCL PointCloud2
        filter.filter(pclCloud2Filtered);

        // Create sensor_msgs/PointCloud2
        sensor_msgs::PointCloud2 msgCloud2Filtered;
        // Convert PCL PointCloud2 to sensor_msgs/PointCloud2
        pcl_conversions::moveFromPCL(pclCloud2Filtered, msgCloud2Filtered);

        //Return filtered sensor_msgs/PointCloud2
        return msgCloud2Filtered;
    }

    /**
     * @brief convertPcl2toVector converts pointCloud data to vector of 3D-Points
     * @param msgCloud2
     * @return pointList
     */
    localisation_pkg::pointList convertPcl2toVector(const sensor_msgs::PointCloud2 msgCloud2)
    {
      //create vector to store points with XYZ-values
      localisation_pkg::pointList pointList;
      //convert PointCloud2 to PointCloud to access points
      sensor_msgs::PointCloud msgCloud;
      sensor_msgs::convertPointCloud2ToPointCloud(msgCloud2,msgCloud);
      //write points from PointCloud data to vector
      pointList.points = msgCloud.points;
      return pointList;
    }


    //----------------------------------clustering-------------------------------------//
    /**
     * @brief clusterPointCloud clusters pointCloud data (as vector of 3D-points) and calculates centroids of clusters
     * @param inputPoints
     * @return clusterCentroids
     */
    localisation_pkg::reflectorList clusterPointCloud(const localisation_pkg::pointList inputPoints, unsigned int minPoints, float maxDistance)
    {
      std::set<unsigned int> ignoreIndices;
      localisation_pkg::pointList clusterCentroids;
      localisation_pkg::pointList tempNearPoints;

      for (unsigned int i = 0; i < inputPoints.points.size(); i++)
      {
        if (ignoreIndices.find(i) == ignoreIndices.end()) //ignore point if it's already in a cluster
        {
          for (unsigned int j = 0; j < inputPoints.points.size(); j++)
          {
            if (i != j && ignoreIndices.find(j) == ignoreIndices.end())
            {
              float distance = calcDistance(inputPoints.points.at(i), inputPoints.points.at(j));
              if (distance < maxDistance)
              {
                tempNearPoints.points.push_back(inputPoints.points.at(j)); //store point in cluster if it's near enough to another point
                ignoreIndices.insert(j);
              }
            }
          }

          if (tempNearPoints.points.size() >= (minPoints-1)) //check if enough points found for cluster
          {
            tempNearPoints.points.push_back(inputPoints.points.at(i));
            ignoreIndices.insert(i);
            clusterCentroids.points.push_back(calcCentroid(tempNearPoints)); //only cluster centroid stored in return list
          }
        }

        tempNearPoints.points.clear();
      }

      localisation_pkg::reflectorList labeledClusterCentroids;

      for (unsigned int i=0; i<clusterCentroids.points.size(); i++)
      {
        localisation_pkg::reflector tempReflector;
        tempReflector.position = clusterCentroids.points.at(i);
        tempReflector.label = i;

        labeledClusterCentroids.reflectors.push_back(tempReflector);
      }

      return labeledClusterCentroids;
    }

    /**
     * @brief calcDistance calculates distance between 2 3D-Points
     * @param pointA
     * @param pointB
     * @return distance
     */
    float calcDistance (geometry_msgs::Point32 pointA, geometry_msgs::Point32 pointB)
    {
      float distance = std::sqrt(std::pow(pointA.x - pointB.x,2) + std::pow(pointA.y - pointB.y,2) + std::pow(pointA.z - pointB.z,2));  //calculate distance with pythagoras theorem
      return distance;
    }

    /**
     * @brief calcCentroid calculates centroid of list of points
     * @param inputPoints
     * @return averagePoint
     */
    geometry_msgs::Point32 calcCentroid (localisation_pkg::pointList inputPoints)
    {
      std::vector<float> collectX;
      std::vector<float> collectY;
      std::vector<float> collectZ;
      float averageX = 0.0F;
      float averageY = 0.0F;
      float averageZ = 0.0F;

      for (unsigned int k = 0; k < inputPoints.points.size(); k++)  //collect all coordinates
      {
        collectX.push_back(inputPoints.points.at(k).x);
        collectY.push_back(inputPoints.points.at(k).y);
        collectZ.push_back(inputPoints.points.at(k).z);
      }

      auto const vectorSize = static_cast<float>(inputPoints.points.size());

      averageX = std::accumulate(collectX.begin(), collectX.end(), 0.0F)/vectorSize;  //calculate mean-value for each coordinate
      averageY = std::accumulate(collectY.begin(), collectY.end(), 0.0F)/vectorSize;
      averageZ = std::accumulate(collectZ.begin(), collectZ.end(), 0.0F)/vectorSize;

      geometry_msgs::Point32 averagePoint;

      averagePoint.x = averageX;
      averagePoint.y = averageY;
      averagePoint.z = averageZ;

      return averagePoint;
    }



    //-----------------------------find triangles (map and lidar)--------------------------------//
    /**
     * @brief findTriangles finds possible constellations of 3 points/reflectors that builds up a triangle
     * @param inputReflectors
     * @return triangleList
     */
    localisation_pkg::trianglesList findTriangles (localisation_pkg::reflectorList inputReflectors)
    {
      for (unsigned int i=0; i<inputReflectors.reflectors.size(); i++) //set every z-coordinate to zero to get 2D-problem
      {
        inputReflectors.reflectors.at(i).position.z = 0.0F;
      }

      localisation_pkg::trianglesList triangleList;

      for (unsigned int i=0; i<inputReflectors.reflectors.size(); i++) //3 for-loops to check every combination of 3 points from input reflectors
      {
        for (unsigned int j=i+1; j<inputReflectors.reflectors.size(); j++)
        {
          for (unsigned int k=j+1; k<inputReflectors.reflectors.size(); k++)
          {
            if(isTriangle(inputReflectors.reflectors.at(i).position, inputReflectors.reflectors.at(j).position, inputReflectors.reflectors.at(k).position))
            {
              localisation_pkg::triangle tempTriangle;
              tempTriangle.reflectors.push_back(inputReflectors.reflectors.at(i));
              tempTriangle.reflectors.push_back(inputReflectors.reflectors.at(j));
              tempTriangle.reflectors.push_back(inputReflectors.reflectors.at(k));
              tempTriangle.sideList = (getTriangleSides(tempTriangle));
              tempTriangle.usable = false;
              triangleList.triangles.push_back(tempTriangle);
            }
          }
        }
      }
      return triangleList;
    }

    /**
     * @brief isTriangle find out wether a triangle can be build from 3 2D-points
     * @param PointA
     * @param PointB
     * @param PointC
     * @return true/false
     */
    bool isTriangle (geometry_msgs::Point32 PointA, geometry_msgs::Point32 PointB, geometry_msgs::Point32 PointC)
    {
      float distanceA = calcDistance(PointA, PointB);
      float distanceB = calcDistance(PointB, PointC);
      float distanceC = calcDistance(PointC, PointA);

      if (distanceA < distanceB + distanceC && distanceB < distanceA + distanceC && distanceC < distanceB + distanceA)  //check if triangle can be built with given points with triangle inequality (a + b >= c)
      {
        return true;
      }

      return false;
    }


    /**
     * @brief getTriangleSides creates list of triangle sides from triangle data
     * @param triangle
     * @return sideList
     */
    localisation_pkg::triangleSideList getTriangleSides (localisation_pkg::triangle triangle)
    {
      localisation_pkg::triangleSide sideA;
      localisation_pkg::triangleSide sideB;
      localisation_pkg::triangleSide sideC;
      localisation_pkg::triangleSideList sideList;

      sideA.length = calcDistance(triangle.reflectors.at(0).position, triangle.reflectors.at(1).position);
      sideA.reflector1 = triangle.reflectors.at(0);
      sideA.reflector2 = triangle.reflectors.at(1);
      sideList.sides.push_back(sideA);

      sideB.length = calcDistance(triangle.reflectors.at(1).position, triangle.reflectors.at(2).position);
      sideB.reflector1 = triangle.reflectors.at(1);
      sideB.reflector2 = triangle.reflectors.at(2);
      sideList.sides.push_back(sideB);

      sideC.length = calcDistance(triangle.reflectors.at(2).position, triangle.reflectors.at(0).position);
      sideC.reflector1 = triangle.reflectors.at(2);
      sideC.reflector2 = triangle.reflectors.at(0);
      sideList.sides.push_back(sideC);

      sideList = sortSideList(sideList); //sort side list to make following reflector matching easier

      return sideList;
    }

    /**
     * @brief sortSideList sorts list of triangle sides by length
     * @param inputList
     * @return inputList(sorted)
     */
    localisation_pkg::triangleSideList sortSideList (localisation_pkg::triangleSideList inputList)
    {

      localisation_pkg::triangleSide tempSide;

      for (unsigned int i=0; i<inputList.sides.size(); i++)
      {
        for (unsigned int j=0; j<inputList.sides.size(); j++)
        {
          if(inputList.sides.at(j).length < inputList.sides.at(i).length) //using bubble sort algorithm
          {
            tempSide = inputList.sides.at(i);
            inputList.sides.at(i) = inputList.sides.at(j);
            inputList.sides.at(j) = tempSide;
          }
        }
      }

      return inputList;
    }

    /**
     * @brief filterMapTriangles used to filter Map triangles. Only triangles with longest side length < maxLength remain in vector.
     * @param inputTriangles
     * @param maxLength
     * @return filtered Triangles
     */
    localisation_pkg::trianglesList filterMapTriangles (localisation_pkg::trianglesList inputTriangles, float maxLength)
    {
      localisation_pkg::trianglesList filteredTriangles;

      for (unsigned int i=0; i<inputTriangles.triangles.size(); i++)
      {
        if (inputTriangles.triangles.at(i).sideList.sides.front().length < maxLength)
        {
          localisation_pkg::triangle tempTriangle;
          tempTriangle = inputTriangles.triangles.at(i);
          filteredTriangles.triangles.push_back(tempTriangle);
        }
      }

      return filteredTriangles;
    }


    /**
     * @brief findUsableTriangles finds all triangles from list of triangles that are usable (lidar inside)
     * @param inputTriangles
     * @return usableTriangles
     */
    localisation_pkg::trianglesList findUsableTriangles (localisation_pkg::trianglesList inputTriangles)
    {

      localisation_pkg::trianglesList usableTriangles;

      for (unsigned int i=0; i<inputTriangles.triangles.size(); i++)
      {
        if (inTriangle(inputTriangles.triangles.at(i)))
        {
            localisation_pkg::triangle tempTriangle;
            tempTriangle = inputTriangles.triangles.at(i);
            tempTriangle.usable = true;
            usableTriangles.triangles.push_back(tempTriangle);
        }
      }

      return usableTriangles;
    }

    /**
     * @brief inTriangle finds out wether the lidar is in the triangle or not
     * @param triangle
     * @return true/false
     */
    bool inTriangle (localisation_pkg::triangle triangle)
    {
      geometry_msgs::Point32 point;
      point.x = 0.0F;
      point.y = 0.0F;
      point.z = 0.0F;

      float checkSideA, checkSideB, checkSideC;
      bool cw, ccw;

      //check on which side of three vectors between triangle corners point is located with half-plane and determinant
      //https://www.aleph1.info/?call=Puc&permalink=hm1_5_5_Z2
      checkSideA = checkSide(point, triangle.reflectors.at(0).position, triangle.reflectors.at(1).position);
      checkSideB = checkSide(point, triangle.reflectors.at(1).position, triangle.reflectors.at(2).position);
      checkSideC = checkSide(point, triangle.reflectors.at(2).position, triangle.reflectors.at(0).position);

      cw = (checkSideA < 0) || (checkSideB < 0) || (checkSideC < 0); //point is left of min. one half-plane
      ccw = (checkSideA > 0) || (checkSideB > 0) || (checkSideC > 0);//point is right of min. one half-plane

      return !(cw && ccw);  //if both cases aboth are true, point can not be inside triangle
    }


    /**
     * @brief checkSide get determinant from 3 points that build a half plane and a vector
     * @param A
     * @param B
     * @param C
     * @return determinant
     */
    float checkSide (geometry_msgs::Point32 A, geometry_msgs::Point32 B, geometry_msgs::Point32 C)
    {
      return (A.x - C.x) * (B.y - C.y) - (B.x - C.x) * (A.y - C.y);
    }


    //-----------------------------pull gazebo model states--------------------------------//
    /**
     * @brief getReflectorPositions get list of reflector positions from gazebo model data
     * @param inputModelStates
     * @return reflectorList
     */
    localisation_pkg::reflectorList getReflectorPositions (gazebo_msgs::ModelStates inputModelStates)
    {
      std::vector<geometry_msgs::Pose> poseList;

      for (unsigned int i = 0; i<inputModelStates.name.size(); i++) //find index of reflector models
      {
        if (inputModelStates.name.at(i) == "reflector") poseList.push_back(inputModelStates.pose.at(i));
      }

      localisation_pkg::reflectorList reflectorList;

      for (unsigned int i=0; i<poseList.size(); i++)  //pull coordinates of reflectors
      {
        float PoseX = poseList.at(i).position.x;
        float PoseY = poseList.at(i).position.y;

        localisation_pkg::reflector tempReflector;
        tempReflector.position.x = PoseX;
        tempReflector.position.y = PoseY;
        tempReflector.position.z = 0.0F;
        tempReflector.label = i;

        reflectorList.reflectors.push_back(tempReflector);
      }

      return reflectorList;
    }

    /**
     * @brief getGazeboLidarPos get position of lidar from gazebo model data
     * @param inputModelStates
     * @return gazeboLidarPos
     */
    geometry_msgs::Point32 getGazeboLidarPos (gazebo_msgs::ModelStates inputModelStates)
    {
      geometry_msgs::Point32 gazeboLidarPos;

      for (unsigned int i = 0; i<inputModelStates.name.size(); i++) //find index of lidar and safe position coordinates
      {
        if (inputModelStates.name.at(i) == "lidar_robot")
        {
          gazeboLidarPos.x = inputModelStates.pose.at(i).position.x;
          gazeboLidarPos.y = inputModelStates.pose.at(i).position.y;
          gazeboLidarPos.z = 0;
        }
      }

      return gazeboLidarPos;
    }



    //-----------------------------match triangles--------------------------------//

    /**
     * @brief findMapCompareTriangles find map triangles that should be compared with lidar triangles (within radius of gnss-position of lidar)
     * @param mapTriangles
     * @param gnssPos
     * @param th
     * @return mapCompareTriangles
     */
    localisation_pkg::trianglesList findMapCompareTriangles (localisation_pkg::trianglesList mapTriangles, geometry_msgs::Point32 gnssPos, float th)
    {
      localisation_pkg::trianglesList mapCompareTriangles;

      for (unsigned int i=0; i<mapTriangles.triangles.size(); i++)
      {
        if (triangleInCircle(mapTriangles.triangles.at(i), gnssPos.x, gnssPos.y, th))
        {
          localisation_pkg::triangle tempTriangle;
          tempTriangle = mapTriangles.triangles.at(i);
          tempTriangle.usable = true;
          mapCompareTriangles.triangles.push_back(tempTriangle);
        }
      }

      return mapCompareTriangles;
    }


    /**
     * @brief triangleInCircle check wether triangle is in circle or not
     * @param triangle
     * @param centerX
     * @param centerY
     * @param r
     * @return bool
     */
    bool triangleInCircle (localisation_pkg::triangle triangle, float centerX, float centerY, float r)
    {
      unsigned int countInsidePoints = 0;

      for (unsigned int i=0; i<3; i++)
      {
        if (pointInCircle(triangle.reflectors.at(i).position, centerX, centerY, r))
        {
          countInsidePoints++;
        }
      }

      return countInsidePoints >= 3;
    }


    /**
     * @brief pointInCircle check wether point is in circle or not
     * @param point
     * @param centerX
     * @param centerY
     * @param r
     * @return bool
     */
    bool pointInCircle (geometry_msgs::Point32 point, float centerX, float centerY, float r)
    {
      return pow((point.x - centerX), 2) + pow((point.y - centerY), 2) <= pow(r, 2);
    }


    /**
     * @brief findTrianglePairs matches triangles from map and lidar
     * @param mapTriangles
     * @param usableTriangles
     * @return trianglePairs
     */
    localisation_pkg::trianglePairList findTrianglePairs (localisation_pkg::trianglesList mapTriangles, localisation_pkg::trianglesList usableTriangles, float tolerance)
    {
      localisation_pkg::trianglePairList trianglePairs;

      for (unsigned int i=0; i<usableTriangles.triangles.size(); i++) //compare triangles from map with triangles from lidar
      {
        for (unsigned int j=0; j<mapTriangles.triangles.size(); j++)
        {
          if (compareTriangles(usableTriangles.triangles.at(i), mapTriangles.triangles.at(j), tolerance)) //if triangles congruent safe them as a pair of triangles
          {
            localisation_pkg::trianglePair tempTrianglePair;
            tempTrianglePair.triangleLidar = usableTriangles.triangles.at(i);
            tempTrianglePair.triangleMap = mapTriangles.triangles.at(j);
            trianglePairs.trianglePairs.push_back(tempTrianglePair);
          }
        }
      }

      return trianglePairs;
    }

    /**
     * @brief compareTriangles compares triangles wether they are congruent or not
     * @param triangleA
     * @param triangleB
     * @return true/false
     */
    bool compareTriangles (localisation_pkg::triangle triangleA, localisation_pkg::triangle triangleB, float tolerance)
    {

      //use SSS congruence theorem to check congruency, use tolerance to compensate measurement errors from clustering
      if (abs(triangleA.sideList.sides.at(0).length-triangleB.sideList.sides.at(0).length)<tolerance && abs(triangleA.sideList.sides.at(1).length-triangleB.sideList.sides.at(1).length)<tolerance && abs(triangleA.sideList.sides.at(2).length-triangleB.sideList.sides.at(2).length)<tolerance)
      {
        return true;
      }
      return false;
    }



    //-----------------------------match reflectors--------------------------------//


    /**
     * @brief getCalcTriangelsList get list of triangles with matched reflectors
     * @param trianglePairs
     * @return calcTriangleList
     */
    localisation_pkg::calcTriangleList getCalcTriangelsList (localisation_pkg::trianglePairList trianglePairs)
    {
      localisation_pkg::calcTriangleList calcTriangleList;

      for (unsigned int i=0; i<trianglePairs.trianglePairs.size(); i++)
      {
        calcTriangleList.calcTriangles.push_back(matchReflectors(trianglePairs.trianglePairs.at(i)));
      }

      return calcTriangleList;
    }

    /**
     * @brief matchReflectors matches the reflectors of map triangle and lidar triangle
     * @param inputPair
     * @return outputPairs
     */
    localisation_pkg::calcTriangle matchReflectors (localisation_pkg::trianglePair inputPair)
    {
      std::vector<std::pair<localisation_pkg::reflectorPair, localisation_pkg::reflectorPair>> possibleReflectorPairs;
      localisation_pkg::calcTriangle outputPairs;

      geometry_msgs::Point32 lidarKOS;
      lidarKOS.x = 0.0F;
      lidarKOS.y = 0.0F;
      lidarKOS.z = 0.0F;

      for (unsigned int i=0; i<3; i++) //store every possible reflector match (between lidar reflector and map reflector)
      {
        std::pair <localisation_pkg::reflectorPair, localisation_pkg::reflectorPair> tempPairOfPair1; //matches are stored as pairs (if one pair of reflectors from one side is "true", the other pair from the side therefore is also "true")
        tempPairOfPair1.first.reflectorMap = inputPair.triangleMap.sideList.sides.at(i).reflector1;
        tempPairOfPair1.first.reflectorLidar = inputPair.triangleLidar.sideList.sides.at(i).reflector1;
        tempPairOfPair1.first.distance2Lidar = calcDistance(lidarKOS, tempPairOfPair1.first.reflectorLidar.position);
        tempPairOfPair1.second.reflectorMap = inputPair.triangleMap.sideList.sides.at(i).reflector2;
        tempPairOfPair1.second.reflectorLidar = inputPair.triangleLidar.sideList.sides.at(i).reflector2;
        tempPairOfPair1.second.distance2Lidar = calcDistance(lidarKOS, tempPairOfPair1.second.reflectorLidar.position);
        possibleReflectorPairs.push_back(tempPairOfPair1);

        std::pair <localisation_pkg::reflectorPair, localisation_pkg::reflectorPair> tempPairOfPair2;
        tempPairOfPair2.first.reflectorMap = inputPair.triangleMap.sideList.sides.at(i).reflector1;
        tempPairOfPair2.first.reflectorLidar = inputPair.triangleLidar.sideList.sides.at(i).reflector2;
        tempPairOfPair2.first.distance2Lidar = calcDistance(lidarKOS, tempPairOfPair2.first.reflectorLidar.position);
        tempPairOfPair2.second.reflectorMap = inputPair.triangleMap.sideList.sides.at(i).reflector2;
        tempPairOfPair2.second.reflectorLidar = inputPair.triangleLidar.sideList.sides.at(i).reflector1;
        tempPairOfPair2.second.distance2Lidar = calcDistance(lidarKOS, tempPairOfPair2.second.reflectorLidar.position);
        possibleReflectorPairs.push_back(tempPairOfPair2);
      }


     for (unsigned int j=0; j<possibleReflectorPairs.size(); j++) //compare all possible reflector matches among themselves
     {
       for (unsigned int k=j+1; k<possibleReflectorPairs.size(); k++)
       {
         if (compareReflectorPair (possibleReflectorPairs.at(j).first, possibleReflectorPairs.at(k).first) ||   //if a match appears twice in the list, it must be "true" (and the other pair of the pair must also be)
             compareReflectorPair (possibleReflectorPairs.at(j).second, possibleReflectorPairs.at(k).first)   )
         {
             outputPairs.reflectorPairs.push_back(possibleReflectorPairs.at(j).first);
             outputPairs.reflectorPairs.push_back(possibleReflectorPairs.at(j).second);
             outputPairs.reflectorPairs.push_back(possibleReflectorPairs.at(k).second);
             j=4;
             break;
         }
         else if (compareReflectorPair (possibleReflectorPairs.at(j).first, possibleReflectorPairs.at(k).second) ||
                  compareReflectorPair (possibleReflectorPairs.at(j).second, possibleReflectorPairs.at(k).second)    )
         {
             outputPairs.reflectorPairs.push_back(possibleReflectorPairs.at(j).first);
             outputPairs.reflectorPairs.push_back(possibleReflectorPairs.at(j).second);
             outputPairs.reflectorPairs.push_back(possibleReflectorPairs.at(k).first);
             j=4;
             break;
         }
       }
     }

      return outputPairs;
    }


    /**
     * @brief compareReflectorPair compares reflector pairs by label
     * @param pairA
     * @param pairB
     * @return true/false
     */
    bool compareReflectorPair (localisation_pkg::reflectorPair pairA, localisation_pkg::reflectorPair pairB)
    {
      if (pairA.reflectorMap.label == pairB.reflectorMap.label && pairA.reflectorLidar.label == pairB.reflectorLidar.label)
      {
        return true;
      }
      return false;
    }


    //-----------------------------calculate lidar position--------------------------------//
    /**
     * @brief getPosTriangelsList calculates lidar pos for each triangle pair with matched reflectors
     * @param inputList
     * @return calcTriangleList
     */
    localisation_pkg::calcTriangleList getPosTriangelsList (localisation_pkg::calcTriangleList inputList)
    {
      localisation_pkg::calcTriangleList calcTriangleList;

      for (unsigned int i=0; i<inputList.calcTriangles.size(); i++)
      {
        calcTriangleList.calcTriangles.push_back(calcLidarPos(inputList.calcTriangles.at(i)));
      }

      return calcTriangleList;
    }

    /**
     * @brief calcLidarPos calculate the lidar pos in map triangle with 2D-Trilateration
     * @param inputCalcTriangle
     * @return outputCalcTriangle
     */
    localisation_pkg::calcTriangle calcLidarPos (localisation_pkg::calcTriangle inputCalcTriangle)
    {          
      localisation_pkg::calcTriangle outputCalcTriangle;

      //safe some values in local variables to increase traceability
      geometry_msgs::Point32 point1 = inputCalcTriangle.reflectorPairs.at(0).reflectorMap.position; //map reflectors position are used to get lidar position in global COS
      geometry_msgs::Point32 point2 = inputCalcTriangle.reflectorPairs.at(1).reflectorMap.position;
      geometry_msgs::Point32 point3 = inputCalcTriangle.reflectorPairs.at(2).reflectorMap.position;
      float r1 = inputCalcTriangle.reflectorPairs.at(0).distance2Lidar;
      float r2 = inputCalcTriangle.reflectorPairs.at(1).distance2Lidar;
      float r3 = inputCalcTriangle.reflectorPairs.at(2).distance2Lidar;

      geometry_msgs::Point32 ex;
      geometry_msgs::Point32 aux;
      geometry_msgs::Point32 aux2;
      geometry_msgs::Point32 ey;


       //to follow 2D-triangulation along, please refere to textual documentation of project

      float p2p1Dist = calcDistance(point2,point1);
      ex.x = (point2.x - point1.x)/p2p1Dist;
      ex.y = (point2.y - point1.y)/p2p1Dist;
      aux.x = point3.x - point1.x;
      aux.y = point3.y - point1.y;

      float i = ex.x * aux.x + ex.y * aux.y;

      aux2.x = point3.x - point1.x - i * ex.x;
      aux2.y = point3.y - point1.y - i * ex.y;
      ey.x = aux2.x/normVector(aux2);
      ey.y = aux2.y/normVector(aux2);

      float j = ey.x * aux.x + ey.y * aux.y;

      float xTemp = (pow(r1,2) - pow(r2,2) + pow(p2p1Dist,2))/(2 * p2p1Dist);
      float yTemp = (pow(r1,2) - pow(r3,2) + pow(i,2) + pow(j,2))/(2*j) - i*xTemp/j;

      float x = point1.x + xTemp*ex.x + yTemp*ey.x;
      float y = point1.y + xTemp*ex.y + yTemp*ey.y;

      outputCalcTriangle = inputCalcTriangle;
      outputCalcTriangle.lidarPos.x = x;
      outputCalcTriangle.lidarPos.y = y;

      return outputCalcTriangle;

    }

    /**
     * @brief normVector get norm of a vector
     * @param point
     * @return norm
     */
    float normVector (geometry_msgs::Point32 point)
    {
        return sqrt(pow(point.x,2)+pow(point.y,2));
    }

    /**
     * @brief getMeanPosition calculate the mean lidar position from every triangle calculation, safe number of used triangles, calculate deviation to actual lidar position
     * @param inputList
     * @param gazeboLidarPos
     * @return outputPosition
     */
    localisation_pkg::calcPosition getMeanPosition (localisation_pkg::calcTriangleList inputList, geometry_msgs::Point32 gazeboLidarPos)
    {

      localisation_pkg::calcPosition outputPosition;
      std::vector<float> listX;
      std::vector<float> listY;
      unsigned int countTriangles = 0;

      for (unsigned int i=0; i<inputList.calcTriangles.size(); i++)
      {
        listX.push_back(inputList.calcTriangles.at(i).lidarPos.x);
        listY.push_back(inputList.calcTriangles.at(i).lidarPos.y);
        countTriangles++;
      }

      if (countTriangles == 0)
      {
        outputPosition.meanLidarPosition.x = NAN;
        outputPosition.meanLidarPosition.y = NAN;
        outputPosition.usedTriangles = countTriangles;
        outputPosition.deviationX = NAN;
        outputPosition.deviationY = NAN;
        outputPosition.locDeviation = NAN;
      }
      else
      {
        outputPosition.meanLidarPosition.x = findMedian(listX);
        outputPosition.meanLidarPosition.y = findMedian(listY);
        outputPosition.usedTriangles = countTriangles;
        outputPosition.deviationX = abs(gazeboLidarPos.x - outputPosition.meanLidarPosition.x);
        outputPosition.deviationY = abs(gazeboLidarPos.y - outputPosition.meanLidarPosition.y);
        outputPosition.locDeviation = calcDistance(outputPosition.meanLidarPosition, gazeboLidarPos);
      }

      return outputPosition;
    }

    /**
     * @brief findMedian calculate the median of a dataset in vector
     * @param a
     * @return median
     */
    float findMedian(std::vector<float> a)
    {
      size_t size = a.size();
      if (size == 0)
      {
        return 0.0F;
      }else
      {
        std::sort(a.begin(), a.end());
        if (size % 2 == 0)
        {
          return (a[size / 2 - 1] + a[size / 2]) / 2;
        }
        else
        {
          return a[size / 2];
        }

      }
    }


private:

};




#endif // LOCALISATIONFUNCTIONS_H
