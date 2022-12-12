#ifndef LOCALISATIONFUNCTIONS_H
#define LOCALISATIONFUNCTIONS_H

#pragma once

#include <ros/ros.h>
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
#include <geometry_msgs/Polygon.h>
#include "localisation_pkg/pointList.h"
#include "localisation_pkg/reflector.h"
#include "localisation_pkg/reflectorList.h"
#include "localisation_pkg/triangle.h"
#include "localisation_pkg/trianglesList.h"
#include "localisation_pkg/trianglePair.h"
#include "localisation_pkg/trianglePairList.h"

class LocalisationFunctions
{
public:
    LocalisationFunctions(){}

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
        filter.setFilterLimits(150.0, 250.0);
        // Apply filter -> cloud_filtered is the filtered PCL PointCloud2
        filter.filter(pclCloud2Filtered);

        // Create sensor_msgs/PointCloud2
        sensor_msgs::PointCloud2 msgCloud2Filtered;
        // Convert PCL PointCloud2 to sensor_msgs/PointCloud2
        pcl_conversions::moveFromPCL(pclCloud2Filtered, msgCloud2Filtered);

        //Return filtered sensor_msgs/PointCloud2
        return msgCloud2Filtered;
    }

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


    localisation_pkg::pointList clusterPointCloud(const localisation_pkg::pointList inputPoints)
    {
      std::set<unsigned int> ignoreIndices;
      localisation_pkg::pointList clusterCentroids;
      localisation_pkg::pointList tempNearPoints;

      for (unsigned int i = 0; i < inputPoints.points.size(); i++)
      {
        if (ignoreIndices.find(i) == ignoreIndices.end())
        {
          for (unsigned int j = 0; j < inputPoints.points.size(); j++)
          {
            if (i != j && ignoreIndices.find(j) == ignoreIndices.end())
            {
              float distance = calcDistance(inputPoints.points.at(i), inputPoints.points.at(j));
              if (distance < 1.0F)
              {
                tempNearPoints.points.push_back(inputPoints.points.at(j));
                ignoreIndices.insert(j);
              }
            }
           }

          if (tempNearPoints.points.size()>1)
          {
            tempNearPoints.points.push_back(inputPoints.points.at(i));
            ignoreIndices.insert(i);
            clusterCentroids.points.push_back(calcCentroid(tempNearPoints));
          }
        }
        tempNearPoints.points.clear();
      }

      return clusterCentroids;
    }



    float calcDistance (geometry_msgs::Point32 pointA, geometry_msgs::Point32 pointB)
    {
      float distance = std::sqrt(std::pow(pointA.x - pointB.x,2) + std::pow(pointA.y - pointB.y,2) + std::pow(pointA.z - pointB.z,2));
      return distance;
    }


    geometry_msgs::Point32 calcCentroid (localisation_pkg::pointList inputPoints)
    {
      std::vector<float> collectX;
      std::vector<float> collectY;
      std::vector<float> collectZ;
      float averageX = 0.0F;
      float averageY = 0.0F;
      float averageZ = 0.0F;

      for (unsigned int k = 0; k < inputPoints.points.size(); k++)
      {
        collectX.push_back(inputPoints.points.at(k).x);
        collectY.push_back(inputPoints.points.at(k).y);
        collectZ.push_back(inputPoints.points.at(k).z);
      }

      auto const vectorSize = static_cast<float>(inputPoints.points.size());

      averageX = std::accumulate(collectX.begin(), collectX.end(), 0.0F)/vectorSize;
      averageY = std::accumulate(collectY.begin(), collectY.end(), 0.0F)/vectorSize;
      averageZ = std::accumulate(collectZ.begin(), collectZ.end(), 0.0F)/vectorSize;

      geometry_msgs::Point32 averagePoint;

      averagePoint.x = averageX;
      averagePoint.y = averageY;
      averagePoint.z = averageZ;

      return averagePoint;
    }



    localisation_pkg::trianglesList findTriangles (localisation_pkg::reflectorList inputReflectors)
    {
      for (unsigned int i=0; i<inputReflectors.reflectors.size(); i++)
      {
        inputReflectors.reflectors.at(i).position.z = 0.0F;
      }

      localisation_pkg::trianglesList triangleList;

      for (unsigned int i=0; i<inputReflectors.reflectors.size(); i++)
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
              tempTriangle.usable = false;
              triangleList.triangles.push_back(tempTriangle);
            }
          }
        }
      }
      return triangleList;
    }




    bool isTriangle (geometry_msgs::Point32 PointA, geometry_msgs::Point32 PointB, geometry_msgs::Point32 PointC)
    {
      float distanceA = calcDistance(PointA, PointB);
      float distanceB = calcDistance(PointB, PointC);
      float distanceC = calcDistance(PointC, PointA);

      if (distanceA < distanceB + distanceC && distanceB < distanceA + distanceC && distanceC < distanceB + distanceA)
      {
        return true;
      }

      return false;
    }




    localisation_pkg::reflectorList getReflectorPositions (gazebo_msgs::ModelStates inputModelStates)
    {
      std::vector<geometry_msgs::Pose> poseList;

      for (unsigned int i = 0; i<inputModelStates.name.size(); i++)
      {
        if (inputModelStates.name.at(i) == "reflector") poseList.push_back(inputModelStates.pose.at(i));
      }

      localisation_pkg::reflectorList reflectorList;

      for (unsigned int i=0; i<poseList.size(); i++)
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

    localisation_pkg::reflectorList labelClusterCentroids (localisation_pkg::pointList inputCentroids)
    {
      localisation_pkg::reflectorList labeledClusterCentroids;

      for (unsigned int i=0; i<inputCentroids.points.size(); i++)
      {
        localisation_pkg::reflector tempReflector;
        tempReflector.position = inputCentroids.points.at(i);
        tempReflector.label = i;

        labeledClusterCentroids.reflectors.push_back(tempReflector);
      }

      return labeledClusterCentroids;
    }

    bool compareTriangles (localisation_pkg::triangle triangleA, localisation_pkg::triangle triangleB)
    {
      float tolerance = 0.5F;

      float lengthAa = calcDistance(triangleA.reflectors.at(0).position, triangleA.reflectors.at(1).position);
      float lengthAb = calcDistance(triangleA.reflectors.at(1).position, triangleA.reflectors.at(2).position);
      float lengthAc = calcDistance(triangleA.reflectors.at(2).position, triangleA.reflectors.at(0).position);
      std::vector<float> lengthA = {lengthAa, lengthAb, lengthAc};

      float lengthBa = calcDistance(triangleB.reflectors.at(0).position, triangleB.reflectors.at(1).position);
      float lengthBb = calcDistance(triangleB.reflectors.at(1).position, triangleB.reflectors.at(2).position);
      float lengthBc = calcDistance(triangleB.reflectors.at(2).position, triangleB.reflectors.at(0).position);
      std::vector<float> lengthB = {lengthBa, lengthBb, lengthBc};

      sort(lengthA.begin(), lengthA.end());
      sort(lengthB.begin(), lengthB.end());

      if (abs(lengthA[0]-lengthB[0])<tolerance && abs(lengthA[1]-lengthB[1])<tolerance && abs(lengthA[2]-lengthB[2])<tolerance)
      {
        return true;
      }
      return false;
    }


    localisation_pkg::trianglePairList findTrianglePairs (localisation_pkg::trianglesList mapTriangles, localisation_pkg::trianglesList usableTriangles)
    {
      localisation_pkg::trianglePairList trianglePairs;

      for (unsigned int i=0; i<usableTriangles.triangles.size(); i++)
      {
        for (unsigned int j=0; j<mapTriangles.triangles.size(); j++)
        {
          if (compareTriangles(usableTriangles.triangles.at(i), mapTriangles.triangles.at(j)))
          {
            localisation_pkg::trianglePair tempTrianglePair;
            tempTrianglePair.triangleA = usableTriangles.triangles.at(i);
            tempTrianglePair.triangleB = mapTriangles.triangles.at(j);
            trianglePairs.trianglePairs.push_back(tempTrianglePair);
          }
        }
      }

      return trianglePairs;
    }



    bool inTriangle (localisation_pkg::triangle triangle)
    {
      geometry_msgs::Point32 point;
      point.x = 0.0F;
      point.y = 0.0F;
      point.z = 0.0F;

      float checkSideA, checkSideB, checkSideC;
      bool cw, ccw;

      //Checken auf welcher Seite von 3 Geraden zwischen Eckpunkten Dreieck liegt (durch Aufspannen Halbebene und Bestimmen Determinante)
      //https://www.aleph1.info/?call=Puc&permalink=hm1_5_5_Z2
      checkSideA = checkSide(point, triangle.reflectors.at(0).position, triangle.reflectors.at(1).position);
      checkSideB = checkSide(point, triangle.reflectors.at(1).position, triangle.reflectors.at(2).position);
      checkSideC = checkSide(point, triangle.reflectors.at(2).position, triangle.reflectors.at(0).position);

      cw = (checkSideA < 0) || (checkSideB < 0) || (checkSideC < 0); //Punkt liegt links von min. einer Ebene
      ccw = (checkSideA > 0) || (checkSideB > 0) || (checkSideC > 0);//Punkt liegt rechts von min. einer Ebene

      return !(cw && ccw);  //wenn beide Fälle eintreten kann Punkt nicht in Dreieck liegen
    }



    float checkSide (geometry_msgs::Point32 A, geometry_msgs::Point32 B, geometry_msgs::Point32 C)
    {
      return (A.x - C.x) * (B.y - C.y) - (B.x - C.x) * (A.y - C.y);
    }




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



private:
};


#endif // LOCALISATIONFUNCTIONS_H
