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

    std::vector<geometry_msgs::Point32> convertPcl2toVector(const sensor_msgs::PointCloud2 msgCloud2)
    {
      //create vector to store points with XYZ-values
      std::vector<geometry_msgs::Point32> points;
      //convert PointCloud2 to PointCloud to access points
      sensor_msgs::PointCloud msgCloud;
      sensor_msgs::convertPointCloud2ToPointCloud(msgCloud2,msgCloud);
      //write points from PointCloud data to vector
      points = msgCloud.points;
      return points;
    }


    std::vector<geometry_msgs::Point32> clusterPointCloud(const std::vector<geometry_msgs::Point32> inputPoints)
    {
      std::vector<geometry_msgs::Point32> inputCopy = inputPoints;
      std::set<int> ignoreIndices;
      std::vector<geometry_msgs::Point32> clusterCentroids;
      std::vector<std::vector<geometry_msgs::Point32>> clusterList;
      std::vector<geometry_msgs::Point32> tempNearPoints;


      for (unsigned int i = 0; i < inputCopy.size(); i++)
      {
        if (ignoreIndices.find(i) == ignoreIndices.end())
        {
          for (unsigned int j = 0; j < inputCopy.size(); j++)
          {
            if (i != j && ignoreIndices.find(j) == ignoreIndices.end())
            {
              float distance = calcDistance(inputCopy.at(i), inputCopy.at(j));
              if (distance < 1.0F)
              {
                tempNearPoints.push_back(inputCopy.at(j));
                ignoreIndices.insert(j);
              }
            }
           }

          if (tempNearPoints.size()>1)
          {
            tempNearPoints.push_back(inputCopy.at(i));
            ignoreIndices.insert(i);
            clusterList.push_back(tempNearPoints);
            clusterCentroids.push_back(calcCentroid(tempNearPoints));
          }
        }
        tempNearPoints.clear();
      }

      return clusterCentroids;
    }



    float calcDistance (geometry_msgs::Point32 pointA, geometry_msgs::Point32 pointB)
    {
      float distance = std::sqrt(std::pow(pointA.x - pointB.x,2) + std::pow(pointA.y - pointB.y,2) + std::pow(pointA.z - pointB.z,2));
      return distance;
    }


    geometry_msgs::Point32 calcCentroid (std::vector<geometry_msgs::Point32> inputPoints)
    {
      std::vector<float> collectX;
      std::vector<float> collectY;
      std::vector<float> collectZ;
      float averageX = 0.0F;
      float averageY = 0.0F;
      float averageZ = 0.0F;

      for (unsigned int k = 0; k < inputPoints.size(); k++)
      {
        collectX.push_back(inputPoints.at(k).x);
        collectY.push_back(inputPoints.at(k).y);
        collectZ.push_back(inputPoints.at(k).z);
      }

      auto const vectorSize = static_cast<float>(inputPoints.size());

      averageX = std::accumulate(collectX.begin(), collectX.end(), 0.0F)/vectorSize;
      averageY = std::accumulate(collectY.begin(), collectY.end(), 0.0F)/vectorSize;
      averageZ = std::accumulate(collectZ.begin(), collectZ.end(), 0.0F)/vectorSize;

      geometry_msgs::Point32 averagePoint;

      averagePoint.x = averageX;
      averagePoint.y = averageY;
      averagePoint.z = averageZ;

      return averagePoint;
    }



    std::vector<geometry_msgs::Polygon> findTriangles (std::vector<geometry_msgs::Point32> pointsList)
    {
      for (unsigned int i=0; i<pointsList.size(); i++)
      {
        pointsList.at(i).z = 0.0F;
      }

      std::vector<geometry_msgs::Polygon> triangleList;

      for (unsigned int i=0; i<pointsList.size(); i++)
      {
        for (unsigned int j=i+1; j<pointsList.size(); j++)
        {
          for (unsigned int k=j+1; k<pointsList.size(); k++)
          {
            if(isTriangle(pointsList.at(i), pointsList.at(j), pointsList.at(k)))
            {
              geometry_msgs::Polygon tempTriangle;
              tempTriangle.points.push_back(pointsList.at(i));
              tempTriangle.points.push_back(pointsList.at(j));
              tempTriangle.points.push_back(pointsList.at(k));
              triangleList.push_back(tempTriangle);
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

    std::vector<geometry_msgs::Point32> getReflectorPositions (gazebo_msgs::ModelStates inputModelStates)
    {
      std::vector<geometry_msgs::Pose> poseList;
      for (unsigned int i = 0; i<inputModelStates.name.size(); i++)
      {
        if (inputModelStates.name.at(i) == "reflector") poseList.push_back(inputModelStates.pose.at(i));
      }
      std::vector<geometry_msgs::Point32> pointList;

      for (unsigned int i=0; i<poseList.size(); i++)
      {
        float PoseX = poseList.at(i).position.x;
        float PoseY = poseList.at(i).position.y;

        geometry_msgs::Point32 tempPoint;
        tempPoint.x = PoseX;
        tempPoint.y = PoseY;
        tempPoint.z = 0.0F;

        pointList.push_back(tempPoint);
      }

      return pointList;
    }





private:
};


#endif // LOCALISATIONFUNCTIONS_H
