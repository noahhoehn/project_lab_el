#ifndef MOVEFUNCTIONS_H
#define MOVEFUNCTIONS_H

#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>



class MoveFunctions
{
public:

  MoveFunctions(){}

  /**
   * @brief initLidar initialise lidar in position and orientation depending on selected scenario
   * @param scenario
   * @param err
   * @return lidar
   */
  gazebo_msgs::ModelState initLidar(int scenario, int *err)
  {
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    gazebo_msgs::ModelState lidar;

    tf2::Quaternion q;
    q.setRPY (0.0, 0.0 , 0.0);
    q = q.normalize();

    pose.orientation.x = q[0];
    pose.orientation.y = q[1];
    pose.orientation.z = q[2];
    pose.orientation.w = q[3];
    pose.position.z = 0.0;

    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 3.14;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;


    switch (scenario)
    {
      case 1:
        pose.position.x = -5;
        pose.position.y = 10;
      break;
      case 2:
        pose.position.x = -5;
        pose.position.y = 28;
      break;
      case 3:
        pose.position.x = -5;
        pose.position.y = 28;
      break;
      case 4:
        pose.position.x = -80;
        pose.position.y = 60;
      break;
      case 5:
        pose.position.x = -5;
        pose.position.y = 26;
      break;
      default:
        *err = 1;
    }

    lidar.model_name = "lidar_robot";
    lidar.pose = pose;
    lidar.twist = twist;

    return lidar;
  }

  /**
   * @brief moveLidar moving the lidar depending on selected scenario
   * @param lidar
   * @param scenario
   * @param samplingTime
   * @param endPoint
   * @return lidar
   */
  gazebo_msgs::ModelState moveLidar(gazebo_msgs::ModelState lidar, int scenario, const float samplingTime, int* endPoint)
  {
    switch (scenario)
    {
      case 1:
        if (lidar.pose.position.x <= 50)
        {
          lidar = moveLinear(lidar, 1.0F, 0.0F, calcWayPerStep(10.0F, samplingTime));
        }
        else
        {
          *endPoint = 1;
        }
      break;
      case 2:
        if (lidar.pose.position.x <= 85)
        {
          lidar = moveLinear(lidar, 1.0F, 0.0F, calcWayPerStep(10.0F, samplingTime));
        }
        else
        {
          *endPoint = 1;
        }
      break;
      case 3:
        if (lidar.pose.position.x <= 85)
        {
          lidar = moveLinear(lidar, 1.0F, 0.0F, calcWayPerStep(10.0F, samplingTime));
        }
        else
        {
          *endPoint = 1;
        }
      break;
      case 4:
        if (phase == 0)
        {
          lidar = moveLinear(lidar, 88.0F, -7.0F, calcWayPerStep(10.0F, samplingTime));
          if (lidar.pose.position.x >= 8) phase = 1;
        }

        else if (phase == 1)
        {
          lidar = moveCircular(lidar, 8, 63, 10, calcWayPerStep(10.0F,samplingTime));
          if (lidar.pose.position.x <= 8 && lidar.pose.position.y > 70) phase = 2;
        }
        else if (phase == 2)
        {
          lidar = moveLinear(lidar, -28.0F, 2.0F, calcWayPerStep(10.0F, samplingTime));
          if (lidar.pose.position.x <= -20) phase = 3;
        }
        else
        {
          *endPoint = 1;
        }
      break;
      case 5:
        if (phase == 0)
        {
          lidar = moveLinear(lidar, 67.0F, 0.0F, calcWayPerStep(10.0F, samplingTime));
          if (lidar.pose.position.x >= 72) phase = 1;
        }

        else if (phase == 1)
        {
          lidar = moveCircular(lidar, 72, 35.5, 9.5, calcWayPerStep(10,samplingTime));
          if (lidar.pose.position.x <= 72 && lidar.pose.position.y > 42) phase = 2;
        }
        else if (phase == 2)
        {
          lidar = moveLinear(lidar, -27, 2.0F, calcWayPerStep(10.0F, samplingTime));
          if (lidar.pose.position.x <= 45) phase = 3;
        }
        else
        {
          *endPoint = 1;
        }
      break;
    }

    return lidar;

  }

  /**
   * @brief calcWayPerStep calculate the way per sampling step with given speed and samplingTime
   * @param v
   * @param samplingTime
   * @return s_step
   */
  float calcWayPerStep (float v, const float samplingTime)
  {
    float s_step = (v*samplingTime)/3.6F;
    return s_step;
  }

  /**
   * @brief moveLinear moving the lidar linear in direction of given vector at given speed
   * @param lidar
   * @param compX
   * @param compY
   * @param sStep
   * @return lidar
   */
  gazebo_msgs::ModelState moveLinear(gazebo_msgs::ModelState lidar, float compX, float compY, float sStep)
  {
    float dirX = compX/getNorm(compX,compY);
    float dirY = compY/getNorm(compX, compY);
    float sX = sStep * dirX;
    float sY = sStep * dirY;

    lidar.pose.position.x += sX;
    lidar.pose.position.y += sY;

    return lidar;
  }

  float getNorm (float compX, float compY)
  {
    return sqrt(pow(compX,2)+pow(compY,2));
  }



  /**
   * @brief moveCircular moving the lidar along a circle with given origin and radius at given speed
   * @param lidar
   * @param originX
   * @param originY
   * @param r
   * @param sStep
   * @return lidar
   */
  gazebo_msgs::ModelState moveCircular(gazebo_msgs::ModelState lidar, double originX, double originY, double r, float sStep)
  {
    double phiStep = sStep/r;

    tf2::Quaternion qOrig, qRot,qNew;
    tf2::convert(lidar.pose.orientation, qOrig);
    qRot.setRPY(0, 0, phiStep);
    qNew = qRot * qOrig;
    qNew.normalize();
    tf2::convert(qNew, lidar.pose.orientation);

    double yaw = getYaw(lidar);
    double sX = r*cos(yaw-M_PI/2); //Offset pi/2 because of orientation of map in scenario 4
    double sY = r*sin(yaw-M_PI/2);
    lidar.pose.position.x = originX + sX;
    lidar.pose.position.y = originY + sY;

    return lidar;
  }

  /**
   * @brief getYaw reads yaw out of quaternion
   * @param lidar
   * @return yaw
   */
  double getYaw (gazebo_msgs::ModelState lidar)
  {
    tf2::Quaternion q;
    tf2::convert(lidar.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }


private:

  unsigned int phase = 0;


};

#endif // MOVEFUNCTIONS_H
