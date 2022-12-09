#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

ros::Publisher sim_pub;

gazebo_msgs::ModelState lidar;
geometry_msgs::Pose pose_lidar;
geometry_msgs::Twist twist_lidar;

std_msgs::Float32 msg;

float v = 2;
float f = 100;
float x;
float v_ms;

void step();

int main(int argc, char **argv)
{
  ROS_INFO("Simulation Starten");

  v_ms = v/3.6F;
  x = v_ms/f;

  ros::init(argc, argv, "move_sim");

  ros::NodeHandle nh;

  sim_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);

  pose_lidar.orientation.x = 0.0;
  pose_lidar.orientation.y = 0.0;
  pose_lidar.orientation.z = 0.0;
  pose_lidar.orientation.w = 0.0;
  pose_lidar.position.x = 0.0;
  pose_lidar.position.y = 0.0;
  pose_lidar.position.z = 0.0;

  twist_lidar.linear.x = 0.0;
  twist_lidar.linear.y = 0.0;
  twist_lidar.linear.z = 0.0;
  twist_lidar.angular.x = 3.14;
  twist_lidar.angular.y = 0.0;
  twist_lidar.angular.z = 0.0;

  lidar.model_name = "lidar_robot";
  lidar.pose = pose_lidar;
  lidar.twist = twist_lidar;

  ros::Rate loop_rate(f);

  while (ros::ok())
  {
    step();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void step()
{
  if (pose_lidar.position.x <= 40)
  {
    sim_pub.publish(lidar); //Publishen Pose Lidar
    pose_lidar.position.x += x;//Lidar weiterfahren lassen in x-Richtung
    lidar.pose = pose_lidar; //Pose aktualisieren
  }


  //Else Beenden Simulation
}
