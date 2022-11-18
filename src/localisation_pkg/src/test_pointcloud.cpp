#include "test_pointcloud.h"


ros::Publisher pub;
//ros::Publisher pub_intense;
//msg::test_pcl intensity
sensor_msgs::PointCloud pointcloud_1;
//Call-Back-Funktion Lidar
void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //Pointcloud message erstellen
  //Eingelesene Pointcloud2 in Pointcloud wandeln
  sensor_msgs::convertPointCloud2ToPointCloud(*msg,pointcloud_1);

  //Speichern in Vektor
  //velodyne = pointcloud_1;
}

void step()
{
//   for(int i = 0; i < sizeof(pointcloud_1.channels.at(0).values); i++){
//       intensity.intensity.at(i) = pointcloud_1.channels.at(0).values.at(i);
//   }
   pub.publish(pointcloud_1);
//   pub_intense.publish(intensity);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "localisation_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, LidarCallback);
  pub = nh.advertise<sensor_msgs::PointCloud>("/new_pcl", 10);
//  pub_intense = nh.advertise<std_msgs::Float32>("/new_intense", 10);
  listener = new tf::TransformListener;

  ros::Rate loopRate(10);
  // loop while ROS is running
  while (ros::ok()) {
    // pass control to ROS for background tasks
    step();
    ros::spinOnce();
    // wait for next sampling point
    // neighbor sampling points have a time distance of 100ms
    loopRate.sleep();
  }
  return 0;
}
