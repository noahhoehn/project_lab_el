#include "test_pointcloud.h"

//Call-Back-Funktion Lidar
void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //Pointcloud message erstellen
  sensor_msgs::PointCloud pointcloud_1;
  //Eingelesene Pointcloud2 in Pointcloud wandeln
  sensor_msgs::convertPointCloud2ToPointCloud(*msg,pointcloud_1);
  //Speichern in Vektor
  velodyne = pointcloud_1.points;
}

void step()
{
   velodyne = transform_points(velodyne,"velodyne");
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "localisation_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, LidarCallback);

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
