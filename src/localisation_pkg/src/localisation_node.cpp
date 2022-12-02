#include "localisation_node.h"

float x, y, z;

//Call-Back-Funktion Lidar
void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
//  //Pointcloud message erstellen
//  sensor_msgs::PointCloud data_pc1;
//  //Eingelesene Pointcloud2 in Pointcloud wandeln
//  sensor_msgs::convertPointCloud2ToPointCloud(*msg, data_pc1);

//  //FILTERING
//  //Channels an Stelle 0 ist intensity, enthält name (string mit intensity) und values (vector mit intensitäten zu Punkten)
//  for (int i=0; i < data_pc1.points.size(); i++)
//  {
//    if (data_pc1.channels.at(0).values[i] > 100) //Intensität hoch genug
//    {
//      points_filtered.push_back(data_pc1.points[i]);
//    }
//  }

//  pcl::PCLPointCloud2 pcl_pc2; //Nachricht vom Typ Pointcloud2
//  pcl_conversions::toPCL(*input,pcl_pc2);
//  pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr temp_cloud(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
//  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

//  geometry_msgs::Point position;
//  geometry_msgs::PointStamped position2;
//  std::vector <geometry_msgs::Point> points_filt;
//  pcl::PointCloud<pcl::PointXYZ> test_cloud;


//  for (int i=0;i<(input->row_step*input->height)/input->point_step;i++)
//  {
//    if(temp_cloud->points[i].z >= -2.50)
//    {
//      x = temp_cloud->points[i].x;
//      y = temp_cloud->points[i].y;
//      z = temp_cloud->points[i].z;
//      position.x = x;
//      position.y = y;
//      position.z = z;

//      position2.point.x = x;
//      position2.point.y = y;
//      position2.point.z = z;

//      pcl::PointXYZ newPoint;
//      newPoint.x = x;
//      newPoint.y = y;
//      newPoint.z = z;
//      test_cloud.points.push_back(newPoint);

//      points_filt.push_back(position);


//      //pub.publish(position);
//      pub3.publish(position2);

//    }
//  }

  //pub2.publish(test_cloud.makeShared());



  //Speichern in Vektor
  //velodyne = data_pc1.points;

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

  //pub = nh.advertise<geometry_msgs::Point>("/test", 1000);

  //pub2 = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/testcloud", 10);

  pub3 = nh.advertise<geometry_msgs::PointStamped>("/testpoints", 10);


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
