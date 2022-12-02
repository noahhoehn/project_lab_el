#include "test_pointcloud.h"


ros::Publisher pub;
//ros::Publisher pub_intense;
//msg::test_pcl intensity
sensor_msgs::PointCloud pointcloud_1;

///**
// * @brief transform_points
// * Funktion für das umrechnen der Punktewolke eines Sensors auf das Fahrzeugkoordinatensystem
// * @param input
// * Vektor mit Punktwolke
// * @param koordinatensystem
// * String von welchem Koordinatensystem aus umgerechnet werden soll
// * @return
// * Umgerechnete Punktewolke
// */
//sensor_msgs::PointCloud transform_points(sensor_msgs::PointCloud input, std::string koordinatensystem){
//  //Variablen für die Transformation
//  geometry_msgs::PoseStamped poseIn;
//  geometry_msgs::PoseStamped poseOut;
//  //Vector für die Ausgabe
//  std::vector <geometry_msgs::Point32> output;
//  //Variable für einen Punkt
//  geometry_msgs::Point32 point;

//  //For Schleife über alle Punkte im Inputvector
//  for(unsigned int i = 0; i< input.points.size();i++){
//    poseIn.pose.position.x = static_cast<double>(input.points.at(i).x);
//    poseIn.pose.position.y = static_cast<double>(input.points.at(i).y);
//    poseIn.pose.position.z = static_cast<double>(input.points.at(i).z);
//    poseIn.pose.orientation.x = 0.0;
//    poseIn.pose.orientation.y = 0.0;
//    poseIn.pose.orientation.z = 0.0;
//    poseIn.pose.orientation.w = 1.0;
//    poseIn.header.frame_id = koordinatensystem;
//    //Umrechnen auf Koordinatensystem von base_link
//    listener->transformPose("base_link", poseIn, poseOut);
//    //Komponenten zu Punkt zusammenfügen
//    point.x = static_cast<float>(poseOut.pose.position.x);
//    point.y = static_cast<float>(poseOut.pose.position.y);
//    point.z = static_cast<float>(poseOut.pose.position.z);
//    //Outputvector mit Punkt erweitern
//    output.push_back(point);
//  }
//  input.points = output;
//  return input;
//}


//Call-Back-Funktion Lidar
void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //Pointcloud message erstellen
  //Eingelesene Pointcloud2 in Pointcloud wandeln
  sensor_msgs::convertPointCloud2ToPointCloud(*msg,pointcloud_1);
  sensor_msgs::PointCloud points_filtered;


  for (int i=0; i < pointcloud_1.points.size(); i++)
  {
    if (pointcloud_1.points[i].z <-1)
    {

      //      temp_cloud->points[i] = temp_cloud->points[temp_cloud->points.size()-1];
      //      temp_cloud->points.resize(temp_cloud->points.size()-1);
      //      i--;
    }
  }

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
