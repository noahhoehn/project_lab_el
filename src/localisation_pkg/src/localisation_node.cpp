#include "localisation_node.h"

//Call-Back-Funktion Lidar
void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //Pointcloud message erstellen
  sensor_msgs::PointCloud points;
  //Eingelesene Pointcloud2 in Pointcloud wandeln
  sensor_msgs::convertPointCloud2ToPointCloud(*msg,points);
  //Speichern in Vektor
  velodyne = points.points;
}

void step()
{
   velodyne = transform_points(velodyne,"velodyne");
}

/**
 * @brief transform_points
 * Funktion für das umrechnen der Punktewolke eines Sensors auf das Fahrzeugkoordinatensystem
 * @param input
 * Vektor mit Punktwolke
 * @param koordinatensystem
 * String von welchem Koordinatensystem aus umgerechnet werden soll
 * @return
 * Umgerechnete Punktewolke
 */
std::vector <geometry_msgs::Point32>transform_points(std::vector <geometry_msgs::Point32> input,std::string koordinatensystem){
  //Variablen für die Transformation
  geometry_msgs::PoseStamped poseIn;
  geometry_msgs::PoseStamped poseOut;
  //Vector für die Ausgabe
  std::vector <geometry_msgs::Point32> output;
  //Variable für einen Punkt
  geometry_msgs::Point32 point;

  //For Schleife über alle Punkte im Inputvector
  for(unsigned int i = 0; i< input.size();i++){
    poseIn.pose.position.x = static_cast<double>(input.at(i).x);
    poseIn.pose.position.y = static_cast<double>(input.at(i).y);
    poseIn.pose.position.z = static_cast<double>(input.at(i).z);
    poseIn.pose.orientation.x = 0.0;
    poseIn.pose.orientation.y = 0.0;
    poseIn.pose.orientation.z = 0.0;
    poseIn.pose.orientation.w = 1.0;
    poseIn.header.frame_id = koordinatensystem;
    //Umrechnen auf Koordinatensystem von base_link
    listener->transformPose("base_link", poseIn, poseOut);
    //Komponenten zu Punkt zusammenfügen
    point.x = static_cast<float>(poseOut.pose.position.x);
    point.y = static_cast<float>(poseOut.pose.position.y);
    point.z = static_cast<float>(poseOut.pose.position.z);
    //Outputvector mit Punkt erweitern
    output.push_back(point);
  }
  return output;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localisation_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/velodyne", 1, LidarCallback);

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
