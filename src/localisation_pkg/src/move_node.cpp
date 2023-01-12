#include <ros/ros.h>
#include <stdio.h>
#include <movefunctions.h>


class MoveNode
{
public:

  using moveType = MoveFunctions;


  MoveNode (const float samplingTimeArg):
    samplingTime(samplingTimeArg)
  {
    move_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
  }

  void step()
  {
    move_pub.publish(lidar);
    lidar = move.moveLidar(lidar, scenario, samplingTime, &endPoint);
    if (endPoint == 1)
    {
      ROS_INFO("Endpoint reached. LiDAR-Movement ended.");
      ros::shutdown();
    }
  }

  void initMove(int s) //called once when starting node to set scenario given by argument "S"
  {
    scenario = s;
    int err = 0;
    lidar = move.initLidar(s, &err);
    if (err == 1)
    {
      ROS_ERROR("Invalid Scenario Parameter: [%d]", s);
      ros::shutdown();
    }
    else
    {
      ROS_INFO("Scenario [%d] selected. LiDAR-Movement started.", s);
    }
  }

private:

  moveType move;

  ros::NodeHandle nh { "~" }; /**< The ROS node handle. */
  const float samplingTime = 0.0F;

  ros::Publisher move_pub;

  gazebo_msgs::ModelState lidar;
  geometry_msgs::Pose pose_lidar;
  geometry_msgs::Twist twist_lidar;

  std_msgs::Float32 msg;

  int scenario = 0;

  bool endPoint = 0; //used to check wether endpoint of movement is reached

};


int main(int argc, char **argv)
{
    const float samplingTime = 100e-3F;
    ros::init(argc, argv, "move_node");
    MoveNode node(samplingTime);
    ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));
    ros::NodeHandle nh2("~");
    int scenario = 0;
    nh2.getParam("S", scenario);
    node.initMove(scenario);
    while (ros::ok()) {
        node.step();
        ros::spinOnce();
        loopRate.sleep();
    }
    // return success
    return EXIT_SUCCESS;
}
