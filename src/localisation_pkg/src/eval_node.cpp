#include <ros/ros.h>
#include <stdio.h>
#include <localisation_pkg/calcPosition.h>
#include <fstream>
#include <std_msgs/Bool.h>


class EvalNode
{
public:

  EvalNode (const float samplingTimeArg):
    samplingTime(samplingTimeArg)
  {
    //Subscriptions
    calcPositionSub = node.subscribe("/calcPosition", 1, &EvalNode::calcPosCallback, this);
    gazeboLidarPosSub = node.subscribe("/gazeboLidarPos", 1, &EvalNode::actualPosCallback, this);
  }

  void step()
  {
    simTime = ros::Time::now() - begin; //calculate Simulation Time
    outfile.open(fName, std::ios_base::app); //open file
    outfile << simTime << ";" << actualPos.x << ";" << actualPos.y << ";" << calcPos.x << ";" << calcPos.y << ";" << deviation << "\n"; //safe data to file
    outfile.close();  //close file
  }

  void getFileName (std::string filename)
  {
    //safe filename given by argument or default falue
    fName = filename + ".txt";
  }

private:

  ros::NodeHandle node { "~" }; /**< The ROS node handle. */
  const float samplingTime = 0.0F;

  ros::Subscriber calcPositionSub;
  ros::Subscriber gazeboLidarPosSub;

  geometry_msgs::Point32 actualPos;
  geometry_msgs::Point32 calcPos;
  localisation_pkg::calcPosition msgCalcPos;
  float deviation;

  std::ofstream outfile;
  ros::Time begin = ros::Time::now();
  ros::Time now;
  ros::Duration simTime;

  std::string fName = "eval.txt"; //default file name

  void calcPosCallback (const localisation_pkg::calcPosition::ConstPtr& msg)
  {
    msgCalcPos = *msg;
    calcPos = msgCalcPos.meanLidarPosition;
    deviation = msgCalcPos.locDeviation;
  }

  void actualPosCallback (const geometry_msgs::Point32::ConstPtr& msg)
  {
    actualPos = *msg;
  }

};

int main(int argc, char **argv)
{
    const float samplingTime = 10e-3F;
    ros::init(argc, argv, "eval_node");
    EvalNode node(samplingTime);
    ros::Rate loopRate(static_cast<double>(1.0F / samplingTime));
    ros::NodeHandle nh2("~");
    std::string filename;
    nh2.getParam("filename", filename);
    node.getFileName(filename);
    ROS_INFO("Data-Recording started.");
    while (ros::ok()) {
        node.step();
        ros::spinOnce();
        loopRate.sleep();
    }
    return EXIT_SUCCESS;
}
