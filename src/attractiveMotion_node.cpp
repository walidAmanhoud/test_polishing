#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MoveToDesiredJoints.h"
#include "geometry_msgs/Quaternion.h"
#include "AttractiveMotionGenerator.h"
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "attractiveMotion");

  ros::NodeHandle nh;
  double frequency = 500.0;


  // Parameters
  std::string inputTopicName = "/lwr/ee_pose";
  std::string outputTopicName = "/lwr/joint_controllers/passive_ds_command_vel";
  // std::string outputTopicName = "/lwr/joint_controllers/command_pos";

  // Check if attractor position is specified with the command line
  float px, py, pz;


  if(argc == 4)
  {
    px = atof(argv[1]);
    py = atof(argv[2]);
    pz = atof(argv[3]);
  }
  else
  {
    return 0;
  }

  float convergenceRate = 1.0f;


  geometry_msgs::Quaternion msg;
  msg.x = 0.0f;
  msg.y = 1.0f;
  msg.z = 0.0f;
  msg.w = 0.0f;



  AttractiveMotionGenerator attractiveMotionGenerator(nh,frequency,inputTopicName,outputTopicName, convergenceRate);

  if (!attractiveMotionGenerator.init()) 
  {
    return -1;
  }
  else 
  {
    attractiveMotionGenerator.setAttractorPosition(px,py,pz);
    attractiveMotionGenerator.setDesiredOrientation(msg);
    attractiveMotionGenerator.run();
  }
}

