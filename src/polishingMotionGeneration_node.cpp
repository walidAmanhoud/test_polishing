#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MoveToDesiredJoints.h"
#include "geometry_msgs/Quaternion.h"
#include "CycleMotionGenerator.h"
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "polishFixedTarget");

  ros::NodeHandle nh;
  double frequency = 500.0;


  // Parameters
  std::string input_topic_name = "/lwr/ee_pose";
  std::string output_filtered_topic_name = "/ds1/desired_velocity";
  std::string output_topic_name = "/lwr/joint_controllers/passive_ds_command_vel";
  // std::string output_topic_name = "/lwr/joint_controllers/command_pos";

  std::vector<double> CenterRotation(3);

  for(int k = 0; k < 3; k++)
  {
    CenterRotation[k] = 0.0f;
  }

  // Check if initial position is specified with the command line
  if(argc == 4)
  {
    for(int k = 0; k < 3; k++)
    {
      CenterRotation[k] = atof(argv[k+1]);
    }
  }


  double radius = 0.05f;
  double RotationSpeed = M_PI;
  double ConvergenceRate = 3.0f;


  // ros::Publisher pub = nh.advertise<geometry_msgs::Quaternion>("lwr/joint_controllers/passive_ds_command_orient", 1);

  geometry_msgs::Quaternion msg;
  // msg.x = 0.7071f;
  // msg.y = 0.7071f;
  // msg.z = 0.0f;
  // msg.w = 0.0f;
  msg.x = 0.0f;
  msg.y = 1.0f;
  msg.z = 0.0f;
  msg.w = 0.0f;

  // pub.publish(msg);
  // ros::spinOnce();


  CycleMotionGenerator cycle_motion_generator(nh,frequency,
      input_topic_name,
      output_topic_name,
      output_filtered_topic_name,
      CenterRotation,
      radius,
      RotationSpeed,
      ConvergenceRate);

  if (!cycle_motion_generator.Init()) 
  {
    return -1;
  }
  else 
  {
    bool ready = false;
    if(nh.hasParam("ready"))
    {
      while(!ready)
      {

        nh.getParam("ready", ready);
      }
    
      ROS_INFO("Start polishing");
      cycle_motion_generator.setDesiredOrientation(msg);
      cycle_motion_generator.Run();

      nh.deleteParam("ready");
    }
    else
    {
      cycle_motion_generator.setDesiredOrientation(msg);
      cycle_motion_generator.Run();
    }  
  }
}

