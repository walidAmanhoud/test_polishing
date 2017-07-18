#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MoveToDesiredJoints.h"
#include <sstream>


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "MoveToDesiredJoints");

  std_msgs::Float64MultiArray desiredJoints;

  // Set the number of joints
  desiredJoints.data.resize(7);

  // Initialize desired joints
  for(int k = 0; k < 7; k++)
  {
    desiredJoints.data[k] = 0.0f;
  }

  // Check if desired angles are specified with the command line
  if(argc == 8)
  {
    for(int k = 0; k < 7; k++)
    {
      desiredJoints.data[k] = atof(argv[k+1])*M_PI/180.0f;
    }
  }

  ros::NodeHandle n;
  float frequency = 100.0f;

  MoveToDesiredJoints moveToDesiredJoints(n,frequency);

  if (!moveToDesiredJoints.init()) 
  {
    return -1;
  }
  else
  {
    moveToDesiredJoints.setDesiredJoints(desiredJoints);
    moveToDesiredJoints.run();
  }

  if(n.hasParam("readyForPolishing"))
  {
    n.setParam("readyForPolishing", true);
    ROS_INFO("Ready for polishing");
  }

  return 0;

}


// std_msgs::Float64MultiArray desiredJoints;
// bool reached;

// void readJointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
// {
//   reached = true;
//   for(int k = 0; k < 7; k++)
//   {
//     if(fabs(msg->position[k]-desiredJoints.data[k])>1e-3f)
//     {
//       reached = false;
//       break;
//     }
//   }
// }

// int main(int argc, char **argv)
// {
//   // Ros initialization
//   ros::init(argc, argv, "goToJoints");

//   // Set the number of joints
//   desiredJoints.data.resize(7);

//   // Initialize desired joints
//   for(int k =0; k < 7; k++)
//   {
//     desiredJoints.data[k] = 0.0f;
//   }

//   // Check if desired angles are specified with the command line
//   if(argc == 8)
//   {
//     for(int k = 0; k < 7; k++)
//     {
//       desiredJoints.data[k] = atof(argv[k+1])*M_PI/180.0f;
//     }
//   }
//   else
//   {
//     std::cerr << "Send default desired joint configuration" << std::endl;
//   }

//   ros::NodeHandle n;

//   // Subscribe to joint states topic
//   ros::Subscriber sub = n.subscribe("/lwr/joint_states", 10, &readJointsCallback);

//   // Publish to the joint position controller topic
//   ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("lwr/joint_controllers/command_joint_pos", 10);

//   // Node frequency 
//   ros::Rate loop_rate(100);
   
//   int count = 0;

//   // Loop until user stop or target reached
//   while (n.ok())
//   {
//     pub.publish(desiredJoints);

//     ros::spinOnce();

//     loop_rate.sleep();
//     ++count;

//     if(reached)
//     {
//       std::cerr << "Desired joint angles reached" << std::endl;
//       break;
//     }
//   }

//   return 0;
// }
