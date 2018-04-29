#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MoveToDesiredJoints.h"
#include <sstream>


int main(int argc, char **argv)
{
  // Ros initialization
  ros::init(argc, argv, "MoveToDesiredJoints");

  MoveToDesiredJoints::Mode mode;

  Eigen::VectorXd finalDesiredJoints;
  finalDesiredJoints.resize(NB_JOINTS);

  finalDesiredJoints.setConstant(0.0f);

  // Check if desired angles are specified with the command line
  if(argc == 9)
  {
    if(std::string(argv[1]) == "s")
    {
      mode = MoveToDesiredJoints::Mode::SIM;
    }  
    else if(std::string(argv[1]) == "r")
    {
      mode = MoveToDesiredJoints::Mode::REAL;
    }
    else
    {
      ROS_ERROR("Wrong input arguments, the first argument should be the mode either s(sim) or r(real) followed by 7 joint angles in degree");
      return 0;
    }

    for(int k = 0; k < NB_JOINTS; k++)
    {
      finalDesiredJoints(k) = atof(argv[k+2])*M_PI/180.0f;
    }
  }
  else
  {
    return 0;
  }

  ros::NodeHandle n;
  float frequency = 200.0f;

  MoveToDesiredJoints moveToDesiredJoints(n,frequency,mode);

  if (!moveToDesiredJoints.init()) 
  {
    return -1;
  }
  else
  {
    moveToDesiredJoints.setDesiredJoints(finalDesiredJoints);
    moveToDesiredJoints.run();
  }


  return 0;

}