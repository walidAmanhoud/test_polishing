#include "ros/ros.h"
#include "ObjectStateManager.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "objectStateManager");


  ros::NodeHandle n;
  float frequency = 300.0f;

  ObjectStateManager objectStateManager(n,frequency);

  if (!objectStateManager.init()) 
  {
    return -1;
  }
  else
  {
    objectStateManager.run();
  }

  return 0;

}


