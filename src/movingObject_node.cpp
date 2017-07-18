#include "ros/ros.h"
#include "MovingObject.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "movingObject");


  // Initialize object position
  Eigen::Vector3f initialPosition;
  initialPosition.setConstant(0.0f);

  // Check if initial position is specified with the command line
  if(argc == 4)
  {
    for(int k = 0; k < 3; k++)
    {
      initialPosition(k) = atof(argv[k+1]);
    }
  }

  ros::NodeHandle n;
  float frequency = 300.0f;

  MovingObject movingObject(n,frequency);

  if (!movingObject.init(initialPosition)) 
  {
    return -1;
  }
  else
  {
    movingObject.run();
  }

  return 0;

}

