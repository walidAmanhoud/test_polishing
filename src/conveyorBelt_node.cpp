#include "ros/ros.h"
#include "ConveyorBeltController.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "conveyorBelt");

  ros::NodeHandle n;
  float frequency = 300.0f;

  ConveyorBeltController conv(n,frequency);

  int mode, speed, acceleration, decceleration;

  if(argc == 5)
  {
    mode = atoi(argv[1]);
   
    if(mode < 0 || mode > 2)
    {
      mode = 0;
    }

    speed = atoi(argv[2]);

    if(speed < 40)
    {
        speed = 40;
    }
    else if(speed > 1500)
    {
        speed = 1500;
    }

    acceleration = atoi(argv[3]);

    if(acceleration < 10)
    {
        acceleration = 10;
    }
    else if(acceleration > 600)
    {
        acceleration = 600;
    }

    decceleration = atoi(argv[4]);

    if(decceleration < 10)
    {
        decceleration = 10;
    }
    else if(decceleration > 600)
    {
        decceleration = 600;
    }
  }

  if (!conv.init()) 
  {
    return -1;
  }
  else
  {
    if(argc == 5)
    {
      ROS_INFO("Use provided config");  
      conv.setDesiredConfig(mode,speed,acceleration,decceleration);
    }

    conv.run();
  }

  return 0;

}