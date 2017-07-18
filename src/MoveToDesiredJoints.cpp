#include "MoveToDesiredJoints.h"
// #include <tf/transform_datatypes.h>


MoveToDesiredJoints::MoveToDesiredJoints(ros::NodeHandle &n, float frequency, float jointTolerance):
	_n(n),
  _loopRate(frequency),
	_jointTolerance(jointTolerance)
{

	ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MoveToDesiredJoints::init() 
{

  // Set the number of joints
  _desiredJoints.data.resize(7);

  _currentJoints.position.resize(7);
  _currentJoints.velocity.resize(7);
  _currentJoints.effort.resize(7);

  // Initialize desired joints
  for(int k =0; k < 7; k++)
  {
    _desiredJoints.data[k] = 0.0f;
    _currentJoints.position[k] = 0.0f;
    _currentJoints.velocity[k] = 0.0f;
    _currentJoints.effort[k] = 0.0f;
  }

  _firstJointsUpdate = false;

  // Subscribe to joint states topic
  _subCurrentJoints = _n.subscribe("/lwr/joint_states", 10, &MoveToDesiredJoints::updateCurrentJoints,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publish to the joint position controller topic
  _pubDesiredJoints = _n.advertise<std_msgs::Float64MultiArray>("lwr/joint_controllers/command_joint_pos", 10);

	if (_n.ok())
	{ 
	  // Wait for callback to be called
		ros::spinOnce();
		ROS_INFO("The ros node is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void MoveToDesiredJoints::run() {

	while (_n.ok()) 
	{
		_pubDesiredJoints.publish(_desiredJoints);

		ros::spinOnce();

		_loopRate.sleep();

		if(checkJointsError() && _firstJointsUpdate)
		{
			ROS_INFO("The desired joints configuration is reached");
			break;
		}
	}
}


void MoveToDesiredJoints::setDesiredJoints(std_msgs::Float64MultiArray desiredJoints) 
{
	_desiredJoints = desiredJoints;
}


void MoveToDesiredJoints::updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg) 
{
	for(int k = 0; k < 7; k++)
	{
		_currentJoints.position[k] = msg->position[k];
	}
	if(!_firstJointsUpdate)
	{
		_firstJointsUpdate = true;
	}
}


bool MoveToDesiredJoints::checkJointsError() 
{
	_mutex.lock();

	// Check of all joint angles reached their respectives targets
	bool reached = true;
	for(int k = 0; k < 7; k++)
	{
    if(fabs(_currentJoints.position[k]-_desiredJoints.data[k])>_jointTolerance)
    {
      reached = false;
      break;
    }
	}

	_mutex.unlock();

	return reached;
}