#include "MoveToDesiredJoints.h"
// #include <tf/transform_datatypes.h>


MoveToDesiredJoints::MoveToDesiredJoints(ros::NodeHandle &n, float frequency, Mode mode):
	_n(n),
  _loopRate(frequency),
  _mode(mode),
  _dt(1.0f/frequency),
  _filter(NB_JOINTS,1e-6,1)
{

	ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MoveToDesiredJoints::init() 
{

  // Set the number of joints
  _desiredJointsSimMsg.data.resize(NB_JOINTS);
  _desiredJointsRealMsg.position.resize(NB_JOINTS);
  _desiredJointsRealMsg.velocity.resize(NB_JOINTS);
  _desiredJointsRealMsg.effort.resize(NB_JOINTS);
  _desiredJointsRealMsg.stiffness.resize(NB_JOINTS);

	_currentJoints.resize(NB_JOINTS);
	_desiredJoints.resize(NB_JOINTS);
	_finalDesiredJoints.resize(NB_JOINTS);

  _currentJoints.setConstant(0.0f);
  _desiredJoints.setConstant(0.0f);
  _finalDesiredJoints.setConstant(0.0f);

  // Initialize desired joints
  for(int k =0; k < NB_JOINTS; k++)
  {
    _desiredJointsSimMsg.data[k] = 0.0f;
    _desiredJointsRealMsg.position[k] = 0.0f;
    _desiredJointsRealMsg.stiffness[k] = 0.0f;
    _desiredJointsRealMsg.velocity[k] = 0.0f;
    _desiredJointsRealMsg.effort[k] = 0.0f;

  }

  _firstJointsUpdate = false;

  motion::Vector velocityLimits(NB_JOINTS);

  for(int k = 0; k < NB_JOINTS; k++)
  {
      velocityLimits(k)  = 0.25f; // x ms^-1
  }
  _filter.SetVelocityLimits(velocityLimits);


  // Subscribe to joint states topic

  if(_mode==SIM)
  {
  	// Publish to the joint position controller topic
  	_subCurrentJoints = _n.subscribe("/r_arm_pos_controller/joint_states", 1, &MoveToDesiredJoints::updateCurrentJoints,this,ros::TransportHints().reliable().tcpNoDelay());
  	_pubDesiredJoints = _n.advertise<std_msgs::Float32MultiArray>("/r_arm_pos_controller/command", 1); 	
  }
  else if(_mode == REAL)
  {
  	_subCurrentJoints = _n.subscribe("/real_r_arm_pos_controller/joint_states", 1, &MoveToDesiredJoints::updateCurrentJoints,this,ros::TransportHints().reliable().tcpNoDelay());
  	_pubDesiredJoints = _n.advertise<kuka_fri_bridge::JointStateImpedance>("/real_r_arm_controller/joint_imp_cmd", 1);
  }
  else
  {
  	ROS_ERROR("Mode not recognized");
  	return false;
  }

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


void MoveToDesiredJoints::run() 
{

	while (_n.ok()) 
	{
		if(_firstJointsUpdate)
		{
			_mutex.lock();

			computeDesiredJoints();

			publishData();


			if(checkJointsError())
			{
				ROS_INFO("The desired joints configuration is reached");
				break;
			}

			_mutex.unlock();
		}
			
		ros::spinOnce();
		_loopRate.sleep();
	}

}


void MoveToDesiredJoints::computeDesiredJoints()
{
  _filter.SetDt(_dt);
  _filter.SetTarget(_finalDesiredJoints);
  _filter.Update();
  _filter.GetState(_desiredJoints);
}


void MoveToDesiredJoints::publishData()
{
	if(_mode == SIM)
	{
		for(int k = 0; k < NB_JOINTS; k++)
		{
			_desiredJointsSimMsg.data[k] = _desiredJoints(k);
		}
		_pubDesiredJoints.publish(_desiredJointsSimMsg);
	}
	else if(_mode == REAL)
	{
		for(int k = 0; k < NB_JOINTS; k++)
		{
			_desiredJointsRealMsg.position[k] = _desiredJoints(k);
		}
		_pubDesiredJoints.publish(_desiredJointsRealMsg);
	}
}


void MoveToDesiredJoints::setDesiredJoints(Eigen::VectorXd finalDesiredJoints) 
{
	_finalDesiredJoints = finalDesiredJoints;
}


void MoveToDesiredJoints::updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg) 
{
	for(int k = 0; k < NB_JOINTS; k++)
	{
		_currentJoints(k) = msg->position[k];
	}
	if(!_firstJointsUpdate)
	{
		_firstJointsUpdate = true;
		_filter.SetState(_currentJoints.cast<double>());
	}
}


bool MoveToDesiredJoints::checkJointsError() 
{
	// Check of all joint angles reached their respectives targets
	bool reached = true;

	float error = (_currentJoints-_finalDesiredJoints).array().abs().maxCoeff();

  if(fabs(error)>JOINT_TOLERANCE)
  {
    reached = false;
  }

	return reached;
}