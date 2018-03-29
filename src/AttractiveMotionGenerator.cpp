#include "AttractiveMotionGenerator.h"
// #include <tf/transform_datatypes.h>


AttractiveMotionGenerator::AttractiveMotionGenerator(ros::NodeHandle &n, double frequency, std::string inputTopicName, std::string outputTopicName, double convergenceRate)
	: _n(n),
	  _loopRate(frequency),
	  inputTopicName(inputTopicName),
	  outputTopicName(outputTopicName),
	  _convergenceRate(convergenceRate),
	  _dt(1 / frequency),
	  _velocityLimit(0)

{

	ROS_INFO_STREAM("Motion generator node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz");
}


bool AttractiveMotionGenerator::init() {

	_realPose.Resize(3);
	_realPose.Zero();
	_desiredPosition.Resize(3);
	_desiredPosition.Zero();
	_desiredVelocity.Resize(3);
	_desiredVelocity.Zero();

	_attractorPosition.Resize(3);
	_attractorPosition.Zero();
	_attractorSpeed.Resize(3);
	_attractorSpeed.Zero();

	_attractorOffset.Resize(3);
	_attractorOffset.Zero();

	_endEffectorForces.Resize(3);
	_endEffectorForces.Zero();

	_pidAttractorOffset.Resize(3);
	_pidAttractorOffset.Zero();

	if (_convergenceRate < 0) 
	{
		ROS_ERROR("The convergence rate cannot be negative!");
		return false;
	}

	_startThread = true;

    if(pthread_create(&_thread, NULL, &AttractiveMotionGenerator::startPathPublishingLoop, this))
    {
        throw std::runtime_error("Cannot create reception thread");  
    }

	if (!InitializeROS()) 
	{
		ROS_ERROR_STREAM("ERROR intializing the DS");
		return false;
	}

	return true;
}



bool AttractiveMotionGenerator::InitializeROS() {

	_subRealPose = _n.subscribe( inputTopicName , 1, &AttractiveMotionGenerator::updateRealPosition, this, ros::TransportHints().reliable().tcpNoDelay());
	_subAttractorState = _n.subscribe("test_polishing/object_state", 1000, &AttractiveMotionGenerator::updateAttractorState, this, ros::TransportHints().reliable().tcpNoDelay());
	_subEndEffectorFt = _n.subscribe("lwr/ee_ft",1,&AttractiveMotionGenerator::updateEndEffectorFt, this, ros::TransportHints().reliable().tcpNoDelay());

	// if(_outputVelocity)
	// {
	// }
	// else
	// {
	// 	_pubDesiredTwist = _n.advertise<geometry_msgs::Pose>(outputTopicName, 1);
		
	// }

	_pubDesiredTwist = _n.advertise<geometry_msgs::Twist>(outputTopicName, 1);
	_pubDesiredOrientation = _n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);
	_pubAttractorPosition = _n.advertise<geometry_msgs::PointStamped>("DS/attractor", 1);
	_pubDesiredPath = _n.advertise<nav_msgs::Path>("DS/desiredPath", 1);

	///////

	_msgDesiredPath.poses.resize(MAX_FRAME);

	_dynRecCallback = boost::bind(&AttractiveMotionGenerator::dynamicReconfigureCallback, this, _1, _2);
	_dynRecServer.setCallback(_dynRecCallback);
	// _dynRecServer.getConfigDefault(_config);



	if (_n.ok()) 
	{ // Wait for poses being published
		ros::spinOnce();
		ROS_INFO("The Motion generator is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}

}

void AttractiveMotionGenerator::setDesiredOrientation(geometry_msgs::Quaternion msg) 
{
	_msgDesiredOrientation = msg;
}


void AttractiveMotionGenerator::setAttractorPosition(float px, float py, float pz) 
{
	_attractorPosition(0) = px;
	_attractorPosition(1) = py;
	_attractorPosition(2) = pz;
}



void AttractiveMotionGenerator::run()
{

	while (_n.ok()) {

		if(_firstRealPoseReceived && _firstAttractorStateReceived)
		{
			computeDesiredVelocity();

			publishData();
		}

		ros::spinOnce();

		_loopRate.sleep();
	}

	_startThread = false;
	pthread_join(_thread,NULL);

}

void AttractiveMotionGenerator::updateRealPosition(const geometry_msgs::Pose::ConstPtr& msg)
{

	_msgRealPose = *msg;

	_realPose(0) = _msgRealPose.position.x;
	_realPose(1) = _msgRealPose.position.y;
	_realPose(2) = _msgRealPose.position.z;


	if(!_firstRealPoseReceived)
	{
		_desiredPosition = _realPose;
		std:cerr << _desiredPosition(2) << " " <<  _realPose(2) << std::endl;

		_firstRealPoseReceived = true;
	}
}

void AttractiveMotionGenerator::updateEndEffectorFt(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{

	_msgEndEffectorFt = *msg;

	_endEffectorForces(0) = _msgEndEffectorFt.wrench.force.x;
	_endEffectorForces(1) = _msgEndEffectorFt.wrench.force.y;
	_endEffectorForces(2) = _msgEndEffectorFt.wrench.force.z;

}



void AttractiveMotionGenerator::updateAttractorState(const std_msgs::Float64MultiArray::ConstPtr& msg) 
{

	_attractorPosition(0) = msg->data[0];
	_attractorPosition(1) = msg->data[1];
	_attractorPosition(2) = msg->data[2];
	_attractorSpeed(0) = msg->data[3];
	_attractorSpeed(1) = msg->data[4];
	_attractorSpeed(2) = msg->data[5];

	if(!_firstAttractorStateReceived)
	{
		_firstAttractorStateReceived= true;
	}
}


void AttractiveMotionGenerator::computeDesiredVelocity() 
{
	mutex_.lock();

	MathLib::Vector error;
	error.Resize(3);
	error = _realPose-_attractorPosition;


	if(_usePid)

	{
		if(error.Norm() < 0.02)
		{
			if(!_firstContact)
			{
				_firstContact = true;
				ROS_INFO("Target touched");
			}	
		}

		if(_firstContact)
		{
			_pidError = -(_targetForce-_endEffectorForces(2));
			_pidInteg += _dt*_pidError;
			_up = _kp*_pidError;
			_ui = _ki*_pidInteg;

			if(_ui < -0.3)
			{
				_ui = -0.3f;
			}

			float command = _up+_ui;

			if(command<-0.3f)
			{
				command = -0.3f;
			}
			_pidAttractorOffset(2) = command;
			std::cerr << "command: " << command << " up " << _up << " ui " << _ui << std::endl;
		}
	}
	else
	{
		_pidAttractorOffset(2) = 0.0f;
		_pidInteg = 0.0f;
	}

	// std::cerr << _attractorOffset(0) << " " << _attractorOffset(1) << " " << _attractorOffset(2) << " " << std::endl;
	for(int k = 0; k < 3; k++)
	{
		// _desiredVelocity(k) = -_convergenceScale*_convergenceRate*(_realPose(k)-_attractorPosition(k)-_attractorOffset(k))+_attractorSpeed(k);
		_desiredVelocity(k) = -_convergenceScale*_convergenceRate*(_realPose(k)-_attractorPosition(k)-_attractorOffset(k)-_pidAttractorOffset(k))+_attractorSpeed(k);
	}

	if (std::isnan(_desiredVelocity.Norm2())) 
	{
		ROS_WARN_THROTTLE(1, "DS is generating NaN. Setting the output to zero.");
		_desiredVelocity.Zero();
	}


	if (_desiredVelocity.Norm() > _velocityLimit) 
	{
		_desiredVelocity = _desiredVelocity / _desiredVelocity.Norm() * _velocityLimit;
	}

	// std::cerr << _desiredVelocity(0) << " " << _desiredVelocity(1) << " " << _desiredVelocity(2) << std::endl; 
	for(int k = 0; k < 3; k++)
	{
		_desiredPosition(k) = _realPose(k)+_dt*_desiredVelocity(k);
	}

	_msgDesiredTwist.linear.x  = _desiredVelocity(0);
	_msgDesiredTwist.linear.y  = _desiredVelocity(1);
	_msgDesiredTwist.linear.z  = _desiredVelocity(2);

	_msgDesiredTwist.angular.x = 0.0f;
	_msgDesiredTwist.angular.y = 0.0f;
	_msgDesiredTwist.angular.z = 0.0f;

	_msgDesiredPose.position.x = _desiredPosition(0);
	_msgDesiredPose.position.y = _desiredPosition(1);
	_msgDesiredPose.position.z = _desiredPosition(2);
	_msgDesiredPose.orientation.x = _msgDesiredOrientation.x;
	_msgDesiredPose.orientation.y = _msgDesiredOrientation.y;
	_msgDesiredPose.orientation.z = _msgDesiredOrientation.z;
	_msgDesiredPose.orientation.w = _msgDesiredOrientation.w;

	mutex_.unlock();

}




void AttractiveMotionGenerator::publishData() {

	// if(_outputVelocity)
	// {
		_pubDesiredTwist.publish(_msgDesiredTwist);
		_pubDesiredOrientation.publish(_msgDesiredOrientation);
	// }
	// else
	// {
	// 	_pubDesiredTwist.publish(_msgDesiredPose);
	// }
}


void AttractiveMotionGenerator::dynamicReconfigureCallback(test_polishing::attractive_paramsConfig &config, uint32_t level) {

	ROS_INFO("Reconfigure request. Updatig the parameters ...");

	_convergenceScale = config.convergenceScale;
	_velocityLimit = config.velocityLimit;

	_attractorOffset(0) = config.attractorOffsetX;
	_attractorOffset(1) = config.attractorOffsetY;
	_attractorOffset(2) = config.attractorOffsetZ;

	_kp = config.kp;
	_ki = config.ki;
	_usePid = config.usePid;
	_targetForce = config.targetForce;

	if (_convergenceRate < 0)
	{
		ROS_ERROR("RECONFIGURE: The scaling factor for convergence rate cannot be negative!");
	}

	if (_velocityLimit < 0) {
		ROS_ERROR("RECONFIGURE: The limit for velocity cannot be negative!");
	}

}

void* AttractiveMotionGenerator::startPathPublishingLoop(void* ptr)
{
    reinterpret_cast<AttractiveMotionGenerator *>(ptr)->pathPublishingLoop(); 
}


void AttractiveMotionGenerator::pathPublishingLoop()
{
    while(_startThread)
    {
        if(_firstRealPoseReceived)
        {
            publishFuturePath();   
        }
    }
    std::cerr << "END path publishing thread" << std::endl;
}

void AttractiveMotionGenerator::publishFuturePath() {

	geometry_msgs::PointStamped msg;

	msg.header.frame_id = "world";
	msg.header.stamp = ros::Time::now();
	msg.point.x = _attractorPosition(0) + _attractorOffset(0);
	msg.point.y = _attractorPosition(1) + _attractorOffset(1);
	// msg.point.z = _attractorPosition(2) + _attractorOffset(2);
	msg.point.z = _attractorPosition(2) + _attractorOffset(2) + _pidAttractorOffset(2);

	_pubAttractorPosition.publish(msg);

	// setting the header of the path
	_msgDesiredPath.header.stamp = ros::Time::now();
	_msgDesiredPath.header.frame_id = "world";

	MathLib::Vector simulatedPosition = _realPose;
	MathLib::Vector simulatedVelocity;
	simulatedVelocity.Resize(3);

	for (int frame = 0; frame < MAX_FRAME; frame++)
	{

		for(int k = 0; k < 3; k++)
		{
			simulatedVelocity(k) = -_convergenceScale*_convergenceRate*(simulatedPosition(k)-_attractorPosition(k)-_attractorOffset(k)-_pidAttractorOffset(k))+_attractorSpeed(k);
			// simulatedVelocity(k) = -_convergenceScale*_convergenceRate*(simulatedPosition(k)-_attractorPosition(k)-_attractorOffset(k))+_attractorSpeed(k);
		}

		if (simulatedVelocity.Norm() > _velocityLimit) 
		{
			simulatedVelocity = simulatedVelocity / simulatedVelocity.Norm() * _velocityLimit;
		}


		for(int k = 0; k < 3; k++)
		{
			simulatedPosition(k) += _dt*simulatedVelocity(k)*20;
		}

		_msgDesiredPath.poses[frame].header.stamp = ros::Time::now();
		_msgDesiredPath.poses[frame].header.frame_id = "world";
		_msgDesiredPath.poses[frame].pose.position.x = simulatedPosition(0);
		_msgDesiredPath.poses[frame].pose.position.y = simulatedPosition(1);
		_msgDesiredPath.poses[frame].pose.position.z = simulatedPosition(2);

		_pubDesiredPath.publish(_msgDesiredPath);


	}


}
