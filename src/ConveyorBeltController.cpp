#include "ConveyorBeltController.h"
// #include <tf/transform_datatypes.h>


ConveyorBeltController::ConveyorBeltController(ros::NodeHandle &n, float frequency):
	_n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency)
{

	ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool ConveyorBeltController::init(Eigen::Vector3f initialPosition) 
{

	// Initialize workspace shape
	_workspaceRadius = 0.7f;
	_workspaceCenter << 0.0f, 0.0f, 0.3105f;

	// Initialize object position and speed
	_position = initialPosition;
	_speed.setConstant(0.0f);

	_convoyerCenter << -0.4f,0.0f,0.2f;
	_convoyerLength = 2.0f;
	_convoyerWidth = 0.3f;

	// Initialize messages
	_objectStateMsg.data.resize(6);
	for(int k = 0; k < 3; k++)
	{
		_objectStateMsg.data[k] = _position(k);
		_objectStateMsg.data[k+3] = _speed(k);
	}

	_objectPositionMsg.header.frame_id = "world"; 
	_objectPositionMsg.header.stamp = ros::Time::now();
	_objectPositionMsg.point.x = _position(0);
	_objectPositionMsg.point.y = _position(1);
	_objectPositionMsg.point.z = _position(2);

	_markerMsg.header.frame_id = "world";
	_markerMsg.header.stamp = ros::Time();
	_markerMsg.ns = "marker_test_triangle_list";
	_markerMsg.id = 0;
	_markerMsg.type = visualization_msgs::Marker::TRIANGLE_LIST;
	_markerMsg.action = visualization_msgs::Marker::ADD;
	_markerMsg.pose.position.x = 0.0;
	_markerMsg.pose.position.y = 0.0;
	_markerMsg.pose.position.z = 0.0;
	_markerMsg.pose.orientation.x = 0.0;
	_markerMsg.pose.orientation.y = 0.0;
	_markerMsg.pose.orientation.z = 0.0;
	_markerMsg.pose.orientation.w = 1.0;
	_markerMsg.scale.x = 1.0;
	_markerMsg.scale.y = 1.0;
	_markerMsg.scale.z = 1.0;
	_markerMsg.color.a = 1.0;
	// _markerMsg.color.r = 0.0;
	// _markerMsg.color.g = 1.0;
	// _markerMsg.color.b = 0.0;

 	geometry_msgs::Point p1,p2,p3,p4,p5,p6;
 	p1.x = _convoyerCenter(0)+_convoyerWidth/2.0f;
	p1.y = _convoyerCenter(1)-_convoyerLength/2.0f;
	p1.z = _convoyerCenter(2);
 	p2.x = _convoyerCenter(0)-_convoyerWidth/2.0f;
	p2.y = _convoyerCenter(1)-_convoyerLength/2.0f;
	p2.z = _convoyerCenter(2);
 	p3.x = _convoyerCenter(0)-_convoyerWidth/2.0f;
	p3.y = _convoyerCenter(1)+_convoyerLength/2.0f;
	p3.z = _convoyerCenter(2);
 	p4.x = _convoyerCenter(0)-_convoyerWidth/2.0f;
	p4.y = _convoyerCenter(1)+_convoyerLength/2.0f;
	p4.z = _convoyerCenter(2);
 	p5.x = _convoyerCenter(0)+_convoyerWidth/2.0f;
	p5.y = _convoyerCenter(1)+_convoyerLength/2.0f;
	p5.z = _convoyerCenter(2);
 	p6.x = _convoyerCenter(0)+_convoyerWidth/2.0f;
	p6.y = _convoyerCenter(1)-_convoyerLength/2.0f;
	p6.z = _convoyerCenter(2);

	std_msgs::ColorRGBA c;
	c.r = 0.7;
	c.g = 0.7;
	c.b = 0.7;
	c.a = 1.0;

	for(int k = 0; k < 6; k++)
	{
		_markerMsg.colors.push_back(c);
	}

	_markerMsg.points.push_back(p1);
	_markerMsg.points.push_back(p2);
	_markerMsg.points.push_back(p3);
	_markerMsg.points.push_back(p4);
	_markerMsg.points.push_back(p5);
	_markerMsg.points.push_back(p6);

  // Subscribe to object speed topic
  // _subObjectSpeed = _n.subscribe("/lwr/joint_states", 10, &ConveyorBeltController::updateCurrentJoints,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publish object state to the ds motion generator
  _pubObjectState = _n.advertise<std_msgs::Float64MultiArray>("test_polishing/object_state", 10);

  // Publish object position for visualization
  _pubObjectPosition = _n.advertise<geometry_msgs::PointStamped>("test_polishing/object_position", 1);
  _pubMarker = _n.advertise<visualization_msgs::Marker>("test_polishing/plane", 1);

  _dynRecCallback = boost::bind(&ConveyorBeltController::dynamicReconfigureCallback,this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);

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



void ConveyorBeltController::run() {

	while (_n.ok()) 
	{

		updateObjectPosition();

		_pubObjectState.publish(_objectStateMsg);
		_pubObjectPosition.publish(_objectPositionMsg);
		_pubMarker.publish(_markerMsg);

		ros::spinOnce();

		_loopRate.sleep();

	}
}


void ConveyorBeltController::updateObjectPosition() 
{

	_mutex.lock();

	Eigen::Vector3f previousPosition = _position;

	// Update position
	_position += _dt*_speed;

	if(!isReachable())
	{
		_position = previousPosition;
		_speed.setConstant(0.0f);

	}
	else
	{
		// Update messages contents
		for(int k = 0; k < 3; k++)
		{
			_objectStateMsg.data[k] = _position(k);
			_objectStateMsg.data[k+3] = _speed(k);
		}
		
	}


	_objectPositionMsg.header.stamp = ros::Time::now();
	_objectPositionMsg.point.x = _position(0);
	_objectPositionMsg.point.y = _position(1);
	_objectPositionMsg.point.z = _position(2);

	_mutex.unlock();
}


bool ConveyorBeltController::isReachable() 
{
	Eigen::Vector3f distance = _position-_workspaceCenter;

	if(distance.norm()<_workspaceRadius)
	{
		return true;
	}
	else
	{
		return false;
	}
}


void ConveyorBeltController::dynamicReconfigureCallback(test_polishing::object_paramsConfig &config, uint32_t level) 
{

	ROS_INFO("Reconfigure request. Updatig the parameters ...");
	
	_speed(0) = config.vx;
	_speed(1) = config.vy;
	_speed(2) = config.vz;

}