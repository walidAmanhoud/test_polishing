#include "ObjectStateManager.h"
// #include <tf/transform_datatypes.h>


ObjectStateManager::ObjectStateManager(ros::NodeHandle &n, float frequency):
	_n(n),
  _loopRate(frequency),
  _dt(1.0f/frequency)
{

	ROS_INFO_STREAM("The move to desired joints node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool ObjectStateManager::init() 
{

	// Initialize workspace shape
	_workspaceRadius = 0.7f;
	_robotWorkspaceCenter << 0.0f, 0.0f, 0.3105f;

	// Initialize object position and speed
	_measuredRobotPosition.setConstant(0.0f);
	_measuredObjectPosition.setConstant(0.0f);
	_estimatedObjectPosition.setConstant(0.0f);
	_estimatedObjectSpeed.setConstant(0.0f);

	_firstObjectPoseMeasured = false;
	_firstRobotPoseMeasured = false;

	// _convoyerCenter << -0.4f,0.0f,0.2f;
	// _convoyerLength = 2.0f;
	// _convoyerWidth = 0.3f;

	// Initialize messages
	_objectStateMsg.data.resize(6);
	for(int k = 0; k < 3; k++)
	{
		_objectStateMsg.data[k] = _estimatedObjectPosition(k);
		_objectStateMsg.data[k+3] = _estimatedObjectSpeed(k);
	}

	_objectPositionMsg.header.frame_id = "world"; 
	_objectPositionMsg.header.stamp = ros::Time::now();
	_objectPositionMsg.point.x = _estimatedObjectPosition(0);
	_objectPositionMsg.point.y = _estimatedObjectPosition(1);
	_objectPositionMsg.point.z = _estimatedObjectPosition(2);

	// _markerMsg.header.frame_id = "world";
	// _markerMsg.header.stamp = ros::Time();
	// _markerMsg.ns = "marker_test_triangle_list";
	// _markerMsg.id = 0;
	// _markerMsg.type = visualization_msgs::Marker::TRIANGLE_LIST;
	// _markerMsg.action = visualization_msgs::Marker::ADD;
	// _markerMsg.pose.position.x = 0.0;
	// _markerMsg.pose.position.y = 0.0;
	// _markerMsg.pose.position.z = 0.0;
	// _markerMsg.pose.orientation.x = 0.0;
	// _markerMsg.pose.orientation.y = 0.0;
	// _markerMsg.pose.orientation.z = 0.0;
	// _markerMsg.pose.orientation.w = 1.0;
	// _markerMsg.scale.x = 1.0;
	// _markerMsg.scale.y = 1.0;
	// _markerMsg.scale.z = 1.0;
	// _markerMsg.color.a = 1.0;
	// // _markerMsg.color.r = 0.0;
	// // _markerMsg.color.g = 1.0;
	// // _markerMsg.color.b = 0.0;

 // 	geometry_msgs::Point p1,p2,p3,p4,p5,p6;
 // 	p1.x = _convoyerCenter(0)+_convoyerWidth/2.0f;
	// p1.y = _convoyerCenter(1)-_convoyerLength/2.0f;
	// p1.z = _convoyerCenter(2);
 // 	p2.x = _convoyerCenter(0)-_convoyerWidth/2.0f;
	// p2.y = _convoyerCenter(1)-_convoyerLength/2.0f;
	// p2.z = _convoyerCenter(2);
 // 	p3.x = _convoyerCenter(0)-_convoyerWidth/2.0f;
	// p3.y = _convoyerCenter(1)+_convoyerLength/2.0f;
	// p3.z = _convoyerCenter(2);
 // 	p4.x = _convoyerCenter(0)-_convoyerWidth/2.0f;
	// p4.y = _convoyerCenter(1)+_convoyerLength/2.0f;
	// p4.z = _convoyerCenter(2);
 // 	p5.x = _convoyerCenter(0)+_convoyerWidth/2.0f;
	// p5.y = _convoyerCenter(1)+_convoyerLength/2.0f;
	// p5.z = _convoyerCenter(2);
 // 	p6.x = _convoyerCenter(0)+_convoyerWidth/2.0f;
	// p6.y = _convoyerCenter(1)-_convoyerLength/2.0f;
	// p6.z = _convoyerCenter(2);

	// std_msgs::ColorRGBA c;
	// c.r = 0.7;
	// c.g = 0.7;
	// c.b = 0.7;
	// c.a = 1.0;

	// for(int k = 0; k < 6; k++)
	// {
	// 	_markerMsg.colors.push_back(c);
	// }

	// _markerMsg.points.push_back(p1);
	// _markerMsg.points.push_back(p2);
	// _markerMsg.points.push_back(p3);
	// _markerMsg.points.push_back(p4);
	// _markerMsg.points.push_back(p5);
	// _markerMsg.points.push_back(p6);

  // Subscribe to object speed topic
  _subOptitrackRobotPoseStamped = _n.subscribe("/optitrack/robot/pose", 1, &ObjectStateManager::updateRobotMeasuredPose,this,ros::TransportHints().reliable().tcpNoDelay());
  _subOptitrackObjectPoseStamped = _n.subscribe("/optitrack/object/pose", 1, &ObjectStateManager::updateObjectMeasuredPose,this,ros::TransportHints().reliable().tcpNoDelay());

  // Publish object state to the ds motion generator
  _pubObjectState = _n.advertise<std_msgs::Float64MultiArray>("test_polishing/object_state", 10);

  // Publish object position for visualization
  _pubObjectPosition = _n.advertise<geometry_msgs::PointStamped>("test_polishing/object_position", 1);
  // _pubMarker = _n.advertise<visualization_msgs::Marker>("test_polishing/plane", 1);

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



void ObjectStateManager::run() 
{

	while (_n.ok()) 
	{

		if(_firstObjectPoseMeasured && _firstRobotPoseMeasured)
		{
			updateObjectPose();

			_pubObjectState.publish(_objectStateMsg);
			_pubObjectPosition.publish(_objectPositionMsg);
			// _pubMarker.publish(_markerMsg);		
		}

		ros::spinOnce();

		_loopRate.sleep();

	}
}

void ObjectStateManager::updateObjectPose() 
{

	_mutex.lock();

	Eigen::Vector3f previousPosition = _estimatedObjectPosition;
	_estimatedObjectPosition = _measuredObjectPosition;
	_estimatedObjectSpeed.setConstant(0.0f);

	if(!isReachable())
	{
		_estimatedObjectPosition = previousPosition;
		_estimatedObjectSpeed.setConstant(0.0f);
		ROS_INFO("Object not reachable");

	}
	else
	{
		// Update messages contents
		for(int k = 0; k < 3; k++)
		{
			_objectStateMsg.data[k] = _estimatedObjectPosition(k);
			_objectStateMsg.data[k+3] = _estimatedObjectSpeed(k);
		}
		
	}

	_objectPositionMsg.header.stamp = ros::Time::now();
	_objectPositionMsg.point.x = _estimatedObjectPosition(0);
	_objectPositionMsg.point.y = _estimatedObjectPosition(1);
	_objectPositionMsg.point.z = _estimatedObjectPosition(2);

	_mutex.unlock();
}

void ObjectStateManager::updateRobotMeasuredPose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
	Eigen::Vector3f temp;
	temp << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

	_measuredRobotPosition = flipAxis(temp);

	if(!_firstRobotPoseMeasured)
	{
		_firstRobotPoseMeasured = true;
	}
}


void ObjectStateManager::updateObjectMeasuredPose(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
	Eigen::Vector3f temp;
	temp << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

	_measuredObjectPosition = flipAxis(temp)-_measuredRobotPosition;
	_measuredObjectPosition(2) -= -0.02;
	_measuredObjectPosition(2) += 0.15;

	if(!_firstObjectPoseMeasured)
	{
		if(_firstRobotPoseMeasured)
		{
			_firstObjectPoseMeasured = true;
		}
	}

}


Eigen::Vector3f ObjectStateManager::flipAxis(Eigen::Vector3f input)
{
	Eigen::Vector3f output;
	output << -input(1), input(0), input(2);

	return output;
}


bool ObjectStateManager::isReachable() 
{
	Eigen::Vector3f distance = _estimatedObjectPosition-_robotWorkspaceCenter;

	if(distance.norm()<_workspaceRadius)
	{
		return true;
	}
	else
	{
		std::cerr << distance.norm() << std::endl;
		return false;
	}
}