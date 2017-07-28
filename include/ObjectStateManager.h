#ifndef __OBJECT_STATE_MANAGER__
#define __OBJECT_STATE_MANAGER__

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <mutex>
#include <Eigen/Eigen>

#include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"

class ObjectStateManager 
{

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subOptitrackObjectPoseStamped;
		ros::Subscriber _subOptitrackRobotPoseStamped;
		ros::Publisher _pubObjectState;
		ros::Publisher _pubObjectPosition;
		ros::Publisher _pubMarker;

		// Node variables
		Eigen::Vector3f _measuredRobotPosition;
		Eigen::Vector3f _measuredObjectPosition;
		Eigen::Vector3f _estimatedObjectPosition;
		Eigen::Vector3f _estimatedObjectSpeed;
		Eigen::Vector3f _robotWorkspaceCenter;
		float _workspaceRadius;
		
		Eigen::Vector3f _convoyerCenter;
		float _convoyerLength;
		float _convoyerWidth;

		std_msgs::Float64MultiArray _objectStateMsg;
		geometry_msgs::PointStamped _objectPositionMsg;
		visualization_msgs::Marker _markerMsg;

		bool _firstObjectPoseMeasured;
		bool _firstRobotPoseMeasured;

		float _dt;

		// Class variables
		std::mutex _mutex;


	public:

		ObjectStateManager(ros::NodeHandle &n, float frequency);

		// Initialize node
		bool init();

		// Run node main loop
		void run();


	private:

		void updateObjectPose();

		Eigen::Vector3f flipAxis(Eigen::Vector3f input); 

		bool isReachable(); 

		void updateObjectMeasuredPose(const geometry_msgs::PoseStamped::ConstPtr& msg); 

		void updateRobotMeasuredPose(const geometry_msgs::PoseStamped::ConstPtr& msg); 

};


#endif
