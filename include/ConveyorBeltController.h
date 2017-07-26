#ifndef __CONVOYER_CONTROLLER_H__
#define __CONVOYER_CONTROLLER_H__

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <mutex>
#include <Eigen/Eigen>

#include <dynamic_reconfigure/server.h>
#include <test_polishing/object_paramsConfig.h>

#include "visualization_msgs/Marker.h"
// #include "visualization_msgs/MarkerArray.h"

class MovingObject 
{

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subObjectSpeed;
		ros::Publisher _pubObjectState;
		ros::Publisher _pubObjectPosition;
		ros::Publisher _pubMarker;

		// Node variables
		Eigen::Vector3f _position;
		Eigen::Vector3f _speed;
		Eigen::Vector3f _workspaceCenter;
		float _workspaceRadius;
		
		Eigen::Vector3f _convoyerCenter;
		float _convoyerLength;
		float _convoyerWidth;

		std_msgs::Float64MultiArray _objectStateMsg;
		geometry_msgs::PointStamped _objectPositionMsg;
		visualization_msgs::Marker _markerMsg;

		float _dt;

		// Class variables
		std::mutex _mutex;

		// Dynamic reconfigure definition (server+callback)
		dynamic_reconfigure::Server<test_polishing::object_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<test_polishing::object_paramsConfig>::CallbackType _dynRecCallback;


	public:

		MovingObject(ros::NodeHandle &n, float frequency);

		// Initialize node
		bool init(Eigen::Vector3f initialPosition);

		// Run node main loop
		void run();


	private:

		void updateObjectPosition(); 

		void dynamicReconfigureCallback(test_polishing::object_paramsConfig &config, uint32_t level); 

		bool isReachable(); 


		// Callback to update joint position
		// void updateObjectSpeed(const geometry_msgs::Vector3ConstPtr& msg);
};


#endif
