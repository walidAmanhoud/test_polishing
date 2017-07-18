#ifndef __MOVE_TO_DESIRED_JOINTS_H__
#define __MOVE_TO_DESIRED_JOINTS_H__

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <mutex>

class MoveToDesiredJoints 
{

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subCurrentJoints;
		ros::Publisher _pubDesiredJoints;

		// Node variables
		sensor_msgs::JointState _currentJoints;
		std_msgs::Float64MultiArray _desiredJoints;
		float _jointTolerance;
		bool _firstJointsUpdate;

		// Class variables
		std::mutex _mutex;


	public:
		MoveToDesiredJoints(ros::NodeHandle &n, float frequency, float jointTolerance = 1.0e-3f);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

		// Set desired joint angles
		void setDesiredJoints(std_msgs::Float64MultiArray desiredJoints);

	private:


		// Callback to update joint position
		void updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg);

		// Check joints error
		bool checkJointsError();

};


#endif
