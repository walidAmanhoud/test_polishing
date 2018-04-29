#ifndef __MOVE_TO_DESIRED_JOINTS_H__
#define __MOVE_TO_DESIRED_JOINTS_H__

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <robot_motion_generation/CDDynamics.h>
#include "kuka_fri_bridge/JointStateImpedance.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <mutex>
#include "mathlib_eigen_conversions.h"


#define JOINT_TOLERANCE 1e-2f
#define NB_JOINTS 7

class MoveToDesiredJoints 
{
	public:
		enum Mode {SIM = 0, REAL = 1};
	
	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Subscriber _subCurrentJoints;
		ros::Publisher _pubDesiredJoints;

		// Node variables
		kuka_fri_bridge::JointStateImpedance _desiredJointsRealMsg;
		std_msgs::Float32MultiArray _desiredJointsSimMsg;

		Eigen::VectorXd _finalDesiredJoints;
		Eigen::VectorXd _desiredJoints;
		Eigen::VectorXd _currentJoints;

		float _jointTolerance;
		bool _firstJointsUpdate;

		// Class variables
		std::mutex _mutex;

		Mode _mode;

		motion::CDDynamics _filter;

		float _dt;


	public:
		MoveToDesiredJoints(ros::NodeHandle &n, float frequency, Mode mode);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

		void computeDesiredJoints();

		void publishData();

		void setDesiredJoints(Eigen::VectorXd finalDesiredJoints);

	private:

		// Callback to update joint position
		void updateCurrentJoints(const sensor_msgs::JointState::ConstPtr& msg);

		// Check joints error
		bool checkJointsError();
};


#endif
