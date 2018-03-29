#ifndef __ATTRACTIVE_MOTION_GENERATOR_H__
#define __ATTRACTIVE_MOTION_GENERATOR_H__

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"

#include <vector>

#include "MathLib.h"
#include "CDDynamics.h"

#include <mutex>
#include <pthread.h>

#include <dynamic_reconfigure/server.h>
#include <test_polishing/attractive_paramsConfig.h>

class AttractiveMotionGenerator {


private:


	// ROS variables
	ros::NodeHandle _n;
	ros::Rate _loopRate;

	ros::Subscriber _subRealPose;
	ros::Subscriber _subAttractorState;
	ros::Subscriber _subEndEffectorFt;

	ros::Publisher _pubAttractorPosition;
	ros::Publisher _pubDesiredTwist;
	ros::Publisher _pubDesiredPath;
	ros::Publisher _pubDesiredOrientation;

	std::string inputTopicName;
	std::string outputTopicName;

	geometry_msgs::Pose _msgRealPose;
	geometry_msgs::Pose _msgDesiredPose;
	geometry_msgs::Twist _msgDesiredTwist;
	geometry_msgs::Quaternion _msgDesiredOrientation;
	geometry_msgs::WrenchStamped _msgEndEffectorFt;

	nav_msgs::Path _msgDesiredPath;
	int MAX_FRAME = 200;
	float _velocityLimit;
	float _convergenceScale;
	float _convergenceRate;
	float _dt;

	float _kp;
	float _ki;
	float _pidInteg = 0.0f;
	float _pidError = 0.0f;
	float _up;
	float _ui;
	float _targetForce;

	MathLib::Vector _pidAttractorOffset;


	bool _firstContact = false;
	bool _usePid = false;




	//dynamic reconfig settig
	dynamic_reconfigure::Server<test_polishing::attractive_paramsConfig> _dynRecServer;
	dynamic_reconfigure::Server<test_polishing::attractive_paramsConfig>::CallbackType _dynRecCallback;

	// Class variables
	std::mutex mutex_;
    pthread_t _thread;
    bool _startThread;

	MathLib::Vector _attractorPosition;
	MathLib::Vector _attractorOffset;
	MathLib::Vector _attractorSpeed;
	MathLib::Vector _endEffectorForces;

	MathLib::Vector _realPose;
	MathLib::Vector _desiredPosition;
	MathLib::Vector _desiredVelocity;

	bool _firstRealPoseReceived = false;
	bool _firstAttractorStateReceived = false;

	// bool _outputVelocity = true;


public:
	AttractiveMotionGenerator(ros::NodeHandle &n, double frequency, std::string inputTopicName, std::string outputTopicName, double convergenceRate);

	bool init();

	void run();

	void setAttractorPosition(float px, float py, float pz);
	void setDesiredOrientation(geometry_msgs::Quaternion msg); 


private:

	bool InitializeDS();

	bool InitializeROS();


	void updateRealPosition(const geometry_msgs::Pose::ConstPtr& msg);

	void updateEndEffectorFt(const geometry_msgs::WrenchStamped::ConstPtr& msg);

	void updateAttractorState(const std_msgs::Float64MultiArray::ConstPtr& msg);


	void computeDesiredVelocity();

	void publishData();

	void publishFuturePath();


	void dynamicReconfigureCallback(test_polishing::attractive_paramsConfig &config, uint32_t level);


 	static void* startPathPublishingLoop(void* ptr);

    void pathPublishingLoop();   


};


#endif
