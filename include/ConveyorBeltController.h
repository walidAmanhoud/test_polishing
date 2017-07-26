#ifndef __CONVOYER_BELT_CONTROLLER_H__
#define __CONVOYER_BELT_CONTROLLER_H__

#include "ros/ros.h"
#include <serial/serial.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <vector>
#include <mutex>
#include <Eigen/Eigen>

#include <dynamic_reconfigure/server.h>
#include <test_polishing/conveyorBelt_paramsConfig.h>

// #include "visualization_msgs/MarkerArray.h"

class ConveyorBeltController 
{

	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;

		// Subscribers and publishers definition
		ros::Publisher _pubConveyorBeltSpeed;

		// Node variables
		int _mode;
		int _desiredSpeed;
		int _measuredSpeed;
		int _acceleration;
		int _decceleration;
		serial::Serial _serial;
		std_msgs::Int32 _speedMessage;
		std::string _outputSerialMessage;

		// Class variables
		std::mutex _mutex;

		// Dynamic reconfigure definition (server+callback)
		dynamic_reconfigure::Server<test_polishing::conveyorBelt_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<test_polishing::conveyorBelt_paramsConfig>::CallbackType _dynRecCallback;
		test_polishing::conveyorBelt_paramsConfig _config;


	public:

		ConveyorBeltController(ros::NodeHandle &n, float frequency);

		// Initialize node
		bool init();

		// Run node main loop
		void run();

		void setDesiredConfig(int mode, int speed, int acceleration, int decceleration);


	private:
		
		void startConveyorBelt();

		void stopConveyorBelt();

		void sendCommand();

		std::string zeroPaddedStringConversion(int value, int desiredSize);

		void processInputSerialMessage(std::string input);

		void buildOutputSerialMessage();

		void dynamicReconfigureCallback(test_polishing::conveyorBelt_paramsConfig &config, uint32_t level); 
};


#endif
