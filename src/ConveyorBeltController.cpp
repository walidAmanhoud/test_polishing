#include "ConveyorBeltController.h"
// #include <tf/transform_datatypes.h>


ConveyorBeltController::ConveyorBeltController(ros::NodeHandle &n, float frequency):
	_n(n),
  _loopRate(frequency)
{

	ROS_INFO_STREAM("The conveyor belt controller node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool ConveyorBeltController::init() 
{
	// Publish conveyor belt speed
	_pubConveyorBeltSpeed = _n.advertise<std_msgs::Int32>("conveyorBelt/speed", 1);


	_dynRecCallback = boost::bind(&ConveyorBeltController::dynamicReconfigureCallback,this, _1, _2);
	_dynRecServer.setCallback(_dynRecCallback);
	_dynRecServer.getConfigDefault(_config);

	_mode = _config.mode;
	_desiredSpeed = _config.speed;
	_acceleration = _config.acc;
	_decceleration = _config.dec;

  try
  {
    _serial.setPort("/dev/ttyS0");
    _serial.setBaudrate(9600);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    _serial.setTimeout(timeout);
    _serial.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port ");
    return false;
  }

  if(_serial.isOpen())
  {
    ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
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

void ConveyorBeltController::setDesiredConfig(int mode, int speed, int acceleration, int decceleration) 
{
	_mode = mode;
	_desiredSpeed = speed;
	_acceleration = acceleration;
	_decceleration = decceleration;

	_config.mode = _mode;
	_config.speed = _desiredSpeed;
	_config.acc = _acceleration;
	_config.dec = _decceleration;

	_dynRecServer.updateConfig(_config);
}



void ConveyorBeltController::run() {

	startConveyorBelt();

	while (_n.ok()) 
	{

    if(_serial.available())
    {
      std_msgs::String input;
      input.data = _serial.read(_serial.available());
      processInputSerialMessage(input.data);
    }

    _pubConveyorBeltSpeed.publish(_speedMessage);

		ros::spinOnce();
		_loopRate.sleep();

	}

	stopConveyorBelt();

	_serial.close();
}

void ConveyorBeltController::startConveyorBelt()
{
	if(_mode == 0)
	{
		_mode = 1;
		_config.mode = _mode;
		_dynRecServer.updateConfig(_config);
	}

	sendCommand();
}


void ConveyorBeltController::stopConveyorBelt()
{

	_mode = 0;		
	_config.mode = _mode;
	_dynRecServer.updateConfig(_config);

	sendCommand();
}


void ConveyorBeltController::processInputSerialMessage(std::string input)
{
    std::string s1,s2;
    s1 = input.substr(0,1);
    s2 = input.substr(1);

    int status = atoi(s2.c_str());
    if(!status)
    {
    	_measuredSpeed = atoi(s1.c_str());	
    }
    else
    {
    	_measuredSpeed = 0;
		ROS_INFO("An error occured, set measured speed to 0");
    }

    _speedMessage.data = _measuredSpeed;
}

void ConveyorBeltController::sendCommand()
{
	buildOutputSerialMessage();
	std::cerr << "Message sent to conveyor belt: " << _outputSerialMessage << std::endl;
	_serial.write(_outputSerialMessage);
}


void ConveyorBeltController::buildOutputSerialMessage()
{
    _outputSerialMessage = zeroPaddedStringConversion(_mode,1)
                + zeroPaddedStringConversion(_desiredSpeed,4)
                + zeroPaddedStringConversion(_acceleration,3)
                + zeroPaddedStringConversion(_decceleration,3);
}


std::string ConveyorBeltController::zeroPaddedStringConversion(int value, int desiredSize)
{

    std::stringstream ss;
    
    // the integer value is converted to string with the help of stringstream
    ss << value; 
    std::string result;
    ss >> result;
    
    // Append zero chars
    int initialSize = result.length();
    for (int k = 0; k < desiredSize-initialSize; k++)
    {
        result = "0" + result;
    }

    return result;
}


void ConveyorBeltController::dynamicReconfigureCallback(test_polishing::conveyorBelt_paramsConfig &config, uint32_t level) 
{

	ROS_INFO("Reconfigure request. Updatig the parameters ...");
	
	_mode = config.mode;
	_desiredSpeed = config.speed;
	_acceleration = config.acc;
	_decceleration = config.dec;

	sendCommand();

}