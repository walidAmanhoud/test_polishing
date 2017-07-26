#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include <string>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

std::string zeroPaddedStringConversion(int value, int desiredSize)
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

std_msgs::String buildOutputMessage(int mode, int speed, int acceleration, int decceleration)
{
    if(mode < 0 || mode > 2)
    {
        mode = 0;
    }

    if(speed < 40)
    {
        speed = 40;
    }
    else if(speed > 1500)
    {
        speed = 1500;
    }

    if(acceleration < 10)
    {
        acceleration = 10;
    }
    else if(acceleration > 600)
    {
        acceleration = 600;
    }

    if(decceleration < 10)
    {
        decceleration = 10;
    }
    else if(decceleration > 600)
    {
        decceleration = 600;
    }

    std_msgs::String result;

    result.data = zeroPaddedStringConversion(mode,1)
                + zeroPaddedStringConversion(speed,4)
                + zeroPaddedStringConversion(acceleration,3)
                + zeroPaddedStringConversion(decceleration,3);

    return result;
}

void processInputMessage(std::string input, int &speed, int &status)
{
    std::string s1,s2;
    s1 = input.substr(0,1);
    s2 = input.substr(1);

    speed = atoi(s1.c_str());
    status = atoi(s2.c_str());
}


int accDecConversion(float desiredAccDec)
{
    return (int) (desiredAccDec/0.1f);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyS0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);

    int mode = 1;
    int speed = 50;
    int acceleration = 50; 
    int decceleration = 50;

    std_msgs::String message = buildOutputMessage(mode,speed,acceleration,decceleration);

    std::string input = "11040";
    int currentSpeed;
    int status;

    processInputMessage(input, currentSpeed, status);
    std::cerr << currentSpeed << " " << status << std::endl;

    std::cerr << message.data << std::endl;
    while(ros::ok()){

        ros::spinOnce();

        // if(ser.available()){
        //     ROS_INFO_STREAM("Reading from serial port");
        //     std_msgs::String result;
        //     result.data = ser.read(ser.available());
        //     ROS_INFO_STREAM("Read: " << result.data);
        //     read_pub.publish(result);
        // }
        loop_rate.sleep();

    }

    ser.close();
}