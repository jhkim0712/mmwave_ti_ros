/*Include standard C/C++ headers*/
#include <iostream>
#include <cstdio>
#include <stdio.h>
#include <sstream>
#include <cstdlib>
#include <fstream>
#include <regex>
#include <thread>
#include <chrono>
#include <functional>
#include <string>

/*Include ROS specific headers*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h"  
#include "pluginlib/class_list_macros.hpp"

/*mmWave Driver Headers*/
#include "ti_mmwave_rospkg_msgs/msg/radar_scan.hpp"
#include "ti_mmwave_rospkg_msgs/srv/mmwave_cli.hpp"

using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> nodeptr2 = nullptr;

class mmWaveCommSrv : public rclcpp::Node
{  
public:

    mmWaveCommSrv() : Node("mmWaveCommSrv")
    {
        this->declare_parameter("command_port", rclcpp::PARAMETER_STRING);
        this->declare_parameter("command_rate", rclcpp::PARAMETER_STRING);
        this->declare_parameter("data_port", rclcpp::PARAMETER_STRING);
        this->declare_parameter("data_rate", rclcpp::PARAMETER_STRING);
        this->declare_parameter("max_allowed_elevation_angle_deg", rclcpp::PARAMETER_STRING);
        this->declare_parameter("max_allowed_azimuth_angle_deg", rclcpp::PARAMETER_STRING);
        this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
        this->declare_parameter("mmwavecli_name", rclcpp::PARAMETER_STRING);
        this->declare_parameter("/ti_mmwave/radar_scan_pcl", rclcpp::PARAMETER_STRING);

    }
private:

};

void handle_service_request(const std::shared_ptr<ti_mmwave_rospkg_msgs::srv::MmwaveCli::Request> request, 
    const std::shared_ptr<ti_mmwave_rospkg_msgs::srv::MmwaveCli::Response> response)
{

    std::string mySerialPort = nodeptr2->get_parameter("command_port").as_string();
    std::string myBaudRate = nodeptr2->get_parameter("command_rate").as_string();
    std::string mmWaveCLIName = nodeptr2->get_parameter("mmwavecli_name").as_string();
    std::string sensorStart = "sensorStart\n";

            /*Open Serial port and error check*/
    serial::Serial mySerialObject("", std::stoi(myBaudRate), serial::Timeout::simpleTimeout(1000));
    mySerialObject.setPort(mySerialPort.c_str());
    try 
    {
        mySerialObject.open();
    } 
    catch (std::exception &e1) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"\n\n\nmmWaveCommSrv: Failed to open User serial port with error: %s\n\n", e1.what());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"\n\n\nmmWaveCommSrv: Power cycle the mmWave Sensor with the reset button while keeping the USB connected. Ensure the correct launch and configuration files are being used. Close all nodes, wait 10 Seconds, then relaunch the driver\n\n");
        rclcpp::shutdown();
        exit(0);
    }

    /*Read any previous pending response(s)*/
    while (mySerialObject.available() > 0)
    {   
        mySerialObject.readline(response->resp, 1024, ":/>");
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"mmWaveCommSrv: Received (previous) response from sensor: '%s'", response->resp.c_str());
        response->resp = "";
    }

    /*Send out command received from the client*/
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"mmWaveCommSrv: Sending command to sensor: '%s'", request->comm.c_str());
    request->comm.append("\n");
    int bytesSent = mySerialObject.write(request->comm.c_str());

    /*Read output from mmwDemo*/
    mySerialObject.readline(response->resp, 1024, ":/>");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"mmWaveCommSrv: Received response from sensor: '%s'", response->resp.c_str());

    mySerialObject.close();

    if(!sensorStart.compare(request->comm))
    {
        rclcpp::shutdown();
        exit(0);   
    }

}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    nodeptr2 = std::make_shared<mmWaveCommSrv>();
    rclcpp::Service<ti_mmwave_rospkg_msgs::srv::MmwaveCli>::SharedPtr service = nodeptr2->create_service<ti_mmwave_rospkg_msgs::srv::MmwaveCli>
    ("/ti_mmwave_rospkg_msgs/mmwave_cli", handle_service_request);
    rclcpp::spin(nodeptr2);
    rclcpp::shutdown();
    exit(0);
    return 0;
}
