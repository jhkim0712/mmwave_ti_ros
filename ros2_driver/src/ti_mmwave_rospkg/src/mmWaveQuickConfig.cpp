#include "rclcpp/rclcpp.hpp"
#include "ti_mmwave_rospkg_msgs/srv/mmwave_cli.hpp"
#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cstdlib>
#include <fstream>
#include <stdio.h>
#include <regex>
#include <thread>
#include <chrono>
#include <functional>
#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;

class mmWaveQuickConfig : public rclcpp::Node
{
public:

    mmWaveQuickConfig() : Node("mmWaveQuickConfig")
    {
        this->declare_parameter("mmwavecli_name", rclcpp::PARAMETER_STRING);
        this->declare_parameter("mmwavecli_cfg", rclcpp::PARAMETER_STRING);
    }

private:

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto nodeptr = std::make_shared<mmWaveQuickConfig>();
    rclcpp::Client<ti_mmwave_rospkg_msgs::srv::MmwaveCli>::SharedPtr client = nodeptr->create_client<ti_mmwave_rospkg_msgs::srv::MmwaveCli>("/ti_mmwave_rospkg_msgs/mmwave_cli");
    auto request = std::make_shared<ti_mmwave_rospkg_msgs::srv::MmwaveCli::Request>();
    auto response = std::make_shared<ti_mmwave_rospkg_msgs::srv::MmwaveCli::Response>();
    std::string mmWaveCLIname = nodeptr->get_parameter("mmwavecli_name").as_string();
    std::string mmWaveCLIcfg = nodeptr->get_parameter("mmwavecli_cfg").as_string();
    std::string sensorStart = "sensorStart";

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"\n\n==============================\nmmWave ROS Driver is starting\n==============================\n");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"mmWaveQuickConfig: Configuring mmWave device using config file: %s", mmWaveCLIcfg.c_str());

    std::ifstream myParams;
    //wait for service to become available
    client->wait_for_service(std::chrono::seconds(5));
    //wait 0.5 secs to avoid multi-sensor conflicts
    rclcpp::Rate rate(500);

    myParams.open(mmWaveCLIcfg);

    if (myParams.is_open())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"File was opened");
        while( std::getline(myParams, request->comm))
        {
        // Remove Windows carriage-return if present
            request->comm.erase(std::remove(request->comm.begin(), request->comm.end(), '\r'), request->comm.end());
        // Ignore comment lines (first non-space char is '%') or blank lines
            if (!(std::regex_match(request->comm, std::regex("^\\s*%.*")) || std::regex_match(request->comm, std::regex("^\\s*"))))
            {
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"mmWaveQuickConfig: Sending command: '%s'", request->comm.c_str() );
                //parser.ParamsParser(request, param_node);
                auto result = client->async_send_request(request);
                if (!(rclcpp::spin_until_future_complete(nodeptr, result) == rclcpp::FutureReturnCode::SUCCESS))
                {
                   RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"\n\n==================================\nmmWave ROS Driver is shutting down\n==================================\n");
                   // return 1;
                }

            }
        }
        myParams.close();
        rclcpp::shutdown();
        exit(0);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"mmWaveQuickConfig: Terminated %s", mmWaveCLIcfg.c_str());
        return 0;
    }
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"mmWaveQuickConfig: mmWaveQuickConfig will now terminate. Done configuring mmWave device using config file");
    rclcpp::spin(nodeptr);
    rclcpp::shutdown();
    exit(0);
    return 0;
}
