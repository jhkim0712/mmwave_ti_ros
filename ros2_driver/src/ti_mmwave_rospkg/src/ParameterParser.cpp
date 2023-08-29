#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ti_mmwave_rospkg_msgs/srv/mmwave_cli.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>
#include <regex>
#include <memory>

std::shared_ptr<rclcpp::Node> nodeptr3 = nullptr;

class ParameterParser : public rclcpp::Node 
{   
public:
    ParameterParser() : Node("ParameterParser")
    {
        this->declare_parameter("device_name", rclcpp::PARAMETER_STRING);        
        this->declare_parameter("mmwavecli_name", rclcpp::PARAMETER_STRING);
        this->declare_parameter("mmwavecli_cfg", rclcpp::PARAMETER_STRING);
        parameters_client_test = std::make_shared<rclcpp::AsyncParametersClient>(this, "/ConfigParameterServer");
        parameters_client_test->wait_for_service();
        auto parameters_future = parameters_client_test->get_parameters({        
            "/ti_mmwave/startFreq",
            "/ti_mmwave/idleTime",
            "/ti_mmwave/adcStartTime",
            "/ti_mmwave/rampEndTime",
            "/ti_mmwave/freqSlopeConst",
            "/ti_mmwave/numAdcSamples",
            "/ti_mmwave/digOutSampleRate",
            "/ti_mmwave/rxGain",
            "/ti_mmwave/chirpStartIdx",
            "/ti_mmwave/chirpEndIdx",
            "/ti_mmwave/numLoops",
            "/ti_mmwave/numFrames",
            "/ti_mmwave/framePeriodicity",
            "/ti_mmwave/zoneMinX",
            "/ti_mmwave/zoneMaxX",
            "/ti_mmwave/zoneMinY",
            "/ti_mmwave/zoneMaxY",
            "/ti_mmwave/zoneMinZ",
            "/ti_mmwave/zoneMaxZ" 
        },
        std::bind(&ParameterParser::callbackGlobalParam, this, std::placeholders::_1));
    }

    void callbackGlobalParam(std::shared_future<std::vector<rclcpp::Parameter>> future)
    {
        auto result = future.get();
        auto param = result.at(0);
        //RCLCPP_INFO(this->get_logger(), "Got global param: %s", param.as_string().c_str());
        rclcpp::shutdown();
    }

private:
    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_test;
};

int main(int argc, char **argv) {

    float c0 = 299792458;
    int chirpStartIdx;
    int chirpEndIdx;
    int numLoops;
    int numFrames;
    float framePeriodicity;
    float startFreq;
    float idleTime;
    float adcStartTime;
    float rampEndTime;
    float digOutSampleRate;
    float freqSlopeConst;
    float numAdcSamples;
    float zoneMinX;
    float zoneMaxX;
    float zoneMinY;
    float zoneMaxY;
    float zoneMinZ;
    float zoneMaxZ;

    rclcpp::init(argc, argv);
    nodeptr3 = std::make_shared<ParameterParser>();
    std::string token;
    std::ifstream myCfgParam;
    std::string str_param;
    std::string deviceName = nodeptr3->get_parameter("device_name").as_string();
    std::string mmWaveCLIname = nodeptr3->get_parameter("mmwavecli_name").as_string();
    std::string mmWaveCLIcfg = nodeptr3->get_parameter("mmwavecli_cfg").as_string();
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(nodeptr3, "/ConfigParameterServer");
    myCfgParam.open(mmWaveCLIcfg);

    if (deviceName.compare("6432") != 0)
    {
        if (myCfgParam.is_open()) 
        {
            while( std::getline(myCfgParam, str_param)) 
            {
                str_param.erase(std::remove(str_param.begin(), str_param.end(), '\r'), str_param.end());
                if (!(std::regex_match(str_param, std::regex("^\\s*%.*")) || std::regex_match(str_param, std::regex("^\\s*")))) 
                {
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"myParams equals %s\n", str_param.c_str() );
                    std::istringstream ss(str_param);
                    std::vector <std::string> v;
                    while(std::getline(ss, token, ' '))
                    {
                        v.push_back(token);
                    }

                    if (!v[0].compare("profileCfg")) 
                    {
                    //RCLCPP_INFO(this->get_logger(), "ProfileCfg");
                        parameters_client->set_parameters(
                        {
                            rclcpp::Parameter("/ti_mmwave/startFreq", v[2]),
                            rclcpp::Parameter("/ti_mmwave/idleTime", v[3]),
                            rclcpp::Parameter("/ti_mmwave/adcStartTime", v[4]),
                            rclcpp::Parameter("/ti_mmwave/rampEndTime", v[5]),
                            rclcpp::Parameter("/ti_mmwave/freqSlopeConst", v[8]),
                            rclcpp::Parameter("/ti_mmwave/numAdcSamples", v[10]),
                            rclcpp::Parameter("/ti_mmwave/digOutSampleRate", v[11]),
                            rclcpp::Parameter("/ti_mmwave/rxGain", v[14])
                        });
                        startFreq = std::stof(v[2]);
                        idleTime = std::stof(v[3]);
                        adcStartTime = std::stof(v[4]);
                        rampEndTime = std::stof(v[5]);
                        freqSlopeConst = std::stof(v[8]);
                        numAdcSamples = std::stof(v[10]);
                        digOutSampleRate = std::stof(v[11]);
                    } 
                    else if (!v[0].compare("frameCfg")) 
                    {
                        parameters_client->set_parameters(
                        {
                            rclcpp::Parameter("/ti_mmwave/chirpStartIdx", v[1]),
                            rclcpp::Parameter("/ti_mmwave/chirpEndIdx", v[2]),
                            rclcpp::Parameter("/ti_mmwave/numLoops", v[3]),
                            rclcpp::Parameter("/ti_mmwave/numFrames", v[4]),
                            rclcpp::Parameter("/ti_mmwave/framePeriodicity", v[5])
                        });

                        chirpStartIdx = std::stoi(v[1]);
                        chirpEndIdx = std::stoi(v[2]);
                        numLoops = std::stoi(v[3]);
                        numFrames = std::stoi(v[4]);
                        framePeriodicity = std::stof(v[5]);
                    } 
                    else if (!v[0].compare("zoneDef")) 
                    {
                        parameters_client->set_parameters(
                        {
                            rclcpp::Parameter("/ti_mmwave/zoneMinX", v[2]),
                            rclcpp::Parameter("/ti_mmwave/zoneMaxX", v[3]),
                            rclcpp::Parameter("/ti_mmwave/zoneMinY", v[4]),
                            rclcpp::Parameter("/ti_mmwave/zoneMaxY", v[5]),
                            rclcpp::Parameter("/ti_mmwave/zoneMinZ", v[6]),
                            rclcpp::Parameter("/ti_mmwave/zoneMaxZ", v[7])
                        });

                        zoneMinX = std::stoi(v[2]);
                        zoneMaxX = std::stoi(v[3]);
                        zoneMinY = std::stoi(v[4]);
                        zoneMaxY = std::stoi(v[5]);
                        zoneMinZ = std::stoi(v[6]);
                        zoneMaxZ = std::stoi(v[7]);
                    }
                }                
            }
        }

        int ntx = chirpEndIdx - chirpStartIdx + 1;
        int nd = numLoops;
        int nr = numAdcSamples;
        float tfr = framePeriodicity * 1e-3;
        float fs = digOutSampleRate * 1e3;
        float kf = freqSlopeConst * 1e12;
        float adc_duration = nr / fs;
        float BW = adc_duration * kf;
        float PRI = (idleTime + rampEndTime) * 1e-6;
        float fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2); 
        float vrange = c0 / (2 * BW);
        float max_range = nr * vrange;
        float max_vel = c0 / (2 * fc * PRI) / ntx;
        float vvel = max_vel / nd;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"\n\n==============================\nList of parameters\n==============================\nNumber of range samples: %d\nNumber of chirps: %d\nf_s: %.3f MHz\nf_c: %.3f GHz\nBandwidth: %.3f MHz\nPRI: %.3f us\nFrame time: %.3f ms\nMax range: %.3f m\nRange resolution: %.3f m\nMax Doppler: +-%.3f m/s\nDoppler resolution: %.3f m/s\n==============================\n",
            nr, nd, fs/1e6, fc/1e9, BW/1e6, PRI*1e6, tfr*1e3, max_range, vrange, max_vel/2, vvel);
    }
    rclcpp::spin_some(nodeptr3);
    rclcpp::shutdown();
    return 0;
}
