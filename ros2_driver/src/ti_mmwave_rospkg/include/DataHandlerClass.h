#ifndef _DATA_HANDLER_CLASS_
#define _DATA_HANDLER_CLASS_

#include <pthread.h>
#include <algorithm>
//#include "pcl_ros/point_cloud.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cmath>
#include <signal.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <boost/shared_ptr.hpp>
#include "mmWave.h"
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "ti_mmwave_rospkg_msgs/msg/radar_scan.hpp"
#include "ti_mmwave_rospkg_msgs/msg/radar_occupancy.hpp"
#include "ti_mmwave_rospkg_msgs/msg/radar_point_track_id.hpp"
#include "ti_mmwave_rospkg_msgs/msg/radar_track_array.hpp"
#define COUNT_SYNC_MAX 2


class DataUARTHandlerNode : public rclcpp::Node 
{   
public:
    explicit DataUARTHandlerNode();


private:

    size_t stringin;
};

class DataUARTHandler
{
    
public:
    
    //DataUARTHandler();

    DataUARTHandler(std::shared_ptr<rclcpp::Node> nh);
    
    void setFrameID(char const* myFrameID);

    /*User callable function to set the UARTPort*/
    void setUARTPort(char const* mySerialPort);
    
    /*User callable function to set the BaudRate*/
    void setBaudRate(int myBaudRate);

    /*User callable function to set maxAllowedElevationAngleDeg*/
    void setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg);
    
    /*User callable function to set maxAllowedElevationAngleDeg*/
    void setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg);

    void setNodeHandle(std::shared_ptr<rclcpp::Node> nh);
      
    /*User callable function to start the handler's internal threads*/
    void start(void);
    
    /*User callable function to stop the handler's internal threads*/
    void stop();

    /*Helper functions to allow pthread compatability*/
    static void* readIncomingData_helper(void *context);
    
    static void* sortIncomingData_helper(void *context);
    
    static void* syncedBufferSwap_helper(void *context);

    /* Function to handle signals such as SIGINT */
    static void sigHandler(int32_t sig);

    /*Sorted mmwDemo Data structure*/
    mmwDataPacket mmwData;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr DataUARTHandler_pub;
    rclcpp::Publisher<ti_mmwave_rospkg_msgs::msg::RadarScan>::SharedPtr radar_scan_pub;
    rclcpp::Publisher<ti_mmwave_rospkg_msgs::msg::RadarOccupancy>::SharedPtr radar_occupancy_pub;
    rclcpp::Publisher<ti_mmwave_rospkg_msgs::msg::RadarPointTrackID>::SharedPtr radar_trackid_pub;
    rclcpp::Publisher<ti_mmwave_rospkg_msgs::msg::RadarTrackArray>::SharedPtr radar_trackarray_pub;

private:

    int nr;
    int nd;
    int ntx;
    int tfr_nanoseconds;
    float fs;
    float fc;
    float BW;
    float PRI;
    float tfr;
    float max_range;
    float vrange;
    float max_vel;
    float vvel;
    float zminx;
    float zmaxx;
    float zminy;
    float zmaxy;
    float zminz;
    float zmaxz;

    char const* frameID;
    /*Contains the name of the serial port*/
    char const* dataSerialPort;
    
    /*Contains the baud Rate*/
    int dataBaudRate;
    
    /*Contains the max_allowed_elevation_angle_deg (points with elevation angles 
      outside +/- max_allowed_elevation_angle_deg will be removed)*/
    int maxAllowedElevationAngleDeg;
    
    /*Contains the max_allowed_azimuth_angle_deg (points with azimuth angles 
      outside +/- max_allowed_azimuth_angle_deg will be removed)*/
    int maxAllowedAzimuthAngleDeg;
    
    /*Mutex protected variable which synchronizes threads*/
    int countSync;

    /*Boolean used to notify threads to exit*/
    bool stop_threads;
    
    /*Read/Write Buffers*/
    std::vector<uint8_t> pingPongBuffers[2];
    
    /*Pointer to current data (sort)*/
    std::vector<uint8_t>* currentBufp;
    
    /*Pointer to new data (read)*/
    std::vector<uint8_t>* nextBufp;
    
    /*Mutex protecting the countSync variable */
    pthread_mutex_t countSync_mutex;
    
    /*Mutex protecting the nextBufp pointer*/
    pthread_mutex_t nextBufp_mutex;
    
    /*Mutex protecting the currentBufp pointer*/
    pthread_mutex_t currentBufp_mutex;
    
    /*Condition variable which blocks the Swap Thread until signaled*/
    pthread_cond_t countSync_max_cv;
    
    /*Condition variable which blocks the Read Thread until signaled*/
    pthread_cond_t read_go_cv;
    
    /*Condition variable which blocks the Sort Thread until signaled*/
    pthread_cond_t sort_go_cv;
    
    /*Swap Buffer Pointers Thread*/
    void *syncedBufferSwap(void);
    
    /*Checks if the magic word was found*/
    int isMagicWord(uint8_t last8Bytes[8]);
    
    /*Read incoming UART Data Thread*/
    void *readIncomingData(void);
    
    /*Sort incoming UART Data Thread*/
    void *sortIncomingData(void);
    
    rclcpp::Node::SharedPtr nodeHandle;
};

#endif 
