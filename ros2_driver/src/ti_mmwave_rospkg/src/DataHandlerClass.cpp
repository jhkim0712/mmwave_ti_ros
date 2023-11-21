#include <DataHandlerClass.h>
#define PCL_NO_PRECOMPILE

//#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using namespace std::chrono_literals;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr DataUARTHandler_pub;
rclcpp::Publisher<ti_mmwave_rospkg_msgs::msg::RadarScan>::SharedPtr radar_scan_pub;
rclcpp::Publisher<ti_mmwave_rospkg_msgs::msg::RadarOccupancy>::SharedPtr radar_occupancy_pub;
rclcpp::Publisher<ti_mmwave_rospkg_msgs::msg::RadarPointTrackID>::SharedPtr radar_trackid_pub;
rclcpp::Publisher<ti_mmwave_rospkg_msgs::msg::RadarTrackArray>::SharedPtr radar_trackarray_pub;
std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2_msg_;
DataUARTHandler* gDataHandlerPtr;
rclcpp::Clock clocker;

DataUARTHandlerNode::DataUARTHandlerNode() : Node("DataUARTHandlerNode")
{

    this->declare_parameter("mmwavecli_name", rclcpp::PARAMETER_STRING);
    this->declare_parameter("mmwavecli_cfg", rclcpp::PARAMETER_STRING);
    this->declare_parameter("data_port", rclcpp::PARAMETER_STRING);
    this->declare_parameter("data_rate", rclcpp::PARAMETER_STRING);

}

void DataUARTHandler::sigHandler(int32_t sig)
{
    switch(sig)
    {
    case SIGINT:
        gDataHandlerPtr->stop();

    }

    rclcpp::shutdown();
    exit(0);
}

struct mmWaveCloudType
{
    PCL_ADD_POINT4D;
    union
    {
        struct
        {
            float intensity;
            float velocity;
        };
        float data_c[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (mmWaveCloudType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, velocity, velocity))

DataUARTHandler::DataUARTHandler(std::shared_ptr<rclcpp::Node> nh) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1])
{
    DataUARTHandler_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/ti_mmwave/radar_scan_pcl", 100);
    radar_scan_pub = nh->create_publisher<ti_mmwave_rospkg_msgs::msg::RadarScan>("/ti_mmwave/radar_scan", 100);
    radar_occupancy_pub = nh->create_publisher<ti_mmwave_rospkg_msgs::msg::RadarOccupancy>("/ti_mmwave/radar_occupancy", 100);
    radar_trackid_pub = nh->create_publisher<ti_mmwave_rospkg_msgs::msg::RadarPointTrackID>("/ti_mmwave/radar_point_track_id", 100);
    radar_trackarray_pub = nh->create_publisher<ti_mmwave_rospkg_msgs::msg::RadarTrackArray>("/ti_mmwave/radar_track_array", 100);
    maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
    maxAllowedAzimuthAngleDeg = 90; // Use max angle if none specified
    gDataHandlerPtr = this;
    stop_threads = false;
}

void DataUARTHandler::setFrameID(char const* myFrameID)
{
    frameID = myFrameID;
}

/*Implementation of setUARTPort*/
void DataUARTHandler::setUARTPort(char const* mySerialPort)
{
    dataSerialPort = mySerialPort;
}

/*Implementation of setBaudRate*/
void DataUARTHandler::setBaudRate(int myBaudRate)
{
    dataBaudRate = myBaudRate;
}

/*Implementation of setMaxAllowedElevationAngleDeg*/
void DataUARTHandler::setMaxAllowedElevationAngleDeg(int myMaxAllowedElevationAngleDeg)
{
    maxAllowedElevationAngleDeg = myMaxAllowedElevationAngleDeg;
}

/*Implementation of setMaxAllowedAzimuthAngleDeg*/
void DataUARTHandler::setMaxAllowedAzimuthAngleDeg(int myMaxAllowedAzimuthAngleDeg)
{
    maxAllowedAzimuthAngleDeg = myMaxAllowedAzimuthAngleDeg;
}

/*Implementation of readIncomingData*/
void *DataUARTHandler::readIncomingData(void)
{

    int firstPacketReady = 0;
    uint8_t last8Bytes[8] = {0};

    /*Open UART Port and error checking*/
    serial::Serial mySerialObject("", dataBaudRate, serial::Timeout::simpleTimeout(10000));
    mySerialObject.setPort(dataSerialPort);
    while(!mySerialObject.isOpen())
    {
        try
        {
            mySerialObject.open();
        }
        catch (std::exception &e1) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Read Thread: Failed to open Data serial port with error: %s", e1.what());
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Read Thread: Waiting 20 seconds before trying again...");
            try
            {
                // Wait 10 seconds and try to open serial port again
                rclcpp::Rate rate(10000);
                mySerialObject.open();
            }
            catch (std::exception &e2) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Read Thread: Failed second time to open Data serial port, error: %s", e1.what());
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Read Thread: Port could not be opened. Port is \"%s\" and baud rate is %d", dataSerialPort, dataBaudRate);
                pthread_exit(NULL);
            }
        }

        if(mySerialObject.isOpen())
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Read Thread: Port is open");
        else
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Read Thread: Port could not be opened");
    }

    /*Quick magicWord check to synchronize program with data Stream*/
    while(!isMagicWord(last8Bytes))
    {

        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];
        mySerialObject.read(&last8Bytes[7], 1);
    }

    /*Lock nextBufp before entering main loop*/
    pthread_mutex_lock(&nextBufp_mutex);

    while(rclcpp::ok())
    {
        /*Start reading UART data and writing to buffer while also checking for magicWord*/
        last8Bytes[0] = last8Bytes[1];
        last8Bytes[1] = last8Bytes[2];
        last8Bytes[2] = last8Bytes[3];
        last8Bytes[3] = last8Bytes[4];
        last8Bytes[4] = last8Bytes[5];
        last8Bytes[5] = last8Bytes[6];
        last8Bytes[6] = last8Bytes[7];

        mySerialObject.read(&last8Bytes[7], 1);

        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Read Thread: last8bytes = %02x%02x %02x%02x %02x%02x %02x%02x",  last8Bytes[7], last8Bytes[6], last8Bytes[5], last8Bytes[4], last8Bytes[3], last8Bytes[2], last8Bytes[1], last8Bytes[0]);

        nextBufp->push_back( last8Bytes[7] );  //push byte onto buffer

        /*If a magicWord is found wait for sorting to finish and switch buffers*/
        if( isMagicWord(last8Bytes) )
        {
            /*Lock countSync Mutex while unlocking nextBufp so that the swap thread can use it*/
            pthread_mutex_lock(&countSync_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);

            /*increment countSync*/
            countSync++;

            /*If this is the first packet to be found, increment countSync again since Sort thread is not reading data yet*/
            if(firstPacketReady == 0)
            {
                countSync++;
                firstPacketReady = 1;
            }

            /*Signal Swap Thread to run if countSync has reached its max value*/
            if(countSync == COUNT_SYNC_MAX)
            {
                pthread_cond_signal(&countSync_max_cv);
            }

            /*Wait for the Swap thread to finish swapping pointers and signal us to continue*/
            pthread_cond_wait(&read_go_cv, &countSync_mutex);

            /*Unlock countSync so that Swap Thread can use it*/
            pthread_mutex_unlock(&countSync_mutex);
            pthread_mutex_lock(&nextBufp_mutex);

            nextBufp->clear();
            memset(last8Bytes, 0, sizeof(last8Bytes));

        }
    }

    mySerialObject.close();
    pthread_exit(NULL);
}


int DataUARTHandler::isMagicWord(uint8_t last8Bytes[8])
{
    int val = 0, i = 0, j = 0;

    for(i = 0; i < 8 ; i++)
    {
        if( last8Bytes[i] == magicWord[i])
        {
            j++;
        }
    }

    if( j == 8)
    {
        val = 1;
    }

    return val;
}

void *DataUARTHandler::syncedBufferSwap(void)
{
    while(rclcpp::ok())
    {
        pthread_mutex_lock(&countSync_mutex);

        while(countSync < COUNT_SYNC_MAX)
        {
            if(stop_threads)
            {
                pthread_mutex_unlock(&countSync_mutex);
                pthread_cond_signal(&sort_go_cv);
                pthread_cond_signal(&read_go_cv);
                pthread_exit(NULL);
            }

            pthread_cond_wait(&countSync_max_cv, &countSync_mutex);

            pthread_mutex_lock(&currentBufp_mutex);
            pthread_mutex_lock(&nextBufp_mutex);

            std::vector<uint8_t>* tempBufp = currentBufp;

            this->currentBufp = this->nextBufp;

            this->nextBufp = tempBufp;

            pthread_mutex_unlock(&currentBufp_mutex);
            pthread_mutex_unlock(&nextBufp_mutex);

            countSync = 0;

            pthread_cond_signal(&sort_go_cv);
            pthread_cond_signal(&read_go_cv);

        }

        pthread_mutex_unlock(&countSync_mutex);

    }

    pthread_exit(NULL);

}

void *DataUARTHandler::sortIncomingData(void)
{
    MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
    uint32_t tlvLen = 0;
    uint32_t headerSize;
    uint32_t tlvSize = 0;
    unsigned int currentDatap = 0;
    SorterState sorterState = READ_HEADER;
    int i = 0, tlvCount = 0, offset = 0;
    int j = 0;
    float maxElevationAngleRatioSquared;
    float maxAzimuthAngleRatio;
    float realElevation;
    float realAzimuth;
    float realDoppler;
    float realRange;
    float realSNR;
    float realNoise;
    float realX;
    float realY;
    float realZ;
    boost::shared_ptr<pcl::PointCloud<mmWaveCloudType>> RScan(new pcl::PointCloud<mmWaveCloudType>);
    ti_mmwave_rospkg_msgs::msg::RadarPointTrackID radartrackid;
    ti_mmwave_rospkg_msgs::msg::RadarTrackArray radartrackarray;
    ti_mmwave_rospkg_msgs::msg::RadarTrackContents radartrackcontents;
    ti_mmwave_rospkg_msgs::msg::RadarOccupancy radaroccupancy;
    ti_mmwave_rospkg_msgs::msg::RadarScan radarscan;

    //wait for first packet to arrive
    pthread_mutex_lock(&countSync_mutex);
    pthread_cond_wait(&sort_go_cv, &countSync_mutex);
    pthread_mutex_unlock(&countSync_mutex);
    pthread_mutex_lock(&currentBufp_mutex);

    while(rclcpp::ok())
    {
        switch(sorterState)
        {
        case READ_HEADER:
            {
            //init variables
                mmwData.numObjOut = 0;

            //make sure packet has at least first three fields (12 bytes) before we read them (does not include magicWord since it was already removed)
                if(currentBufp->size() < 12)
                {
                    sorterState = SWAP_BUFFERS;
                    break;
                }

            //get version (4 bytes)
                memcpy( &mmwData.header.version, &currentBufp->at(currentDatap), sizeof(mmwData.header.version));
                currentDatap += ( sizeof(mmwData.header.version) );

            //get totalPacketLen (4 bytes)
                memcpy( &mmwData.header.totalPacketLen, &currentBufp->at(currentDatap), sizeof(mmwData.header.totalPacketLen));
                currentDatap += ( sizeof(mmwData.header.totalPacketLen) );

            //get platform (4 bytes)
                memcpy( &mmwData.header.platform, &currentBufp->at(currentDatap), sizeof(mmwData.header.platform));
                currentDatap += ( sizeof(mmwData.header.platform) );

            //if packet doesn't have correct header size (which is based on platform), throw it away
            //  (does not include magicWord since it was already removed)
                if ((mmwData.header.platform & 0xFFFF) == 0x1443)  // platform is xWR1443)
                {
                    headerSize = 7 * 4;  // xWR1443 SDK demo header does not have subFrameNumber field
                }
                else
                {
                    headerSize = 8 * 4;  // header includes subFrameNumber field
                }

                if(currentBufp->size() < headerSize)
                {
                    sorterState = SWAP_BUFFERS;
                    break;
                }

                    //get frameNumber (4 bytes)
                memcpy( &mmwData.header.frameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.frameNumber));
                currentDatap += ( sizeof(mmwData.header.frameNumber) );

                    //get timeCpuCycles (4 bytes)
                memcpy( &mmwData.header.timeCpuCycles, &currentBufp->at(currentDatap), sizeof(mmwData.header.timeCpuCycles));
                currentDatap += ( sizeof(mmwData.header.timeCpuCycles) );

                    //get numDetectedObj (4 bytes)
                memcpy( &mmwData.header.numDetectedObj, &currentBufp->at(currentDatap), sizeof(mmwData.header.numDetectedObj));
                currentDatap += ( sizeof(mmwData.header.numDetectedObj) );

                    //get numTLVs (4 bytes)
                memcpy( &mmwData.header.numTLVs, &currentBufp->at(currentDatap), sizeof(mmwData.header.numTLVs));
                currentDatap += ( sizeof(mmwData.header.numTLVs) );

                    //get subFrameNumber (4 bytes) (not used for XWR1443)
                if((mmwData.header.platform & 0xFFFF) != 0x1443)
                {
                    memcpy( &mmwData.header.subFrameNumber, &currentBufp->at(currentDatap), sizeof(mmwData.header.subFrameNumber));
                    currentDatap += ( sizeof(mmwData.header.subFrameNumber) );
                }

                    //if packet lengths do not match, throw it away
                if(mmwData.header.totalPacketLen == currentBufp->size() )
                {
                    sorterState = CHECK_TLV_TYPE;
                }
                else sorterState = SWAP_BUFFERS;

                break;
            }

        case READ_OBJ_STRUCT:
            {
            // CHECK_TLV_TYPE code has already read tlvType and tlvLen

                i = 0;
                offset = 0;

                if (((mmwData.header.version >> 24) & 0xFF) < 3)  // SDK version is older than 3.x
                {
                //get number of objects
                    memcpy( &mmwData.numObjOut, &currentBufp->at(currentDatap), sizeof(mmwData.numObjOut));
                    currentDatap += ( sizeof(mmwData.numObjOut) );

                //get xyzQFormat
                    memcpy( &mmwData.xyzQFormat, &currentBufp->at(currentDatap), sizeof(mmwData.xyzQFormat));
                    currentDatap += ( sizeof(mmwData.xyzQFormat) );
                }
                else  // SDK version is at least 3.x
                {
                    mmwData.numObjOut = mmwData.header.numDetectedObj;
                }

                RScan->header.stamp = static_cast<long int>(clocker.now().seconds());
                RScan->header.frame_id = frameID;
                RScan->height = 1;
                RScan->width = mmwData.numObjOut;
                RScan->is_dense = 1;
                RScan->points.resize(RScan->width * RScan->height);

                // Calculate ratios for max desired elevation and azimuth angles
                if ((maxAllowedElevationAngleDeg >= 0) && (maxAllowedElevationAngleDeg < 90)) {
                    maxElevationAngleRatioSquared = tan(maxAllowedElevationAngleDeg * M_PI / 180.0);
                    maxElevationAngleRatioSquared = maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
                } else maxElevationAngleRatioSquared = -1;
                if ((maxAllowedAzimuthAngleDeg >= 0) && (maxAllowedAzimuthAngleDeg < 90)) maxAzimuthAngleRatio = tan(maxAllowedAzimuthAngleDeg * M_PI / 180.0);
                else maxAzimuthAngleRatio = -1;

                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"maxElevationAngleRatioSquared = %f", maxElevationAngleRatioSquared);
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"maxAzimuthAngleRatio = %f", maxAzimuthAngleRatio);
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"mmwData.numObjOut before = %d", mmwData.numObjOut);

                // Populate pointcloud
                while( i < mmwData.numObjOut ) {
                    if (((mmwData.header.version >> 24) & 0xFF) < 3) { // SDK version is older than 3.x
                        //get object range index
                        memcpy( &mmwData.objOut.rangeIdx, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.rangeIdx));
                        currentDatap += ( sizeof(mmwData.objOut.rangeIdx) );

                        //get object doppler index
                        memcpy( &mmwData.objOut.dopplerIdx, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.dopplerIdx));
                        currentDatap += ( sizeof(mmwData.objOut.dopplerIdx) );

                        //get object peak intensity value
                        memcpy( &mmwData.objOut.peakVal, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.peakVal));
                        currentDatap += ( sizeof(mmwData.objOut.peakVal) );

                        //get object x-coordinate
                        memcpy( &mmwData.objOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.x));
                        currentDatap += ( sizeof(mmwData.objOut.x) );

                        //get object y-coordinate
                        memcpy( &mmwData.objOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.y));
                        currentDatap += ( sizeof(mmwData.objOut.y) );

                        //get object z-coordinate
                        memcpy( &mmwData.objOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.objOut.z));
                        currentDatap += ( sizeof(mmwData.objOut.z) );

                        float temp[7];

                        temp[0] = (float) mmwData.objOut.x;
                        temp[1] = (float) mmwData.objOut.y;
                        temp[2] = (float) mmwData.objOut.z;
                        temp[3] = (float) mmwData.objOut.dopplerIdx;

                        for (int j = 0; j < 4; j++) {
                            if (temp[j] > 32767) temp[j] -= 65536;
                            if (j < 3) temp[j] = temp[j] / pow(2 , mmwData.xyzQFormat);
                        }

                        temp[7] = temp[3] * vvel;

                        temp[4] = (float) mmwData.objOut.rangeIdx * vrange;
                        temp[5] = 10 * log10(mmwData.objOut.peakVal + 1);  // intensity
                        temp[6] = std::atan2(-temp[0], temp[1]) / M_PI * 180;

                        uint16_t tmp = (uint16_t)(temp[3] + nd / 2);

                        // Map mmWave sensor coordinates to ROS coordinate system
                        RScan->points[i].x = temp[1];   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                        RScan->points[i].y = -temp[0];  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                        RScan->points[i].z = temp[2];   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
                        //RScan->points[i].intensity = temp[5];

                        radarscan.frame_id = frameID;
                        radarscan.stamp = static_cast<long int>(clocker.now().seconds());
                        radarscan.point_id = i;
                        radarscan.x = temp[1];
                        radarscan.y = -temp[0];
                        radarscan.z = temp[2];
                        radarscan.range = temp[4];
                        radarscan.velocity = temp[7];
                        radarscan.doppler_bin = tmp;
                        radarscan.bearing = temp[6];
                        radarscan.intensity = temp[5];
                    }
                    else
                    { // SDK version is 3.x+
                        //get object x-coordinate (meters)
                        memcpy( &mmwData.newObjOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.x));
                        currentDatap += ( sizeof(mmwData.newObjOut.x) );

                        //get object y-coordinate (meters)
                        memcpy( &mmwData.newObjOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.y));
                        currentDatap += ( sizeof(mmwData.newObjOut.y) );

                        //get object z-coordinate (meters)
                        memcpy( &mmwData.newObjOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.z));
                        currentDatap += ( sizeof(mmwData.newObjOut.z) );

                        //get object velocity (m/s)
                        memcpy( &mmwData.newObjOut.velocity, &currentBufp->at(currentDatap), sizeof(mmwData.newObjOut.velocity));
                        currentDatap += ( sizeof(mmwData.newObjOut.velocity) );

                        // Map mmWave sensor coordinates to ROS coordinate system
                        RScan->points[i].x = mmwData.newObjOut.y;   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                        RScan->points[i].y = -mmwData.newObjOut.x;  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                        RScan->points[i].z = mmwData.newObjOut.z;   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
                        RScan->points[i].velocity = mmwData.newObjOut.velocity;

                        radarscan.frame_id = frameID;
                        radarscan.stamp = static_cast<long int>(clocker.now().seconds());
                        radarscan.point_id = i;
                        radarscan.x = mmwData.newObjOut.y;
                        radarscan.y = -mmwData.newObjOut.x;
                        radarscan.z = mmwData.newObjOut.z;
                        radarscan.velocity = mmwData.newObjOut.velocity;
                        radar_scan_pub->publish(radarscan);

                        // For SDK 3.x, intensity is replaced by snr in sideInfo and is parsed in the READ_SIDE_INFO code
                    }

                    if (((maxElevationAngleRatioSquared == -1) ||
                       (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                        RScan->points[i].y * RScan->points[i].y)
                       ) < maxElevationAngleRatioSquared)
                       ) &&
                        ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
                        (RScan->points[i].x != 0)
                        )
                    {
                        radar_scan_pub->publish(radarscan);
                    }
                    i++;
                }

                sorterState = CHECK_TLV_TYPE;

                break;
            }

        case READ_SIDE_INFO:
            {
                // Make sure we already received and parsed detected obj list (READ_OBJ_STRUCT)
                if (mmwData.numObjOut > 0)
                {
                    for (i = 0; i < mmwData.numObjOut; i++)
                    {
                        //get snr (unit is 0.1 steps of dB)
                        memcpy( &mmwData.sideInfo.snr, &currentBufp->at(currentDatap), sizeof(mmwData.sideInfo.snr));
                        currentDatap += ( sizeof(mmwData.sideInfo.snr) );
                        //get noise (unit is 0.1 steps of dB)
                        memcpy( &mmwData.sideInfo.noise, &currentBufp->at(currentDatap), sizeof(mmwData.sideInfo.noise));
                        currentDatap += ( sizeof(mmwData.sideInfo.noise) );
                        RScan->points[i].intensity = (float) mmwData.sideInfo.snr / 10.0;   // Use snr for "intensity" field (divide by 10 since unit of snr is 0.1dB)
                    }
                }
                else  // else just skip side info section if we have not already received and parsed detected obj list
                {
                    i = 0;
                    while (i++ < tlvLen - 1)
                    {
                        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Parsing Side Info i=%d and tlvLen = %u", i, tlvLen);
                    }
                    currentDatap += tlvLen;
                }

                sorterState = CHECK_TLV_TYPE;
                break;
            }

        case READ_OCCUPANCY:
            {

                RScan->header.frame_id = frameID;
                RScan->height = 1;
                RScan->width = mmwData.numObjOut;
                RScan->is_dense = 1;
                RScan->points.resize(RScan->width * RScan->height);

                //get Occupancy State which is a uint32, 0 means that zone is unoccupied. Anything else means the stop zone is occupied
                memcpy( &mmwData.occupancy.state, &currentBufp->at(currentDatap), sizeof(mmwData.occupancy.state));
                currentDatap += ( sizeof(mmwData.occupancy.state) );

                radaroccupancy.state = mmwData.occupancy.state;

                if (radaroccupancy.state == 0){
    	      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Area is clear!");
                }
                else{
    	      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Area is OCCUPIED!");
                }

                radar_occupancy_pub->publish(radaroccupancy);

                sorterState = CHECK_TLV_TYPE;

                break;
            }

        case READ_SPHERE_POINT_CLOUD:
            {
                // CHECK_TLV_TYPE code has already read tlvType and tlvLen
                i = 0;
                offset = 0;

                mmwData.numObjOut = mmwData.header.numDetectedObj;
                RScan->header.frame_id = frameID;
                RScan->height = 1;
                RScan->width = mmwData.numObjOut;
                RScan->is_dense = 1;
                RScan->points.resize(RScan->width * RScan->height);

            // Calculate ratios for max desired elevation and azimuth angles
                if ((maxAllowedElevationAngleDeg >= 0) && (maxAllowedElevationAngleDeg < 90)) {
                    maxElevationAngleRatioSquared = tan(maxAllowedElevationAngleDeg * M_PI / 180.0);
                    maxElevationAngleRatioSquared = maxElevationAngleRatioSquared * maxElevationAngleRatioSquared;
                } else maxElevationAngleRatioSquared = -1;
                if ((maxAllowedAzimuthAngleDeg >= 0) && (maxAllowedAzimuthAngleDeg < 90)) maxAzimuthAngleRatio = tan(maxAllowedAzimuthAngleDeg * M_PI / 180.0);
                else maxAzimuthAngleRatio = -1;

                //get object Elevation unit (rad)
                memcpy( &mmwData.newSphereCloudOut.elevationUnit, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.elevationUnit));
                currentDatap += ( sizeof(mmwData.newSphereCloudOut.elevationUnit) );

                //get object Azimuth unit (rad)
                memcpy( &mmwData.newSphereCloudOut.azimuthUnit, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.azimuthUnit));
                currentDatap += ( sizeof(mmwData.newSphereCloudOut.azimuthUnit) );

                //get object Doppler unit (m/s)
                memcpy( &mmwData.newSphereCloudOut.dopplerUnit, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.dopplerUnit));
                currentDatap += ( sizeof(mmwData.newSphereCloudOut.dopplerUnit) );

                //get object Range unit (meters)
                memcpy( &mmwData.newSphereCloudOut.rangeUnit, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.rangeUnit));
                currentDatap += ( sizeof(mmwData.newSphereCloudOut.rangeUnit) );

                //get object SNR unit (ratio)
                memcpy( &mmwData.newSphereCloudOut.snrUnit, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.snrUnit));
                currentDatap += ( sizeof(mmwData.newSphereCloudOut.snrUnit) );

                while( i < mmwData.numObjOut ) {

                    //get Elevation value to multiply by
                    memcpy( &mmwData.newSphereCloudOut.elevation, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.elevation));
                    currentDatap += ( sizeof(mmwData.newSphereCloudOut.elevation) );

                    //get Azimuth value to multiply by
                    memcpy( &mmwData.newSphereCloudOut.azimuth, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.azimuth));
                    currentDatap += ( sizeof(mmwData.newSphereCloudOut.azimuth) );

                    //get Doppler value to multiply by
                    memcpy( &mmwData.newSphereCloudOut.doppler, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.doppler));
                    currentDatap += ( sizeof(mmwData.newSphereCloudOut.doppler) );

                    //get Range value to multiply by
                    memcpy( &mmwData.newSphereCloudOut.range, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.range));
                    currentDatap += ( sizeof(mmwData.newSphereCloudOut.range) );

                    //get SNR value to multiply by
                    memcpy( &mmwData.newSphereCloudOut.snr, &currentBufp->at(currentDatap), sizeof(mmwData.newSphereCloudOut.snr));
                    currentDatap += ( sizeof(mmwData.newSphereCloudOut.snr) );

                    //multiple sensor output value by unit value to decompress data
                    realElevation = mmwData.newSphereCloudOut.elevationUnit * mmwData.newSphereCloudOut.elevation;
                    realAzimuth = mmwData.newSphereCloudOut.azimuthUnit * mmwData.newSphereCloudOut.azimuth;
                    realDoppler = mmwData.newSphereCloudOut.dopplerUnit * mmwData.newSphereCloudOut.doppler;
                    realRange = mmwData.newSphereCloudOut.rangeUnit * mmwData.newSphereCloudOut.range;
                    realSNR =  mmwData.newSphereCloudOut.snrUnit * mmwData.newSphereCloudOut.snr;

                    // Map mmWave sensor coordinates to ROS coordinate system while also converting from spherical to cartesian
                    RScan->points[i].x = realRange * cos(realAzimuth) * cos(realElevation);   // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                    RScan->points[i].y = -1 * realRange * sin(realAzimuth) * cos(realElevation);  // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                    RScan->points[i].z = realRange * sin(realElevation);   // ROS standard coordinate system Z-axis is up which is the same as mmWave sensor Z-axis
                    RScan->points[i].velocity = realDoppler;
                    RScan->points[i].intensity = realSNR;

                    radarscan.frame_id = frameID;
                    radarscan.stamp = clocker.now().seconds();

                    radarscan.point_id = i;
                    radarscan.x = realRange * cos(realAzimuth) * cos(realElevation);
                    radarscan.y = -1 * realRange * sin(realAzimuth) * cos(realElevation);
                    radarscan.z = realRange * sin(realElevation);
                    radarscan.range = realRange;
                    radarscan.velocity = realDoppler;
                    radarscan.intensity = realSNR;

                    if (((maxElevationAngleRatioSquared == -1) ||
                     (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                        RScan->points[i].y * RScan->points[i].y)
                     ) < maxElevationAngleRatioSquared)
                     ) &&
                        ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
                        (RScan->points[i].x != 0)
                        )
                    {
                        radar_scan_pub->publish(radarscan);
                    }
                    i++;

                }

                sorterState = CHECK_TLV_TYPE;
                break;
            }

        case READ_3D_TARGET_LIST:
            {

                i = 0;
                offset = 0;
                mmwData.numObjOut = mmwData.header.numDetectedObj;
                radartrackarray.frame_id = frameID;
                radartrackarray.stamp = clocker.now().seconds();
                radartrackarray.num_tracks = (int) tlvLen / 112;

                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Number of Tracks is: %d",(tlvLen / 112));
                while( i < radartrackarray.num_tracks ) {

                    //get Track ID
                    memcpy( &mmwData.newListOut.tid, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.tid));
                    currentDatap += ( sizeof(mmwData.newListOut.tid) );

                    //get Track position in X dimension (m)
                    memcpy( &mmwData.newListOut.posX, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.posX));
                    currentDatap += ( sizeof(mmwData.newListOut.posX) );

                    //get Track position in Y dimension (m)
                    memcpy( &mmwData.newListOut.posY, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.posY));
                    currentDatap += ( sizeof(mmwData.newListOut.posY) );

                    //get Track position in Z dimension (m)
                    memcpy( &mmwData.newListOut.posZ, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.posZ));
                    currentDatap += ( sizeof(mmwData.newListOut.posZ) );

                    //get Track velocity in X dimension (m)
                    memcpy( &mmwData.newListOut.velX, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.velX));
                    currentDatap += ( sizeof(mmwData.newListOut.velX) );

                    //get Track velocity in Y dimension (m)
                    memcpy( &mmwData.newListOut.velY, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.velY));
                    currentDatap += ( sizeof(mmwData.newListOut.velY) );

                    //get Track velocity in Z dimension (m)
                    memcpy( &mmwData.newListOut.velZ, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.velZ));
                    currentDatap += ( sizeof(mmwData.newListOut.velZ) );

                    //get Track acceleration in X dimension (m)
                    memcpy( &mmwData.newListOut.accX, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.accX));
                    currentDatap += ( sizeof(mmwData.newListOut.accX) );

                    //get Track acceleration in Y dimension (m)
                    memcpy( &mmwData.newListOut.accY, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.accY));
                    currentDatap += ( sizeof(mmwData.newListOut.accY) );

                    //get Track acceleration in Z dimension (m)
                    memcpy( &mmwData.newListOut.accZ, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.accZ));
                    currentDatap += ( sizeof(mmwData.newListOut.accZ) );

                    //Throw Away
                    memcpy( &mmwData.newListOut.ec, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.ec));
                    currentDatap += ( sizeof(mmwData.newListOut.ec) );

                    //Throw Away
                    memcpy( &mmwData.newListOut.g, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.g));
                    currentDatap += ( sizeof(mmwData.newListOut.g) );

                    //Throw Away
                    memcpy( &mmwData.newListOut.confidenceLevel, &currentBufp->at(currentDatap), sizeof(mmwData.newListOut.confidenceLevel));
                    currentDatap += ( sizeof(mmwData.newListOut.confidenceLevel) );

                    radartrackcontents.frame_id = frameID;
                    radartrackcontents.stamp = clocker.now().seconds();
                    radartrackcontents.tid = mmwData.newListOut.tid;
                    radartrackcontents.posx = mmwData.newListOut.posY;  // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                    radartrackcontents.posy = -mmwData.newListOut.posX; // ROS standard coordinate system Y-axis is left which is the mmWave sensor -(X-axis)
                    radartrackcontents.posz = mmwData.newListOut.posZ;
                    radartrackcontents.velx = mmwData.newListOut.velY;
                    radartrackcontents.vely = -mmwData.newListOut.velX;
                    radartrackcontents.velz = mmwData.newListOut.velZ;
                    radartrackcontents.accx = mmwData.newListOut.accY;
                    radartrackcontents.accy = -mmwData.newListOut.accX;
                    radartrackcontents.accz = mmwData.newListOut.accZ;
                    radartrackarray.track.push_back(radartrackcontents);
                    i++;

                }
                radar_trackarray_pub->publish(radartrackarray);
                radartrackarray.track.clear();
                sorterState = CHECK_TLV_TYPE;
                break;
            }

        case READ_COMPRESSED_POINT_CLOUD:
            {
                // CHECK_TLV_TYPE code has already read tlvType and tlvLen
                i = 0;

                //get object xyz unit
                memcpy( &mmwData.newPointCloudCompOut.xyzUnit, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.xyzUnit));
                currentDatap += ( sizeof(mmwData.newPointCloudCompOut.xyzUnit) );

                //get object doppler unit (rad)
                memcpy( &mmwData.newPointCloudCompOut.dopplerUnit, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.dopplerUnit));
                currentDatap += ( sizeof(mmwData.newPointCloudCompOut.dopplerUnit) );

                //get object snr unit (m/s)
                memcpy( &mmwData.newPointCloudCompOut.snrUnit, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.snrUnit));
                currentDatap += ( sizeof(mmwData.newPointCloudCompOut.snrUnit) );

                //get object noise unit (meters)
                memcpy( &mmwData.newPointCloudCompOut.noiseUnit, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.noiseUnit));
                currentDatap += ( sizeof(mmwData.newPointCloudCompOut.noiseUnit) );

                //get number of detected objects
                memcpy( &mmwData.newPointCloudCompOut.numDetectedPoints, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.numDetectedPoints));
                currentDatap += ( sizeof(mmwData.newPointCloudCompOut.numDetectedPoints) );

                RScan->header.frame_id = frameID;
                RScan->height = 1;
                RScan->width = (mmwData.newPointCloudCompOut.numDetectedPoints[0] + mmwData.newPointCloudCompOut.numDetectedPoints[1]);
                RScan->is_dense = 1;
                RScan->points.resize(RScan->width * RScan->height);

                while(i < (mmwData.newPointCloudCompOut.numDetectedPoints[0] + mmwData.newPointCloudCompOut.numDetectedPoints[1])) {

                    //get x value
                    memcpy( &mmwData.newPointCloudCompOut.x, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.x));
                    currentDatap += ( sizeof(mmwData.newPointCloudCompOut.x) );

                    //get y value
                    memcpy( &mmwData.newPointCloudCompOut.y, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.y));
                    currentDatap += ( sizeof(mmwData.newPointCloudCompOut.y) );

                    //get z value
                    memcpy( &mmwData.newPointCloudCompOut.z, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.z));
                    currentDatap += ( sizeof(mmwData.newPointCloudCompOut.z) );

                    //get Doppler value
                    memcpy( &mmwData.newPointCloudCompOut.doppler, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.doppler));
                    currentDatap += ( sizeof(mmwData.newPointCloudCompOut.doppler) );

                    //get SNR value
                    memcpy( &mmwData.newPointCloudCompOut.snr, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.snr));
                    currentDatap += ( sizeof(mmwData.newPointCloudCompOut.snr) );

                    //get Noise value
                    memcpy( &mmwData.newPointCloudCompOut.noise, &currentBufp->at(currentDatap), sizeof(mmwData.newPointCloudCompOut.noise));
                    currentDatap += ( sizeof(mmwData.newPointCloudCompOut.noise) );

                    //multiple sensor output value by unit value to decompress data
                    realX = mmwData.newPointCloudCompOut.xyzUnit * mmwData.newPointCloudCompOut.x;
                    realY = mmwData.newPointCloudCompOut.xyzUnit * mmwData.newPointCloudCompOut.y;
                    realZ = mmwData.newPointCloudCompOut.xyzUnit * mmwData.newPointCloudCompOut.z;
                    realDoppler = mmwData.newPointCloudCompOut.dopplerUnit * mmwData.newPointCloudCompOut.doppler;
                    realSNR = mmwData.newPointCloudCompOut.snrUnit * mmwData.newPointCloudCompOut.snr;
                    realNoise = mmwData.newPointCloudCompOut.noiseUnit * mmwData.newPointCloudCompOut.noise;

                    //Map mmWave sensor coordinates to ROS coordinate system while also converting from spherical to cartesian
                    RScan->points[i].x =  realY; // ROS standard coordinate system X-axis is forward which is the mmWave sensor Y-axis
                    RScan->points[i].y = -(realX); // ROS standard coordinate system Y-axis is left which is the mmWave sensor -X-axis
                    RScan->points[i].z =  realZ;
                    RScan->points[i].velocity = realDoppler;
                    RScan->points[i].intensity = realSNR;
                    radarscan.frame_id = frameID;
                    radarscan.stamp = clocker.now().seconds();
                    radarscan.point_id = i;
                    radarscan.x = realY;
                    radarscan.y = -(realX);
                    radarscan.z = realZ;
                    radarscan.range = realRange;
                    radarscan.velocity = realDoppler;
                    radarscan.intensity = realSNR;

                    if (((maxElevationAngleRatioSquared == -1) ||
                     (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                        RScan->points[i].y * RScan->points[i].y)
                     ) < maxElevationAngleRatioSquared)
                     ) &&
                        ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
                        (RScan->points[i].x != 0)
                        )
                    {
                        radar_scan_pub->publish(radarscan);
                    }

                    radar_scan_pub->publish(radarscan);
                    i++;
                }
                tlvSize = 0;
                sorterState = CHECK_TLV_TYPE;
                break;
            }

        case READ_TARGET_INDEX:

            i = 0;
            offset = 0;

            mmwData.numObjOut = mmwData.header.numDetectedObj;

            RScan->header.frame_id = frameID;
            RScan->height = 1;
            RScan->width = mmwData.numObjOut;
            RScan->is_dense = 1;
            RScan->points.resize(RScan->width * RScan->height);

            while( i < tlvLen ) {

                //get point's associated Track (int)
                memcpy( &mmwData.newIndexOut.targetID, &currentBufp->at(currentDatap), sizeof(mmwData.newIndexOut.targetID));
                currentDatap += ( sizeof(mmwData.newIndexOut.targetID) );

        //      throw away first track in order to allign with PointCloud
        //  if (frameID == 0){
                radartrackid.frame_id = frameID;
        //  }
        //  else{
        //     radartrackid.header.frame_id = frameID - 1;
        //  }
                radartrackid.stamp = clocker.now().seconds();
                radartrackid.tid = mmwData.newIndexOut.targetID;

                radar_trackid_pub->publish(radartrackid);
                i++;
            }

            sorterState = CHECK_TLV_TYPE;
            break;

        case READ_LOG_MAG_RANGE:
            {
                sorterState = CHECK_TLV_TYPE;
                break;
            }

        case READ_NOISE:
            {
                i = 0;
                while (i++ < tlvLen - 1)
                {
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Parsing Noise Profile i=%d and tlvLen = %u", i, tlvLen);
                }
                currentDatap += tlvLen;
                sorterState = CHECK_TLV_TYPE;
                break;
            }

        case READ_AZIMUTH:
            {
                i = 0;
                while (i++ < tlvLen - 1)
                {
                 //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Parsing Azimuth Profile i=%d and tlvLen = %u", i, tlvLen);
                }
                currentDatap += tlvLen;
                sorterState = CHECK_TLV_TYPE;
                break;
            }

        case READ_DOPPLER:
            {

                i = 0;
                while (i++ < tlvLen - 1)
                {
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Parsing Doppler Profile i=%d and tlvLen = %u", i, tlvLen);
                }

                currentDatap += tlvLen;

                sorterState = CHECK_TLV_TYPE;
                break;
            }

        case READ_STATS:
            {
                i = 0;
                while (i++ < tlvLen - 1)
                {
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Parsing Stats Profile i=%d and tlvLen = %u", i, tlvLen);
                }
                currentDatap += tlvLen;
                sorterState = CHECK_TLV_TYPE;
                break;
            }

        case CHECK_TLV_TYPE:
            {
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : tlvCount = %d, numTLV = %d", tlvCount, mmwData.header.numTLVs);
                if(tlvCount++ >= mmwData.header.numTLVs)  // Done parsing all received TLV sections
                {
                        // Publish detected object pointcloud
                    if (mmwData.numObjOut > 0)
                    {
                        j = 0;
                        for (i = 0; i < mmwData.numObjOut; i++)
                        {
                                // Keep point if elevation and azimuth angles are less than specified max values
                                // (NOTE: The following calculations are done using ROS standard coordinate system axis definitions where X is forward and Y is left)
                            if (((maxElevationAngleRatioSquared == -1) ||
                               (((RScan->points[i].z * RScan->points[i].z) / (RScan->points[i].x * RScan->points[i].x +
                                RScan->points[i].y * RScan->points[i].y)
                               ) < maxElevationAngleRatioSquared)
                               ) &&
                                ((maxAzimuthAngleRatio == -1) || (fabs(RScan->points[i].y / RScan->points[i].x) < maxAzimuthAngleRatio)) &&
                                (RScan->points[i].x != 0)
                                )
                            {
                                    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Kept point");
                                    // copy: points[i] => points[j]
                                memcpy( &RScan->points[j], &RScan->points[i], sizeof(RScan->points[i]));
                                j++;
                            }
                        }
                        mmwData.numObjOut = j;  // update number of objects as some points may have been removed
                        // Resize point cloud since some points may have been removed
                        //RScan->width = mmwData.numObjOut;
                        RScan->points.resize(RScan->width * RScan->height);
                        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread: number of obj = %d", mmwData.numObjOut );
                    }
                    pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    pcl::toROSMsg(*RScan, *pc2_msg_);
                    pc2_msg_->header.frame_id = "map";
                    DataUARTHandler_pub->publish(*pc2_msg_);
                    sorterState = SWAP_BUFFERS;
                }
                else  // More TLV sections to parse
                {
                    //get tlvType (32 bits) & remove from queue
                    memcpy( &tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
                    currentDatap += ( sizeof(tlvType) );

                    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : sizeof(tlvType) = %d", sizeof(tlvType));
                    //get tlvLen (32 bits) & remove from queue
                    memcpy( &tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
                    currentDatap += ( sizeof(tlvLen) );

                    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : sizeof(tlvLen) = %d", sizeof(tlvLen));

                    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : tlvType = %d, tlvLen = %d", (int) tlvType, tlvLen);

                    switch(tlvType)
                    {
                    case MMWDEMO_OUTPUT_MSG_NULL:
                        break;

                    case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Object TLV");
                        sorterState = READ_OBJ_STRUCT;
                        break;

                    case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Range TLV");
                        sorterState = READ_LOG_MAG_RANGE;
                        break;

                    case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Noise TLV");
                        sorterState = READ_NOISE;
                        break;

                    case MMWDEMO_OUTPUT_MSG_AZIMUTH_STATIC_HEAT_MAP:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Azimuth Heat TLV");
                        sorterState = READ_AZIMUTH;
                        break;

                    case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : R/D Heat TLV");
                        sorterState = READ_DOPPLER;
                        break;

                    case MMWDEMO_OUTPUT_MSG_STATS:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Stats TLV");
                        sorterState = READ_STATS;
                        break;

                    case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Side info TLV");
                        sorterState = READ_SIDE_INFO;
                        break;

                    case MMWDEMO_OUTPUT_MSG_MAX:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Header TLV");
                        sorterState = READ_HEADER;
                        break;

                    case MMWDEMO_OUTPUT_MSG_OCCUPANCY_STATE_MACHINE:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Occupancy State Machine TLV");
                        sorterState = READ_OCCUPANCY;
                        break;

                    case MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Compressed Points TLV");
                        sorterState = READ_SPHERE_POINT_CLOUD;
                        break;

                    case MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : 3D Target List TLV");
                        sorterState = READ_3D_TARGET_LIST;
                        break;

                    case MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Target Index TLV");
                        sorterState = READ_TARGET_INDEX;
                        break;

                    case MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS:
                            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"DataUARTHandler Sort Thread : Compressed Points TLV MMWAVE-L SDK 5.x");
                        sorterState = READ_COMPRESSED_POINT_CLOUD;
                        break;

                    default:
                        break;
                    }
                }
                break;
            }

        case SWAP_BUFFERS:
            {
                pthread_mutex_lock(&countSync_mutex);
                pthread_mutex_unlock(&currentBufp_mutex);

                countSync++;

                if(countSync == COUNT_SYNC_MAX)
                {
                    pthread_cond_signal(&countSync_max_cv);
                }

                pthread_cond_wait(&sort_go_cv, &countSync_mutex);

                pthread_mutex_unlock(&countSync_mutex);
                pthread_mutex_lock(&currentBufp_mutex);

                currentDatap = 0;
                tlvCount = 0;

                sorterState = READ_HEADER;

                break;
            }
        default:
            break;
        }
    }
    pthread_exit(NULL);
}

void DataUARTHandler::start(void)
{

    pthread_t uartThread, sorterThread, swapThread;
    int  iret1, iret2, iret3;
    sigset_t set;
    int s;

    /* Block SIGINT on main thread and subsequently created threads */
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    s = pthread_sigmask(SIG_BLOCK, &set, NULL);
    pthread_mutex_init(&countSync_mutex, NULL);
    pthread_mutex_init(&nextBufp_mutex, NULL);
    pthread_mutex_init(&currentBufp_mutex, NULL);
    pthread_cond_init(&countSync_max_cv, NULL);
    pthread_cond_init(&read_go_cv, NULL);
    pthread_cond_init(&sort_go_cv, NULL);

    countSync = 0;

    /* Create independent threads each of which will execute function */
    iret1 = pthread_create( &uartThread, NULL, this->readIncomingData_helper, this);
    if(iret1)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Error - pthread_create() return code: %d\n",iret1);
        rclcpp::shutdown();
    }

    iret2 = pthread_create( &sorterThread, NULL, this->sortIncomingData_helper, this);
    if(iret2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Error - pthread_create() return code: %d\n",iret1);
        rclcpp::shutdown();
    }

    iret3 = pthread_create( &swapThread, NULL, this->syncedBufferSwap_helper, this);
    if(iret3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Error - pthread_create() return code: %d\n",iret1);
        rclcpp::shutdown();
    }

        /* Unlock SIGINT on main thread */
    s = pthread_sigmask(SIG_UNBLOCK, &set, NULL);

    signal(SIGINT, sigHandler);

    pthread_join(uartThread, NULL);
    pthread_join(sorterThread, NULL);
    pthread_join(swapThread, NULL);

    pthread_mutex_destroy(&countSync_mutex);
    pthread_mutex_destroy(&nextBufp_mutex);
    pthread_mutex_destroy(&currentBufp_mutex);
    pthread_cond_destroy(&countSync_max_cv);
    pthread_cond_destroy(&read_go_cv);
    pthread_cond_destroy(&sort_go_cv);
}

void* DataUARTHandler::readIncomingData_helper(void *context)
{
    return (static_cast<DataUARTHandler*>(context)->readIncomingData());
}

void* DataUARTHandler::sortIncomingData_helper(void *context)
{
    return (static_cast<DataUARTHandler*>(context)->sortIncomingData());
}

void* DataUARTHandler::syncedBufferSwap_helper(void *context)
{
    return (static_cast<DataUARTHandler*>(context)->syncedBufferSwap());
}

void DataUARTHandler::stop()
{
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Stopping Threads");

    stop_threads = true;
    rclcpp::shutdown();

    pthread_cond_signal(&read_go_cv);
    pthread_cond_signal(&sort_go_cv);
    pthread_cond_signal(&countSync_max_cv);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataUARTHandlerNode>();
    std::string mySerialPort = node->get_parameter("data_port").as_string();
    std::string myBaudRate = node->get_parameter("data_rate").as_string();
    std::string myFrameID = "/ti_mmwave_0";
    DataUARTHandler d(node);
    d.setFrameID(myFrameID.c_str());
    d.setUARTPort(mySerialPort.c_str());
    d.setBaudRate(std::stoi(myBaudRate));
    d.setMaxAllowedElevationAngleDeg(90);
    d.setMaxAllowedAzimuthAngleDeg(90);
    d.start();
    rclcpp::spin(node);
    rclcpp::shutdown();
}