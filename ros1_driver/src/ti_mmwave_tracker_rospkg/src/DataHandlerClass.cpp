#include <DataHandlerClass.h>
#define PCL_NO_PRECOMPILE

//#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

DataUARTHandler* gDataHandlerPtr;

void DataUARTHandler::sigHandler(int32_t sig)
{
    switch(sig)
    {
    case SIGINT:
        gDataHandlerPtr->stop();

    }
    
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

DataUARTHandler::DataUARTHandler(ros::NodeHandle* nh) : currentBufp(&pingPongBuffers[0]) , nextBufp(&pingPongBuffers[1]) {

    // Make sure to add ALL publishers below to DataHandlerClass.h as an include at the top
    // File will be generated during catkin_make, use existing publishers to understand syntax
  
    DataUARTHandler_pub = nh->advertise<sensor_msgs::PointCloud2>("/ti_mmwave/radar_scan_pcl", 100);
    radar_scan_pub = nh->advertise<ti_mmwave_tracker_rospkg::RadarScan>("/ti_mmwave/radar_scan", 100);
    radar_trackid_pub = nh->advertise<ti_mmwave_tracker_rospkg::RadarPointTrackID>("/ti_mmwave/radar_trackid", 100);
    radar_trackarray_pub = nh->advertise<ti_mmwave_tracker_rospkg::RadarTrackArray>("/ti_mmwave/radar_trackarray", 100);
    marker_pub = nh->advertise<visualization_msgs::Marker>("/ti_mmwave/radar_scan_markers", 100);
    
    maxAllowedElevationAngleDeg = 90; // Use max angle if none specified
    maxAllowedAzimuthAngleDeg = 90; // Use max angle if none specified

    // Wait for parameters
    while(!nh->hasParam("/ti_mmwave/doppler_vel_resolution")){}

    nh->getParam("/ti_mmwave/numAdcSamples", nr);
    nh->getParam("/ti_mmwave/numLoops", nd);
    nh->getParam("/ti_mmwave/num_TX", ntx);
    nh->getParam("/ti_mmwave/f_s", fs);
    nh->getParam("/ti_mmwave/f_c", fc);
    nh->getParam("/ti_mmwave/BW", BW);
    nh->getParam("/ti_mmwave/PRI", PRI);
    nh->getParam("/ti_mmwave/t_fr", tfr);
    nh->getParam("/ti_mmwave/max_range", max_range);
    nh->getParam("/ti_mmwave/range_resolution", vrange);
    nh->getParam("/ti_mmwave/max_doppler_vel", max_vel);
    nh->getParam("/ti_mmwave/doppler_vel_resolution", vvel);

    ROS_INFO("\n\n==============================\nList of parameters\n==============================\nNumber of range samples: %d\nNumber of chirps: %d\nf_s: %.3f MHz\nf_c: %.3f GHz\nBandwidth: %.3f MHz\nPRI: %.3f us\nFrame time: %.3f ms\nMax range: %.3f m\nRange resolution: %.3f m\nMax Doppler: +-%.3f m/s\nDoppler resolution: %.3f m/s\n==============================\n", \
        nr, nd, fs/1e6, fc/1e9, BW/1e6, PRI*1e6, tfr*1e3, max_range, vrange, max_vel/2, vvel);

    gDataHandlerPtr = this;

    stop_threads = false;
}

void DataUARTHandler::setFrameID(char* myFrameID)
{
    frameID = myFrameID;
}

/*Implementation of setUARTPort*/
void DataUARTHandler::setUARTPort(char* mySerialPort)
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
    serial::Serial mySerialObject("", dataBaudRate, serial::Timeout::simpleTimeout(100));
    mySerialObject.setPort(dataSerialPort);
    try
    {
        mySerialObject.open();
    } catch (std::exception &e1) {
        ROS_INFO("DataUARTHandler Read Thread: Failed to open Data serial port with error: %s", e1.what());
        ROS_INFO("DataUARTHandler Read Thread: Waiting 20 seconds before trying again...");
        try
        {
            // Wait 20 seconds and try to open serial port again
            ros::Duration(20).sleep();
            mySerialObject.open();
        } catch (std::exception &e2) {
            ROS_ERROR("DataUARTHandler Read Thread: Failed second time to open Data serial port, error: %s", e1.what());
            ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened. Port is \"%s\" and baud rate is %d", dataSerialPort, dataBaudRate);
            pthread_exit(NULL);
        }
    }
    
    if(mySerialObject.isOpen())
        ROS_INFO("DataUARTHandler Read Thread: Port is open");
    else
        ROS_ERROR("DataUARTHandler Read Thread: Port could not be opened");
    
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
    
    while(ros::ok())
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
        
        nextBufp->push_back( last8Bytes[7] );  //push byte onto buffer
        
        //ROS_INFO("DataUARTHandler Read Thread: last8bytes = %02x%02x %02x%02x %02x%02x %02x%02x",  last8Bytes[7], last8Bytes[6], last8Bytes[5], last8Bytes[4], last8Bytes[3], last8Bytes[2], last8Bytes[1], last8Bytes[0]);
        
        /*If a magicWord is found wait for sorting to finish and switch buffers*/
        if( isMagicWord(last8Bytes) )
        {
            //ROS_INFO("Found magic word");
        
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
    while(ros::ok())
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

void *DataUARTHandler::sortIncomingData( void )
{
    MmwDemo_Output_TLV_Types tlvType = MMWDEMO_OUTPUT_MSG_NULL;
    uint32_t tlvLen = 0;
    uint32_t headerSize;
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
    
    //boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> RScan(new pcl::PointCloud<pcl::PointXYZI>);
    boost::shared_ptr<pcl::PointCloud<mmWaveCloudType>> RScan(new pcl::PointCloud<mmWaveCloudType>);
    ti_mmwave_tracker_rospkg::RadarScan radarscan;
    ti_mmwave_tracker_rospkg::RadarPointTrackID radartrackid;
    ti_mmwave_tracker_rospkg::RadarTrackArray radartrackarray;
    ti_mmwave_tracker_rospkg::RadarTrackContents radartrackcontents;

    //wait for first packet to arrive
    pthread_mutex_lock(&countSync_mutex);
    pthread_cond_wait(&sort_go_cv, &countSync_mutex);
    pthread_mutex_unlock(&countSync_mutex);
    
    pthread_mutex_lock(&currentBufp_mutex);
    
    while(ros::ok())
    {

        switch(sorterState)
        {
            
        case READ_HEADER:
                  
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
            if(currentBufp->size() < headerSize) {
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
            if((mmwData.header.platform & 0xFFFF) != 0x1443) {
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
            
        case READ_SPHERE_POINT_CLOUD:

	  // CHECK_TLV_TYPE code has already read tlvType and tlvLen
            i = 0;
            offset = 0;
            
            mmwData.numObjOut = mmwData.header.numDetectedObj;	    
            pcl_conversions::toPCL(ros::Time::now(), RScan->header.stamp);
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

                    radarscan.header.frame_id = frameID;
                    radarscan.header.stamp = ros::Time::now();

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
			radar_scan_pub.publish(radarscan);
		      }
		    i++;

	    }

	    sorterState = CHECK_TLV_TYPE;
	    break;
	  

	case READ_3D_TARGET_LIST:

	    i = 0;
            offset = 0;

	    
	    
            pcl_conversions::toPCL(ros::Time::now(), RScan->header.stamp);
            mmwData.numObjOut = mmwData.header.numDetectedObj;
	    radartrackarray.header.frame_id = frameID;
            radartrackarray.header.stamp = ros::Time::now();
            radartrackarray.num_tracks = (int) tlvLen / 112;

	    //ROS_INFO("Number of Tracks is: %d",(tlvLen / 112));
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

		    //throw away first track in order to allign with PointCloud
		    /*
		    if (frameID == 0){
		      radartrack.header.frame_id = frameID;
		    }
		    else{
		      radartrack.header.frame_id = frameID - 1;
		    }
		    */

		    radartrackcontents.header.frame_id = frameID;
		    radartrackcontents.header.stamp = ros::Time::now();
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
	    radar_trackarray_pub.publish(radartrackarray);
	    radartrackarray.track.clear();

	    sorterState = CHECK_TLV_TYPE;
	    break;
	  

	case READ_TARGET_INDEX:

	    i = 0;
            offset = 0;
            
            mmwData.numObjOut = mmwData.header.numDetectedObj;
            
            pcl_conversions::toPCL(ros::Time::now(), RScan->header.stamp);
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
		    radartrackid.header.frame_id = frameID;
		//  }
	    //  else{
		//     radartrackid.header.frame_id = frameID - 1;
	    //  }
		    radartrackid.header.stamp = ros::Time::now();
		    radartrackid.tid = mmwData.newIndexOut.targetID;
		    
		    radar_trackid_pub.publish(radartrackid);
		    i++;
	    }

	    sorterState = CHECK_TLV_TYPE;
	    break;
        
        case CHECK_TLV_TYPE:
        
            //ROS_INFO("DataUARTHandler Sort Thread : tlvCount = %d, numTLV = %d", tlvCount, mmwData.header.numTLVs);
        
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
                            //ROS_INFO("Kept point");
                            // copy: points[i] => points[j]
                            memcpy( &RScan->points[j], &RScan->points[i], sizeof(RScan->points[i]));
                            j++;
                        }
                    }
                    mmwData.numObjOut = j;  // update number of objects as some points may have been removed

                    // Resize point cloud since some points may have been removed
                    RScan->width = mmwData.numObjOut;
                    RScan->points.resize(RScan->width * RScan->height);
                    
                    //ROS_INFO("mmwData.numObjOut after = %d", mmwData.numObjOut);
                    //ROS_INFO("DataUARTHandler Sort Thread: number of obj = %d", mmwData.numObjOut );
                }
                DataUARTHandler_pub.publish(RScan);

                //ROS_INFO("DataUARTHandler Sort Thread : CHECK_TLV_TYPE state says tlvCount max was reached, going to switch buffer state");
                sorterState = SWAP_BUFFERS;
            }
            
            else  // More TLV sections to parse
            {
               //get tlvType (32 bits) & remove from queue
                memcpy( &tlvType, &currentBufp->at(currentDatap), sizeof(tlvType));
                currentDatap += ( sizeof(tlvType) );
                
            
                //get tlvLen (32 bits) & remove from queue
                memcpy( &tlvLen, &currentBufp->at(currentDatap), sizeof(tlvLen));
                currentDatap += ( sizeof(tlvLen) );
                ;
            
                switch(tlvType)
                {
                case MMWDEMO_OUTPUT_MSG_NULL:
                
                    break;
                
                case MMWDEMO_OUTPUT_MSG_COMPRESSED_POINTS:
                    //ROS_INFO("DataUARTHandler Sort Thread : Compressed Points TLV");
                    sorterState = READ_SPHERE_POINT_CLOUD;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST:
                    //ROS_INFO("DataUARTHandler Sort Thread : 3D Target List TLV");
                    sorterState = READ_3D_TARGET_LIST;
                    break;
                
                case MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX:
                    //ROS_INFO("DataUARTHandler Sort Thread : Target Index TLV");
                    sorterState = READ_TARGET_INDEX;
                    break;		  
		    
                case MMWDEMO_OUTPUT_MSG_MAX:
                    //ROS_INFO("DataUARTHandler Sort Thread : Header TLV");
                    sorterState = READ_HEADER;
                    break;
                
                default: break;
                }
            }
            
        break;
            
       case SWAP_BUFFERS:
       
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
                
            
        default: break;
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
     ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
     ros::shutdown();
    }
    
    iret2 = pthread_create( &sorterThread, NULL, this->sortIncomingData_helper, this);
    if(iret2)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }
    
    iret3 = pthread_create( &swapThread, NULL, this->syncedBufferSwap_helper, this);
    if(iret3)
    {
        ROS_INFO("Error - pthread_create() return code: %d\n",iret1);
        ros::shutdown();
    }

    /* Unlock SIGINT on main thread */
    s = pthread_sigmask(SIG_UNBLOCK, &set, NULL);

    signal(SIGINT, sigHandler);
    
    ros::spin();

    pthread_join(uartThread, NULL);
    ROS_INFO("DataUARTHandler Read Thread joined");

    pthread_join(sorterThread, NULL);
    ROS_INFO("DataUARTHandler Sort Thread joined");

    pthread_join(swapThread, NULL);
    ROS_INFO("DataUARTHandler Swap Thread joined");

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

void DataUARTHandler::visualize(const ti_mmwave_tracker_rospkg::RadarScan &msg){
    visualization_msgs::Marker marker;

    marker.header.frame_id = frameID;
    marker.header.stamp = ros::Time::now();
    marker.id = msg.point_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(tfr);
    marker.action = marker.ADD;

    marker.pose.position.x = msg.x;
    marker.pose.position.y = msg.y;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;

    marker.scale.x = .03;
    marker.scale.y = .03;
    marker.scale.z = .03;
    
    marker.color.a = 1;
    marker.color.r = (int) 255 * msg.intensity;
    marker.color.g = (int) 255 * msg.intensity;
    marker.color.b = 1;

    marker_pub.publish(marker);
}

void DataUARTHandler::stop()
{
    ROS_DEBUG("Stopping Threads");

    stop_threads = true;
    ros::shutdown();
    
    pthread_cond_signal(&read_go_cv);
    pthread_cond_signal(&sort_go_cv);
    pthread_cond_signal(&countSync_max_cv);
}
