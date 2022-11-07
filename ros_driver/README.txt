--------------------------------------------------------------------------------------------------------------------------------------------------


v1.0 

ROS Driver + Tracking added
Detection layer for ti_mmwave_tracker_rospkg will now be based off of Capon algorithm and processing chain with tracking, as opposed to the Bartlett algorithm which is also used in Out of Box Demo and no tracking. Capon algorithm allows for better point cloud density and accuracy, as well as tracking information.

Tested with Industrial Toolbox for mmWave Sensors 4.12.0


--------------------------------------------------------------------------------------------------------------------------------------------------


Setup:

1. Follow the ROS Driver user guide up until the step that requires you to use "roslaunch" 

https://dev.ti.com/tirex/explore/node?a=VLyFKFf__4.12.0&node=AAFPzcnzNHunUphPQZyTyg__VLyFKFf__4.12.0

**NOTE** For anywhere the guide mentions Out of Box demo make sure you are using the People Counting binary file.

2. Use the console command "roslaunch ti_mmwave_tracker_rospkg AOP_3d_Tracking.launch" to start the sensor which will start the sensor and Rviz to show the point cloud. The tracking information is not visualized but number of tracks detected is currently being printed to console via a "ROS_INFO" command within DataHandlerClass.cpp.

If you want to avoid having Rviz start at the same time as the sensor, delete the entire line near the bottom of AOP_3d_Tracking.launch that says "rviz" in it


--------------------------------------------------------------------------------------------------------------------------------------------------


Developer's Guide:

All tracker information is handled within the "DataHandlerClass.cpp" file in the src folder.


Each track has 112 bytes of information. 

Track List 3D	112 Bytes * Number of Tracks	
'tid', {'uint32', 4}, ... % Track ID
'posX', {'float', 4}, ... % Track position in X dimension, m
'posY', {'float', 4}, ... % Track position in Y dimension, m
'posZ', {'float', 4}, ... % Track position in Z dimension, m
'velX', {'float', 4}, ... % Track velocity in X dimension, m/s
'velY', {'float', 4}, ... % Track velocity in Y dimension, m/s
'velZ', {'float', 4}, ... % Track velocity in Z dimension, m/s
'accX', {'float', 4}, ... % Track acceleration in X dimension, m/s2
'accY', {'float', 4}, ... % Track acceleration in Y dimension, m/s
'accZ', {'float', 4}, ... % Track acceleration in Z dimension, m/s


--------------------------------------------------------------------------------------------------------------------------------------------------


Every point is given a track index value, assigning that point to a track ID it falls into. If a point does not fall into a track for whatever reason, a special value is given for why. 

Track Index 	1 Byte * Number of Points	
'targetID', {'uint8', 1}); % Track ID

Special Values:

253 - Point not associated, SNR too weak

254 - Point not associated, located outside boundary of interest

255 - Point not associated, considered as noise


--------------------------------------------------------------------------------------------------------------------------------------------------

3D Spherical Compressed Point Cloud: 20 bytes + (7 bytes * Number of Points)

For data compression, the 20 bytes worth of unit values are sent first and only once. These unit values need to be multiplied by every point's 7 bytes of values to get the point's real values.

 ‘elevationUnit', {'float', 4}, ... % unit resolution of elevation report, in rad
 'azimuthUnit', {'float', 4}, ... % unit resolution of azimuth report, in rad
 'dopplerUint', {'float', 4}, ... % unit resolution of Doppler report, in m/s
 'rangeUint', {'float', 4}, ... % unit resolution of Range report, in m
 'snrUint', {'float', 4}); % unit resolution of SNR report, ratio

Each point has the following:

pointStruct = struct(...
 ‘elevation', {'int8', 1}, ... % Elevation report, in number of elevationUnit
 'azimuth', {'int8', 1}, ... % Azimuth report, in number of azimuthUnit
 'doppler', {'int16', 1}, ... % Doppler, in number of dopplerUint
 'range', {‘uint16', 2}, ... % Range, in number of rangeUint
 'snr', {‘uint16', 2}); % SNR, in number of snrUint

 --------------------------------------------------------------------------------------------------------------------------------------------------
