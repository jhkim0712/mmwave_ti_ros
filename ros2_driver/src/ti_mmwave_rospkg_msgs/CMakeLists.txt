cmake_minimum_required(VERSION 3.5)
project(ti_mmwave_rospkg_msgs)

# Default to C++14
# if(NOT WIN32)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS"} -std=c++14 -Wall -Wextra)
# endif()

## dependencies

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)


## Generate messages in the 'msg' folder
set(msg_files
   "msg/RadarScan.msg"
   "msg/RadarOccupancy.msg"
   "msg/RadarTrackArray.msg"
   "msg/RadarTrackContents.msg"
   "msg/RadarPointTrackID.msg"
)

set(srv_files
   "srv/MmwaveCli.srv"
)

rosidl_generate_interfaces(ti_mmwave_rospkg_msgs
  ${msg_files}
  ${srv_files}
)

## ament_target_dependencies(std_msgs rclcpp)  

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  rclcpp
)

ament_package()