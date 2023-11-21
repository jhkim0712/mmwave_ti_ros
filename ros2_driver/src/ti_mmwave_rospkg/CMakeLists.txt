cmake_minimum_required(VERSION 3.8)
project(ti_mmwave_rospkg)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")
# add_definitions(-std=c++11)


# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter -fPIC)
# endif()

## dependencies
# find_package(ament_lint_auto REQUIRED)
# ament_lint_auto_find_test_dependencies()

# find_package(ament_cmake_auto REQUIRED)
# ament_auto_find_build_dependencies()

# find_package(asio_cmake_module REQUIRED)

# find_package(ASIO REQUIRED)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(rcutils REQUIRED)

find_package(rclcpp_components REQUIRED)

find_package(rcl_interfaces REQUIRED)

find_package(std_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(visualization_msgs REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)

find_package(ti_mmwave_rospkg_msgs REQUIRED)

find_package(serial REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(pluginlib REQUIRED)

find_package(Boost REQUIRED)

find_package(Threads REQUIRED)

find_package(PCL 1.7.5 REQUIRED)

find_package(pcl_conversions REQUIRED)

include_directories( 
  include
  ${ti_mmwave_rospkg_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pthread_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${rt_INCLUDE_DIRS}
  ${rcutils_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

add_library(mmwave SHARED
  #src/mmWaveCommSrv.cpp
  #src/DataHandlerClass.cpp
  #src/mmWaveQuickConfig.cpp
  src/ParameterParser.cpp
  #src/ConfigParameterServer.cpp
)

#add_executable(mmWaveLoader src/mmWaveLoader.cpp)

add_executable(mmWaveCommSrv src/mmWaveCommSrv.cpp)

add_executable(mmWaveQuickConfig src/mmWaveQuickConfig.cpp)

add_executable(ParameterParser src/ParameterParser.cpp)

add_executable(DataHandlerClass src/DataHandlerClass.cpp)

add_executable(ConfigParameterServer src/ConfigParameterServer.cpp)


target_link_libraries(mmWaveCommSrv 
  mmwave 
)

target_link_libraries(mmWaveQuickConfig
  mmwave
)

target_link_libraries(ParameterParser
  mmwave
)

target_link_libraries(DataHandlerClass
  mmwave
  ${Boost_LIBRARIES}
)

ament_target_dependencies(mmwave
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "rcl_interfaces"  
  "serial"
  "sensor_msgs"
  "std_msgs" 
  "visualization_msgs"
  "pluginlib"
  "ti_mmwave_rospkg_msgs"
  "tf2_geometry_msgs"
)

ament_target_dependencies(mmWaveCommSrv
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "rcl_interfaces"  
  "serial"
  "sensor_msgs"
  "std_msgs" 
  "visualization_msgs"
  "pluginlib"
  "ti_mmwave_rospkg_msgs"
  "tf2_geometry_msgs"
)


ament_target_dependencies(mmWaveQuickConfig
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "rcl_interfaces"  
  "serial"
  "sensor_msgs"
  "std_msgs" 
  "visualization_msgs"
  "pluginlib"
  "ti_mmwave_rospkg_msgs"
  "tf2_geometry_msgs"
)

ament_target_dependencies(ParameterParser   
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "rcl_interfaces"  
  "serial"
  "sensor_msgs"
  "std_msgs" 
  "visualization_msgs"
  "pluginlib"
  "ti_mmwave_rospkg_msgs"
  "tf2_geometry_msgs"
)

ament_target_dependencies(DataHandlerClass   
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "rcl_interfaces"  
  "serial"
  "sensor_msgs"
  "std_msgs" 
  "visualization_msgs"
  "pluginlib"
  "ti_mmwave_rospkg_msgs"
  "tf2_geometry_msgs"
  "geometry_msgs"
)

ament_target_dependencies(ConfigParameterServer   
  "rclcpp"
  "rclcpp_components"
  "rcutils"
  "rcl_interfaces"  
  "serial"
  "sensor_msgs"
  "std_msgs" 
  "visualization_msgs"
  "pluginlib"
  "ti_mmwave_rospkg_msgs"
  "tf2_geometry_msgs"
)



ament_export_dependencies(${dependencies} ti_mmwave_rospkg_msgs)
ament_export_include_directories(include)
ament_export_libraries(mmwave ti_mmwave_rospkg_msgs)


install(TARGETS
  mmwave
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS
  ConfigParameterServer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  cfg
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS 
  mmWaveCommSrv 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  mmWaveQuickConfig 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  ParameterParser 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  DataHandlerClass 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
