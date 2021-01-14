/*
* @file odomListener.cpp
*
* @brief
* Subscribes to the global obstacle layers's costmap and footprint nodes, 
* publishes velocities based on the .
*
* \par
* NOTE:
* (C) Copyright 2020 Texas Instruments, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the
* distribution.
*
* Neither the name of Texas Instruments Incorporated nor the names of
* its contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Twist.h"

double robotX, robotY;
nav_msgs::OccupancyGrid gMsg;

/**
 * This callback receives the robot's polygon and calculates the center. 
 * The center point will be used to calculate distances to objects.
 */
void footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    //ROS_INFO("[%3.3f], [%3.3f]", msg->pose.pose.position.x,msg->pose.pose.position.y);
    //geometry_msgs::Point32 p  
    double x, y;
    int s = (int) msg->polygon.points.size();
    if (s == 0)
        return;
    x = 0.0;
    y = 0.0;
    for (int i = 0; i < s; i++)
    {
        x += msg->polygon.points[i].x;
        y += msg->polygon.points[i].y;
    }
    robotX = x / (double) s;
    robotY = y / (double) s;
}

/**
 * This callback saves the global costmap to the gMsg variable.
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    gMsg = *msg;
}

/**
 * Calculates the distance between p1 and p2.
 */ 
double calculateDistance(double p1x, double p1y, double p2x, double p2y)
{
  return sqrt(pow(p1x - p2x, 2) + pow(p1y - p2y, 2));
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "odomListener");

  ros::NodeHandle n;
  ros::NodeHandle n2("~");

  ros::Subscriber footprintSub = n.subscribe("/move_base/global_costmap/footprint", 10, footprintCallback);
  ros::Subscriber mapSub = n.subscribe("/move_base/global_costmap/costmap", 10, mapCallback);

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  std_srvs::Empty srv;

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
  ros::Publisher slowzone_pub = n.advertise<geometry_msgs::PolygonStamped>("/ti_base/slowzone", 1);
  ros::Publisher stopzone_pub = n.advertise<geometry_msgs::PolygonStamped>("/ti_base/stopzone", 1);

  double outerLimit = 1.5; //radius in meters
  double innerLimit = 1.0; //radius in meters
  double mapsClearTime = 7.5; //seconds

  n2.getParam("slow_radius", outerLimit);
  n2.getParam("stop_radius", innerLimit);
  n2.getParam("clear_costmap_period_secs", mapsClearTime);
    

  double rate = 20;
  ros::Rate loop_rate(rate);
  int counter = 0;
  bool stopFlag = false;
  bool slowFlag = false;
  while (ros::ok())
  {
    if (gMsg.info.width != 0) {
      double tempRobotX = robotX;
      double tempRobotY = robotY;
      stopFlag = false;
      slowFlag = false;

      // create polygons for zones
      geometry_msgs::PolygonStamped slowzone;
      geometry_msgs::PolygonStamped stopzone;
      slowzone.header.frame_id = "slowzone";
      stopzone.header.frame_id = "stopzone";
      int numPoints = 12; // number of points for polygon
      for (int i = 0; i < numPoints; ++i) {
        double angle = i * ( 360.0 / (double) numPoints) * M_PI / 180.0;
        geometry_msgs::Point32 slow_point, stop_point;
        slow_point.z = 0.0;
        stop_point.z = 0.0;
        slow_point.x = cos(angle) * outerLimit;
        slow_point.y = sin(angle) * outerLimit;
        stop_point.x = cos(angle) * innerLimit;
        stop_point.y = sin(angle) * innerLimit;
        slowzone.polygon.points.push_back(slow_point);
        stopzone.polygon.points.push_back(stop_point);
      }
      slowzone_pub.publish(slowzone);
      stopzone_pub.publish(stopzone); 

      // calculate distance to objects
      for (int i = 0; i < gMsg.info.height; i++)
      {
        for (int j = 0; j < gMsg.info.width; j++)
        {
          int ind = (i * gMsg.info.width) + j;
          // grid spaces with probability of 80 or higher will have distance calculated
          if (gMsg.data[ind] >= 80)
          {
            double px = (i * gMsg.info.resolution) + gMsg.info.origin.position.x;
            double py = (j * gMsg.info.resolution) + gMsg.info.origin.position.y;
            double dist = calculateDistance(tempRobotX, tempRobotY, py, px);
            if (outerLimit > dist && dist > innerLimit)
            {
              slowFlag = true;
            } else if (innerLimit >= dist)
            {
              stopFlag = true;
            }
          }
        }
      }
      // publish velocity command
      geometry_msgs::Twist vel;
      // hard-coded angular velocity to offset turtlebot2 from straying to the left
      vel.angular.z = -0.0075;
      if (stopFlag) {
        ROS_INFO("STOP");
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
      } else if (slowFlag) {
        ROS_INFO("SLOW");
        vel.linear.x = 0.1;
      } else {
        ROS_INFO("FULL");
        vel.linear.x = 0.2;
      }
      vel_pub.publish(vel);
    }

    // calls move_base service to clear costmaps
    counter += 1;
    if (counter > (int) (rate * mapsClearTime)) {
      if (client.call(srv))
      {
        ROS_INFO("maps cleared");
      } else {
        ROS_INFO("can't call service");
      }
      counter = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
