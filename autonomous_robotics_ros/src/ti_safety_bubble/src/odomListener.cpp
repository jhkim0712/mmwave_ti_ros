/*
* @file odomListener.cpp
*
* @brief
* Subscribes to the global obstacle layers's costmap and footprint nodes, 
* publishes velocities (full/half/stop) according to bubble size and shape.
* Clears the costmap every T secs by calling navigation's clear_costmaps service. 
*
* \par
* NOTE:
* (C) Copyright 2021 Texas Instruments, Inc.
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
#include "std_msgs/Float32.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Twist.h"
#include "tf2_ros/transform_listener.h"

enum SafetyBubbleShape {
  RECTANGULAR=0,
  CIRCULAR
};

struct Point2D {
  double x;
  double y;
};

double robotX, robotY;
// radius for inner and outer bubbles
double innerLimit, outerLimit;
// dimensions for rectangular bubbles 
double innerWidth, innerLength, outerWidth, outerLength; 
nav_msgs::OccupancyGrid gMsg;
double yaw;

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
 * This callback saves the orientation of the robot. 
 */
void orientationCallback(const std_msgs::Float32::ConstPtr& msg)
{
    yaw = (double) msg->data;
}

/**
 * Calculates the distance between p1 and p2.
 */ 
double calculateDistance(double p1x, double p1y, double p2x, double p2y)
{
  return sqrt(pow(p1x - p2x, 2) + pow(p1y - p2y, 2));
}

Point2D rotatePoint(Point2D point) {
  double sinus = sin(yaw);
  double cosinus = cos(yaw);
  Point2D temp;

  point.x = point.x - robotX;
  point.y = point.y - robotY;
  temp.x = point.x * cosinus - point.y * sinus;
  temp.y = point.x * sinus + point.y * cosinus;
  point.x = temp.x + robotX;
  point.y = temp.y + robotY;
  return point;
}

void publishCircularSafetyBubble(ros::Publisher *slow_pub, ros::Publisher *stop_pub)
{
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
  slow_pub->publish(slowzone);
  stop_pub->publish(stopzone);
}

void publishRectangularSafetyBubble(ros::Publisher *slow_pub, ros::Publisher *stop_pub)
{
  geometry_msgs::PolygonStamped slowzone;
  geometry_msgs::PolygonStamped stopzone;
  slowzone.header.frame_id = "slowzone";
  stopzone.header.frame_id = "stopzone";
  // TODO: Calculate rectangular size 
  geometry_msgs::Point32 slow_point, stop_point;
  slow_point.z = 0.0;
  stop_point.z = 0.0;
  // top right 
  slow_point.x = outerLength / 2;
  stop_point.x = innerLength / 2;
  slow_point.y = outerWidth / 2;
  stop_point.y = innerWidth / 2;  
  slowzone.polygon.points.push_back(slow_point);
  stopzone.polygon.points.push_back(stop_point);
  // top left 
  slow_point.x = -outerLength / 2;
  stop_point.x = -innerLength / 2;
  slow_point.y = outerWidth / 2;
  stop_point.y = innerWidth / 2;  
  slowzone.polygon.points.push_back(slow_point);
  stopzone.polygon.points.push_back(stop_point);
    // bottom left 
  slow_point.x = -outerLength / 2;
  stop_point.x = -innerLength / 2;
  slow_point.y = -outerWidth / 2;
  stop_point.y = -innerWidth / 2;  
  slowzone.polygon.points.push_back(slow_point);
  stopzone.polygon.points.push_back(stop_point);
  // bottom right 
  slow_point.x = outerLength / 2;
  stop_point.x = innerLength / 2;
  slow_point.y = -outerWidth / 2;
  stop_point.y = -innerWidth / 2;  
  slowzone.polygon.points.push_back(slow_point);
  stopzone.polygon.points.push_back(stop_point);
  slow_pub->publish(slowzone);
  stop_pub->publish(stopzone);
}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "odomListener");

  ros::NodeHandle n;
  ros::NodeHandle n2("~");

  ros::Subscriber footprintSub = n.subscribe("/move_base/global_costmap/footprint", 10, footprintCallback);
  ros::Subscriber mapSub = n.subscribe("/move_base/global_costmap/costmap", 10, mapCallback);
  ros::Subscriber orientSub = n.subscribe("/ti_base/yaw", 10, orientationCallback);

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  std_srvs::Empty srv;

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
  ros::Publisher slowzone_pub = n.advertise<geometry_msgs::PolygonStamped>("/ti_base/slowzone", 1);
  ros::Publisher stopzone_pub = n.advertise<geometry_msgs::PolygonStamped>("/ti_base/stopzone", 1);

  outerLimit = 1.5; //radius in meters
  innerLimit = 1.0; //radius in meters
  double mapsClearTime = 7.5; //seconds

  // get bubble shape
  std::string shapeString;
  SafetyBubbleShape bubbleShape;
  n2.getParam("bubble_shape", shapeString);
  ROS_INFO("%s", shapeString.c_str());

  // circular bubble
  if (shapeString.compare("circular") == 0) {
    bubbleShape = CIRCULAR;
    n2.getParam("slow_radius", outerLimit);
    n2.getParam("stop_radius", innerLimit);
  } else if (shapeString.compare("rectangular") == 0) {
    // rectangular bubble
    bubbleShape = RECTANGULAR;
    n2.getParam("inner_width", innerWidth);
    n2.getParam("inner_length", innerLength);
    n2.getParam("outer_width", outerWidth);
    n2.getParam("outer_length", outerLength);
  } else {
    ROS_ERROR("Bubble shape parameter not set, stopping safety bubble");
    ros::shutdown();
  }
  // set time for periodically clearing costmap
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
      
      if (bubbleShape == CIRCULAR)
      {
        publishCircularSafetyBubble(&slowzone_pub, &stopzone_pub);
      } else if (bubbleShape == RECTANGULAR)
      {
        publishRectangularSafetyBubble(&slowzone_pub, &stopzone_pub);
      } 

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
            if (bubbleShape == CIRCULAR) 
            {
              double dist = calculateDistance(tempRobotX, tempRobotY, py, px);
              if (outerLimit > dist && dist > innerLimit)
              {
                slowFlag = true;
              } else if (innerLimit >= dist)
              {
                stopFlag = true;
              }
            } else if (bubbleShape == RECTANGULAR)
            {
              Point2D point;
              point.x = py;
              point.y = px;
              Point2D rotated = rotatePoint(point);
              double diffLength = abs(tempRobotX - rotated.x);
              double diffWidth = abs(tempRobotY - rotated.y);
              if (innerLength/2 >= diffLength && innerWidth/2 >= diffWidth) {
                stopFlag = true;
              } else if (outerLength/2 >= diffLength && outerWidth/2 >= diffWidth) {
                slowFlag = true;
              }
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
