/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Bram Fenijn
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <boost/asio.hpp>
#include <slambot_core/slambot.h>
#include <slambot_core/serialMsg.h>

#include "geometry_msgs/Twist.h"
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
#include "slambot_core/Pid.h"
//#include <sensor_msgs/LaserScan.h>
 

 Slambot* g_slambot;

const float countTomps = 149.714;
const float inerWheelDis = 0.145;

 float lidar_kp = 1.0;
 float lidar_ki = 0.5;
 int16_t integral = 0;

double g_x = 0.0;
double g_y = 0.0;
double g_th = 0.0;

 void sendOdom(double right_speed, double left_speed);

void cmdCallback(const geometry_msgs::Twist::ConstPtr &velMsg)
{
 // ROS_INFO("cmdCallback");
  geometry_msgs::Twist twist = (geometry_msgs::Twist)*velMsg;
  float linear = twist.linear.x;
  float angular = twist.angular.z;

  int16_t speedLeft = (linear + angular)*countTomps;
  int16_t speedRight = (linear - angular)*countTomps;
  ROS_INFO("Send speed: %i, %i",speedLeft,speedRight);
  g_slambot->sendVelocity(speedLeft,speedRight);
}


void rpmCallback(const std_msgs::UInt16::ConstPtr &rpmsMsg)
{
 // ROS_INFO("cmdCallback");
  std_msgs::UInt16 rpms = (std_msgs::UInt16)*rpmsMsg;
  uint16_t rpm = rpms.data;

  
  if(rpm > 800)
  {
    return;
  }
  
  int16_t rpmError = 300 - rpm;
  integral += rpmError;
  int16_t lidarSpeed = 200 + (rpmError * lidar_kp) + (integral * lidar_ki);

  if(lidarSpeed < 0)
  {
    lidarSpeed = 0;
  }
  ROS_INFO("Send lidar: %i, %i",rpm,lidarSpeed);
  g_slambot->sendLidarSpeed(lidarSpeed);
}
void pidCallback(const slambot_core::Pid::ConstPtr &pidMsg)
{
 // ROS_INFO("cmdCallback");
  slambot_core::Pid pids = (slambot_core::Pid)*pidMsg;
  ROS_INFO("change pids");
  
   g_slambot->sendPids(pids.kp,pids.ki,pids.kd);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "slambot_core");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  ROS_INFO("starting slambot_core");
  
   ros::Time g_current_time, g_last_time;
 ros::Publisher g_odom_pub;
 tf::TransformBroadcaster g_odom_broadcaster;

  std::string port;
  int baud_rate;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("baud_rate", baud_rate, 115200);

  priv_nh.param("lidar/kp", lidar_kp, 1.0f);
  priv_nh.param("lidar/ki", lidar_ki, 0.5f);

  boost::asio::io_service io;

  

  try {
    
    g_slambot = new Slambot(port, baud_rate, io);

    ros::Subscriber cmdSubscriber = n.subscribe("/cmd_vel", 1000, cmdCallback);
    ros::Subscriber rpmSubscriber = n.subscribe("/rpms", 1000, rpmCallback);
    ros::Subscriber pidSubscriber = n.subscribe("/wheel_pid", 1000, pidCallback);
    g_odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    


    
    g_current_time = ros::Time::now();
    g_last_time = ros::Time::now();

    ROS_INFO("ready");
    ros::Rate r(100); 
    while (ros::ok()) 
    {
      ros::spinOnce();
      g_current_time = ros::Time::now();

      SerialMsg sMsg;
      int8_t status = g_slambot->readMsg(&sMsg);
      if(status == 1)
      {
        switch(sMsg.type)
        {
          case 1:
          {
            SpeedMsg speedMsg;
            memcpy(&speedMsg,sMsg.data,4);

            float right_speed = speedMsg.right / countTomps;
            float left_speed = speedMsg.left / countTomps;

            double vx = (right_speed + left_speed) / 2;
            double vy = 0.0;
            double vth = ((left_speed - right_speed) / 0.6988);

            double dt = (g_current_time - g_last_time).toSec();
            double delta_x = (vx * cos(g_th) - vy * sin(g_th)) * dt;
            double delta_y = (vx * sin(g_th) + vy * cos(g_th)) * dt;
            double delta_th = vth * dt;

            g_x += (delta_x*0.3);
            g_y += (delta_y*0.3);
            g_th += delta_th;
	          ROS_INFO("x: %f y: %f th: %f",g_x,g_y,g_th);
            //since all odometry is 6DOF we'll need a quaternion created from yaw
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(g_th);

            //first, we'll publish the transform over tf
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = g_current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = g_x;
            odom_trans.transform.translation.y = g_y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            //send the transform
            g_odom_broadcaster.sendTransform(odom_trans);

            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = g_current_time;
            odom.header.frame_id = "odom";

            //set the position
            odom.pose.pose.position.x = g_x;
            odom.pose.pose.position.y = g_y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
            odom.twist.twist.angular.z = vth;

            //publish the message
            g_odom_pub.publish(odom);
            break;
          }
          default:
          {
            break;
          }
        }
      }
      else
      {
         ROS_INFO("receive failed, error: %i",status);
      }

      g_last_time = g_current_time;
      r.sleep();
    }
    ROS_INFO("stop");
    g_slambot->close();

    delete g_slambot;
    g_slambot = NULL;

    return 0;
  } catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating slambot object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}

void sendOdom(double right_speed, double left_speed)
{ 

   
}
