/**BSD 3-Clause License
  *
  *Copyright (c) 2019, Pruthvikumar Sanghavi
  *Copyright (c) 2019, Achal Vyas
  *All rights reserved.
  *
  *Redistribution and use in source and binary forms, with or without
  *modification, are permitted provided that the following conditions are met:
  *
  *1. Redistributions of source code must retain the above copyright notice, this
  *   list of conditions and the following disclaimer.
  *
  *2. Redistributions in binary form must reproduce the above copyright notice,
  *   this list of conditions and the following disclaimer in the documentation
  *   and/or other materials provided with the distribution.
  *
  *3. Neither the name of the copyright holder nor the names of its
  *   contributors may be used to endorse or promote products derived from
  *   this software without specific prior written permission.
  *
  *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/
/*
*@copyright Copyright 2019 Pruthvikumar Sanghavi
*                          Achal Vyas
*@file turtlebot.cpp
*@author 	Pruthvikumar Sanghavi, Achal Vyas
*@license BSD 3-Clause
*@brief  Functions definitions for turtlebot class
*/

#include <geometry_msgs/Twist.h>
#include <vector>
#include "ros/ros.h"
#include "ros/console.h"
#include "turtlebot.hpp"
#include "tenezbot/pos.h"

void turtlebot::dir_sub(tenezbot::pos msg) {
    turtlebot::dir = msg.direction;
}
void turtlebot::vel_cmd(geometry_msgs::Twist &velocity,
 ros::Publisher &pub, ros::Rate &rate) {
    // If direction is left
    if (turtlebot::dir == 0) {
        velocity.linear.x = 0.1;
        velocity.angular.z = 0.15;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Turning Left");
    }
    // If direction is straight
    if (turtlebot::dir == 1) {
        velocity.linear.x = 0.15;
        velocity.angular.z = 0;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Straight");
    }
    // If direction is right
    if (turtlebot::dir == 2) {
        velocity.linear.x = 0.1;
        velocity.angular.z = -0.15;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Turning Right");
    }
    // If robot has to search
    if (turtlebot::dir == 3) {
        velocity.linear.x = 0;
        velocity.angular.z = 0.25;
        pub.publish(velocity);
        rate.sleep();
        ROS_INFO_STREAM("Searching");
    }
}
