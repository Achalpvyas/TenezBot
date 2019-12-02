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
*@file turtlebot.hpp
*@author 	Pruthvikumar Sanghavi, Achal Vyas
*@license BSD 3-Clause
*@brief  Class definition for turtlebot
*/

#pragma once
#include <geometry_msgs/Twist.h>
#include <vector>
#include "ros/ros.h"
#include "tenezbot/pos.h"

/**
*@brief Class turtlebot subscribes to the directions published and publishes velocity commands
*/
class turtlebot {
 public:
    int dir;  /// Direction message to read published directions
/**
*@brief Callback used to subscribe to the direction message published by the Line detection node
*@param msg is the custom message pos which publishes a direction int between 0 and 3
*@return none
*/
    void dir_sub(tenezbot::pos msg);
/**
*@brief Function to publish velocity commands based on direction
*@param velocity is the twist 
*@param pub is used to publish the velocity commands to the turtlebot
*@param rate is the ros loop rate for publishing the commands
*@return none
*/
    void vel_cmd(geometry_msgs::Twist &velocity,
     ros::Publisher &pub, ros::Rate &rate);
};
