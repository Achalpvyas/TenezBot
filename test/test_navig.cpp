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
*@file test_navig.cpp
*@author 	Pruthvikumar Sanghavi, Achal Vyas
*@license BSD 3-Clause
*@brief  Unit Test for all the functions in the turtlebot navigation class
*/
#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "turtlebot.hpp"
#include "tenezbot/pos.h"

/**
*@brief Testing if detection works accurately and publishes straight accurately
*@return int dir which is the direction to move in
*/
double ang_vel(int direction) {
    ros::NodeHandle n;
    turtlebot bot;
    geometry_msgs::Twist velocity;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>
    ("/cmd_vel_mux/input/teleop", 10);
    bot.dir = direction;
    ros::Rate rate(10);
    ros::spinOnce();
    bot.vel_cmd(velocity, pub, rate);
    rate.sleep();
    return velocity.angular.z;
}
/**
*@brief Testing if detection works accurately and publishes straight accurately
*@return int dir which is the direction to move in
*/
double linear_vel(int direction) {
    ros::NodeHandle n;
    turtlebot bot;
    geometry_msgs::Twist velocity;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>
    ("/cmd_vel_mux/input/teleop", 10);

    bot.dir = direction;
    ros::Rate rate(100);
    ros::spinOnce();
    bot.vel_cmd(velocity, pub, rate);
    rate.sleep();
    return velocity.linear.x;
}
/**
*@brief Function to spin the callbacks at a specific rate
*/
void processThread(void) {
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}
/**
*@brief Testing if message subscriber is working properly 
*/
TEST(TestROS, TestPubSub) {
  ros::NodeHandle nh;
  geometry_msgs::Twist velocity;
  ros::Rate rate(10);
  turtlebot bot;
  ros::Publisher pub = nh.advertise<tenezbot::pos>
    ("direction", 1000);
  ros::Subscriber sub = nh.subscribe("/direction",
        1, &turtlebot::dir_sub, &bot);

    tenezbot::pos msg;
    msg.direction = 0;
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
    EXPECT_EQ(1, sub.getNumPublishers());
    EXPECT_EQ(1, pub.getNumSubscribers());
    EXPECT_EQ(msg.direction, bot.dir);
}
/**
*@brief Testing if velocity published is for moving straight 
*/
TEST(TestVelocity, Teststraight_vel) {
    double rot = ang_vel(1);
    double trans = linear_vel(1);
    EXPECT_EQ(0, rot);
    EXPECT_EQ(0.15, trans);
}
/**
*@brief Testing if velocity published is for turning left 
*/
TEST(TestDirections, Testleft_vel) {
    double rot = ang_vel(0);
    double trans = linear_vel(0);
    EXPECT_EQ(0.15, rot);
    EXPECT_EQ(0.1, trans);
}
/**
*@brief Testing if velocity published is for turning right 
*/
TEST(TestDirections, Testright_vel) {
    double rot = ang_vel(2);
    double trans = linear_vel(2);
    EXPECT_EQ(-0.15, rot);
    EXPECT_EQ(0.1, trans);
}
/**
*@brief Testing if velocity published is for searching 
*/
TEST(TestDirections, Testsearch_vel) {
    double rot = ang_vel(3);
    double trans = linear_vel(3);
    EXPECT_EQ(0.25, rot);
    EXPECT_EQ(0, trans);
}
/**
 *@brief Function to run all the tests for the navigation node
 *@param argc is the number of arguments of the main function
 *@param argv is the array of arugments
 *@return result of the tests
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_velocity");
    testing::InitGoogleTest(&argc, argv);
    ros::NodeHandle nh;
    boost::thread th(processThread);
    int test_flag = RUN_ALL_TESTS();
    ros::shutdown();
    th.join();
    return test_flag;
}
