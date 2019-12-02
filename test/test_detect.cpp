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
*@file test_detect.cpp
*@author 	Pruthvikumar Sanghavi, Achal Vyas
*@license BSD 3-Clause
*@brief  Unit Test for all the functions in the detection class
*/

#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "balldetect.hpp"
#include "tenezbot/pos.h"

/**
*@brief Testing if detection works accurately and publishes left accurately
*@return int dir which is the direction to move in
*/
int turn_left() {
    ros::NodeHandle n;
    BallDetect det;
    cv::String im_path1 = "./test_images/left_turn.png";
    det.img = cv::imread(im_path1);

    if (!det.img.empty()) {
        det.img_filt = det.Gauss(det.img);
        det.dir = det.colorthresh(det.img_filt);
        return det.dir;
    }
}
/**
*@brief Testing if detection works accurately and publishes straight accurately
*@return int dir which is the direction to move in
*/
int drive_straight() {
    ros::NodeHandle n;
    BallDetect det;
    cv::String im_path = "./test_images/straight.png";
    det.img = cv::imread(im_path);
    if (!det.img.empty()) {
        det.img_filt = det.Gauss(det.img);
        det.dir = det.colorthresh(det.img_filt);
        return det.dir;
    }
}
/**
*@brief Testing if detection works accurately and publishes right accurately
*@return int dir which is the direction to move in
*/
int turn_right() {
    ros::NodeHandle n;
    BallDetect det;
    cv::String im_path2 = "./test_images/right_turn.png";
    det.img = cv::imread(im_path2);

    if (!det.img.empty()) {
        det.img_filt = det.Gauss(det.img);
        det.dir = det.colorthresh(det.img_filt);
        return det.dir;
    }
}

/**
*@brief Testing if detection works accurately and publishes stop accurately
*@return int dir which is the direction to move in
*/
int stop() {
    ros::NodeHandle n;
    BallDetect det;
    cv::String im_path3 = "./test_images/stop.png";
    det.img = cv::imread(im_path3);

    if (!det.img.empty()) {
        det.img_filt = det.Gauss(det.img);
        det.dir = det.colorthresh(det.img_filt);
        return det.dir;
    }
}
/**
*@brief Testing if gaussian filtering functions properly
*@return bool to test if function worked well
*/
bool gauss() {
    ros::NodeHandle n;
    BallDetect det;
    cv::String im_path3 = "./test_images/stop.png";
    det.img = cv::imread(im_path3);
    cv::Size im_size = det.img.size();
    det.img_filt = det.Gauss(det.img);
    cv::Size imfilt_size = det.img_filt.size();
    if (im_size == imfilt_size)
        return true;
}

/**
*@brief Testing if direction published is straight 
*/

TEST(TestDirections, Teststraight) {
    int direction = drive_straight();
    EXPECT_EQ(1, direction);
}

/**
*@brief Testing if direction published is left
*/

TEST(TestDirections, Testleft) {
    int direction = turn_left();
    EXPECT_EQ(0, direction);
}

/**
*@brief Testing if direction published is right 
*/

TEST(TestDirections, Testright ) {
    int direction = turn_right();
    EXPECT_EQ(2, direction);
}

/**
*@brief Testing if search command is published 
*/

TEST(TestDirections, Testsearch ) {
    int direction = stop();
    EXPECT_EQ(3, direction);
}

/**
*@brief Testing if Gaussian filter is applied to image properly 
*/

TEST(TestDetFunc , TestGauss) {
    EXPECT_EQ(true , gauss());
}
/**
 *@brief Function to run all the tests for the detection node
 *@param argc is the number of arguments of the main function
 *@param argv is the array of arugments
 *@return result of the tests
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "test_detection");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
