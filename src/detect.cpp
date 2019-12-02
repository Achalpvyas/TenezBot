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
 *@file   detect.cpp
*@author 	Pruthvikumar Sanghavi, Achal Vyas
*@license BSD 3-Clause
 *@brief  Ros Nod to subscribe to turtlebot images and perform image processing to detect ball
 */
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "balldetect.hpp"
#include "tenezbot/pos.h"

/**
 *@brief Main function that reads image from the turtlebot and provides direction to move using image processing
 *@param argc is the number of arguments of the main function
 *@param argv is the array of arugments
 *@return 0
*/

int main(int argc, char **argv) {
    // Initializing node and object
    ros::init(argc, argv, "detection");
    ros::NodeHandle n;
    BallDetect det;
    // Creating Publisher and subscriber
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw",
        1, &BallDetect::imageCallback, &det);

    ros::Publisher dirPub = n.advertise<
    tenezbot::pos>("direction", 1);
        tenezbot::pos msg;

    while (ros::ok()) {
        if (!det.img.empty()) {
            // Perform image processing
            det.img_filt = det.Gauss(det.img);
            msg.direction = det.colorthresh(det.img_filt);
            // Publish direction message
            dirPub.publish(msg);
            }
        ros::spinOnce();
    }
    // Closing image viewer
    cv::destroyWindow("Turtlebot View");
}
