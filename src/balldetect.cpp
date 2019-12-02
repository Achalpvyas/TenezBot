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
*@file 		balldetect.cpp
*@author 	Pruthvikumar Sanghavi, Achal Vyas
*@license BSD 3-Clause
*@brief  	Class balldetect's function definitions
*/
#include "balldetect.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "ros/console.h"
#include "tenezbot/pos.h"

void BallDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img = cv_ptr->image;
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

cv::Mat BallDetect::Gauss(cv::Mat input) {
  cv::Mat output;
// Applying Gaussian Filter
  cv::GaussianBlur(input, output, cv::Size(3, 3), 0.1, 0.1);
  return output;
}

int BallDetect::colorthresh(cv::Mat input) {
  // Initializaing variables
  cv::Size s = input.size();
  std::vector<std::vector<cv::Point> > v;
  auto w = s.width;
  auto h = s.height;
  auto c_x = 0.0;
  // Detect all objects within the HSV range
  cv::cvtColor(input, BallDetect::img_hsv, CV_BGR2HSV);
  BallDetect::LowerRed = {20, 100, 100};
  BallDetect::UpperRed = {30, 255, 255};
  cv::inRange(BallDetect::img_hsv, LowerRed,
   UpperRed, BallDetect::img_mask);
  img_mask(cv::Rect(0, 0, w, 0.8*h)) = 0;
  // Find contours for better visualization
  cv::findContours(BallDetect::img_mask, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // If contours exist add a bounding
  // Choosing contours with maximum area
  if (v.size() != 0) {
  auto area = 0;
  auto idx = 0;
  auto count = 0;
  while (count < v.size()) {
    if (area < v[count].size()) {
       idx = count;
       area = v[count].size();
    }
    count++;
  }
  cv::Rect rect = boundingRect(v[idx]);
  cv::Point pt1, pt2, pt3;
  pt1.x = rect.x;
  pt1.y = rect.y;
  pt2.x = rect.x + rect.width;
  pt2.y = rect.y + rect.height;
  pt3.x = pt1.x+5;
  pt3.y = pt1.y-5;
  // Drawing the rectangle using points obtained
  rectangle(input, pt1, pt2, CV_RGB(255, 0, 0), 2);
  // Inserting text box
  cv::putText(input, "Ball Detected", pt3,
    CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
  }
  // Mask image to limit the future turns affecting the output
  img_mask(cv::Rect(0.7*w, 0, 0.3*w, h)) = 0;
  img_mask(cv::Rect(0, 0, 0.3*w, h)) = 0;
  // Perform centroid detection of ball
  cv::Moments M = cv::moments(BallDetect::img_mask);
  if (M.m00 > 0) {
    cv::Point p1(M.m10/M.m00, M.m01/M.m00);
    cv::circle(BallDetect::img_mask, p1, 5, cv::Scalar(155, 200, 0), -1);
  }
  c_x = M.m10/M.m00;
  // Tolerance to chooise directions
  auto tol = 15;
  auto count = cv::countNonZero(img_mask);
  // Turn left if centroid is to the left of the image center minus tolerance
  // Turn right if centroid is to the right of the image center plus tolerance
  // Go straight if centroid is near image center
  if (c_x < w/2-tol) {
    BallDetect::dir = 0;
  } else if (c_x > w/2+tol) {
    BallDetect::dir = 2;
  } else {
    BallDetect::dir = 1;
  }
  // Search if no Ball detected
  if (count == 0) {
    BallDetect::dir = 3;
  }
  // Output images viewed by the turtlebot
  cv::namedWindow("Turtlebot View");
  imshow("Turtlebot View", input);
  return BallDetect::dir;
}
