# TenezBot::A Tennis Ball Collecting Robot

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Build Status](https://travis-ci.org/Pruthvi-Sanghavi/TenezBot.svg?branch=master)](https://travis-ci.org/Pruthvi-Sanghavi/TenezBot)
[![Coverage Status](https://coveralls.io/repos/github/Pruthvi-Sanghavi/TenezBot/badge.svg?branch=master)](https://coveralls.io/github/Pruthvi-Sanghavi/TenezBot?branch=master)

## Developers

- Achal Vyas [Github](https://github.com/Achalpvyas)
- Pruthvi Sanghavi [Github](https://github.com/Pruthvi-Sanghavi)

## About us

- Achal Vyas (linkedin:[Achal Vyas](https://www.linkedin.com/in/achal-vyas-862a43146/)): A graduate student pursuing Masters in Robotics at the University of Maryland - College Park. He is a part of Autonomous Micro Aerial Vehicles Team and has interests in Computer Vision and Machine Learning. 
- Pruthvi Sanghavi (linkedin:[Pruthvi Sanghavi](https://www.linkedin.com/in/pruthvi-sanghavi/)): A graduate student pursuing Masters in Robotics at the University of Maryland College Park. He is interested in Reinforcement Learning and Dynamics.

## Disclaimer
```
This software is released under BSD 3 Clause License.
```

## Overview

We propose to design "TenezBot" a tennis ball collecting robot, by incorporating high-quality software engineering practices for Acme Robotics. "TenezBot" package is a complete software package which will be integrated in their new line of products. 
Tennis is played widely throughout the world. Mastering this game requires a good amount of time. But with that comes the demanding task of collecting hundreds of tennis balls scattered across the court, which becomes really frustrating after a tiring practice session. Thus we are aiming to develop “TenezBot” a robot which can detect ball, its position in the environment, reach out to the ball and collect it in a sac using a custom made collector attached with the TurtleBot base platform. The robot would use vision camera for vision and depth measurement. Path planning algorithms are used to reach out to the balls so that the task performance is optimum.

## Agile Iteration Process (AIP)

- [Team Review Notes](https://docs.google.com/document/d/1xD4v_xm90qUi-JRTBeB70zwtmSkE1hAyr0sW1udbbTA/edit)
- [AIP Spreadsheet](https://docs.google.com/spreadsheets/d/17ZCHpeQtKOI61sXRT4afAt2YDU3qQ2JU7icFp7B_i5g/edit#gid=0)

## Approach

The robot would perform the following steps to complete its task:
- Sense the environment and detect ball using computer vision algorithms.
- Calculate the distance to the nearest ball.
- Plan the path from current position to the ball position coordinate.
- Reach the ball coordinate and collect the ball in sac.
- Then calculate the distance to the other nearest target.

## Dependencies

- Ubuntu Xenial 16.04 [link](http://releases.ubuntu.com/16.04/)
- ROS Kinetic [link](http://wiki.ros.org/kinetic)
- Gazebo [link](http://gazebosim.org/)
- Git 2.7.4
- Opencv [link](https://opencv.org/)
- ROS Navigation stack
- ROS Geometry MOdules
- Gtest

## Outline of ROS Messages and Services
- ROS Navigation Stack: It uses tf transform tree to maintain the model, messages used are geometry_msgs/twists, std_msgs, gen_msgs, nav_msgs/GridCells, nav_msgs/MapMetaData, nav_msgs/OccupancyGrid, nav_msgs/Odometry, nav_msgs/Path.

## Build, Test and Run Instructions

### Building Workspace and Packages

```


```

### Running the packages

```


```

### Testing the packages

```


```

## Assumptions and Known Bugs/Issues

### Assumptions

- The shape, colour and size of the balls are predefined.
- There are no obstacles in the work space of the robot.
- All the Balls are in have a stationary state.
- The dimensions of the work area of the robot is predefined.

### Known Bugs/Issues

```


```

## Developer Level Documentation

```


```

## Graphics/Videos/GIFS as demo and project promotion

```


```


