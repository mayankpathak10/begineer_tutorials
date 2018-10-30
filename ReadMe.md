# ROS Beginner Tutorials - Week 10
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


## Overview

In this project, simple ROS publisher and subscriber were implemented in C++. The publisher and subscriber nodes were designed as explained in the ROS tutorials. The publisher node was modified to publish a custom string instead of the one given by the tutorial.

The communication between nodes is done with a service that can be found in the `srv` folder. The package was updated by adding the service code in the talker node so that when commanded, the published string changes by using the service.

The outputs of cpplint and cppcheck are added to the outputs folder of this repository

Below are instructions to build and run the nodes.

## Dependencies

* ROS Kinetic
* Catkin
* roscpp package
* std_msgs package
* genmsg package

## How to build

* Create a catkin workspace if it was not created previously:

```
mkdir ros_ws
cd ros_ws
mkdir src
catkin_make
```
* Clone the repository and build the package:
```
cd ros_ws
cd src
git clone -b Week10_HW https://github.com/mayankpathak10/beginner_tutorials.git
cd ..
catkin_make
```



## How to run

* In a first terminal run:

```
roscore
```
* In a second terminal:
```
cd ros_ws
source devel/setup.bash
rosrun beginner_tutorials talker
```
* In a third terminal:
```
cd ros_ws
source devel/setup.bash
rosrun beginner_tutorials listner
```

The output of talker node will look like:
```
[ INFO] [1540938283.640045415]: ENPM 808X: Publisher(talker) Node running!! Message #1Published
```
and the output of listner node will look like:
```
[ INFO] [1540938285.840936112]: Message Received by Subscriber, [ENPM 808X: Publisher(talker) Node running!! Message #1 Published] 
```
