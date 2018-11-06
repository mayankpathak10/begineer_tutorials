# ROS Beginner Tutorials - Week10_HW Branch
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)


## Overview

In this project, simple ROS publisher and subscriber were implemented in C++. The publisher and subscriber nodes were designed as explained in the ROS tutorials. The publisher node is modified to publish a custom string instead of the one given by the tutorial.

The communication between nodes is done with a service that can be found in the ```srv``` folder. The package was updated by adding the service code in the talker node so that when commanded, the published string changes by using the service. The publisher node file (talker.cpp) is modified to use all logging levels: INFO, DEBUG, WARN, ERROR, and FATAL.

A launch file is created to run both the nodes at the same time with single command. An argument for changing the publisher frequency is added for the launch file. A default frequency of 10 is set.

The outputs of cpplint and cppcheck are added to the outputs folder of this repository

Below are instructions to build and run the nodes.

## Dependencies

* ROS Kinetic
* Catkin
* roscpp package
* std_msgs package
* message_generation package

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
git checkout Week10_HW
cd ..
catkin_make
```



## How to run (using rosrun)

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
[ INFO] [1541470046.441127979]: Custom String Inserted. Message# 1 Published
```
and the output of listner node will look like:
```
[ INFO] [1541470046.442204251]: Message Received by Subscriber, [Custom String Inserted. Message# 1 Published] 
```

## How to run (using roslaunch)

Here a launch file is created and executing this single launch file will run all the nodes associated with this package. Follow the below steps to run the package using using roslauch.
* First close all terminals that are associated with this package.
* running ```roscore``` is not necessary.
* Now run the following commands in the terminal:
```
cd ros_ws
source ./devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch frequency:=5
```
To change the frequency of publishing messages, input the value as argument to 	```frequency``` in the above command. By default, it is set to 10 Hz.

The output of the above command looks like:
```
[ INFO] [1541470046.441127979]: Custom String Inserted. Message# 1 Published
[ INFO] [1541470046.442204251]: Message Received by Subscriber, [Custom String Inserted. Message# 1 Published] 
```
## How to call the service

A service for changing string is created in this assignment, which can be accessed with the below steps.

* Once the nodes are running, in a new terminal window, use ```rosservice list``` to check if the service ```/change_string``` is listed.
* If the service is listed, you can call the service as below:
```
cd ros_ws
source ./devel/setup.bash
rosservice call /change_string stringName
```
Previous string will be replaced by the new string ```stringName```.
