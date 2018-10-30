# ROS Beginner Tutorials - Week 10

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

## Dependencies

* ROS Kinetic
* Catkin
* roscpp package
* std_msgs package
* genmsg package

## How to run

* In a first terminal run:

```
roscore
```
* In a second terminal:
```
cd ros_ws
source devel/setup.bash
rosrun <the package file>
```