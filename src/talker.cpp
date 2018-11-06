/**
 *MIT License

Copyright (c) 2018 Mayank Pathak

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 *@copyright Copyright 2018 Mayank Pathak
 *@file talker.cpp
 *@author Mayank Pathak
 *@brief This file is a part of tutorial that demonstrates
 *       simple receipt of messages over the ROS system.
 *       ROS listener node that receives messages.
 */

#include <string>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/service.h"

// Default string to publish in message.
std::string custom_string = "Custom String Inserted. Message# ";  //NOLINT

/**
 *@brief Function to provide service that changes the string to publish.
 *@param req is the request type defined in the srv file
 *@param res is the response type defined in the srv file
 *@return true if everything works
 */
bool change_string(beginner_tutorials::service::Request  &req, //NOLINT
                   beginner_tutorials::service::Response &res) {  //NOLINT
    custom_string = req.a;  // "a" is the input string in the service
    res.b = custom_string;  // "b" is the output string of the service
    ROS_INFO_STREAM("Custom String is being updated");
    return true;
}

/**
 * @brief The main function is where the talker node is created
 * @param argc is the number of input arguments
 * @param argv is the publisher frequency, given as argument through command line.
 * @return 0 if everything works
 */
int main(int argc, char **argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "talker");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle nh;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher chatter_pub =
        nh.advertise<std_msgs::String>("chatter", 1000);

    // Create the service and advertise over ROS
    ros::ServiceServer service = nh.advertiseService
                                 ("change_string", change_string);

    // Publishing frequency is given as an argument in tutorial.launch
    int freq;
    ROS_INFO_STREAM("Set Publisher frequency value upto(Hz): " << 20);
    freq = std::atoi(argv[1]);  // give frequency the value of argument
    // ERROR Logging level check
    if (freq <= 1)
        ROS_ERROR_STREAM("Invalid publisher frequency");

    // DEBUG Logging level check
    ROS_DEBUG_STREAM("Publisher frequency set to: " << freq);


    // WARN Logging level check
    if (freq <= 2)
        ROS_WARN_STREAM("Frequency too low. Please set to atleast 5 Hz");

    ros::Rate loop_rate(freq);

    // If ROS
    if (!ros::ok())
        ROS_FATAL_STREAM("ROS node not running...");

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok()) {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;

        std::stringstream ss;
        ss << custom_string <<
           count << " Published";
        msg.data = ss.str();

        // ROS_INFO("%s", msg.data.c_str());
        ROS_INFO(msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}
