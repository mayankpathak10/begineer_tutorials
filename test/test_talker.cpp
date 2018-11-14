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
 *
 *@copyright Copyright 2018 Mayank Pathak
 *@file test_talker.cpp
 *@author Mayank  Pathak
 *@brief This file is a test file to implement google test cases.
 */

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/service.h"

// std::shared_ptr<ros::NodeHandle> nh;

/**
 *@brief Declare a test to check if service has been created and is empty.
 */
TEST(TestSuite, ServiceTest) {
    ros::NodeHandle n;
    beginner_tutorials::service srv;
    ros::ServiceClient client =
        n.serviceClient<beginner_tutorials::service>("change_string");

    /*Tests that the service exists
     * and that it has changed the string correctly
     */
    EXPECT_EQ(srv.response.b, srv.request.a);
}

/**
 * @brief      main function to initiate test
 *
 * @param[in]  argc  The argc
 * @param[in]  argv  The argv
 *
 * @return     returns if the test runs
 */
int main(int argc,
         char **argv) {
    ros::init(argc, argv, "test_talker");
    // nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
