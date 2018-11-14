

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


int main(int argc,
         char **argv) {
    ros::init(argc, argv, "test_talker");
    // nh.reset(new ros::NodeHandle);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
