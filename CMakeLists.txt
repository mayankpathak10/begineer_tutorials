cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rospy
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Num.msg
    )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
service.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package()

#####################################
## Build  talker and listener nodes##
#####################################


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(BEFORE ${GTEST_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(test_talker launch/test_talker.launch test/test_talker.cpp)
  target_link_libraries(test_talker ${catkin_LIBRARIES})
endif()