#ifndef DRONENAV_TEST_FIXTURE_HPP
#define DRONENAV_TEST_FIXTURE_HPP

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <image_saver.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <dronenav_msgs/SaveImageAction.h>
#include <dronenav_msgs/SavePointCloudAction.h>
#include <dronenav_msgs/SaveVideoAction.h>

class DronenavTestFixture : public ::testing::Test
{
  public:
  void SetUp()
  {
    /*Subscribers*/

    /*Publishers*/
  }

  void TearDown()
  {

  }

  ros::NodeHandle nh;

  /*Subscribers*/

  /*Publisher*/
  
};

#endif