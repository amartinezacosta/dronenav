#ifndef DRONENAV_TEST_FIXTURE_HPP
#define DRONENAV_TEST_FIXTURE_HPP

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <dronenav_msgs/Takeoff.h>
#include <dronenav_msgs/Land.h>
#include <dronenav_msgs/Waypoint.h>
#include <dronenav_msgs/Status.h>
#include <dronenav_msgs/GlobalGoal.h>

class DronenavTestFixture : public ::testing::Test
{
  public:
  void SetUp()
  {
    /*Service clients*/
    takeoff_client = nh.serviceClient<dronenav_msgs::Takeoff>("dronenav/takeoff");
    land_client = nh.serviceClient<dronenav_msgs::Land>("dronenav/land");
  
    /*Subscribers*/
    status_sub = nh.subscribe<dronenav_msgs::Status>("dronenav/status", 50, 
      &DronenavTestFixture::status_callback, this);

    /*Publishers*/
    global_goal_pub = nh.advertise<dronenav_msgs::GlobalGoal>("dronenav/global_planner/goal", 10);
    waypoint_pub = nh.advertise<dronenav_msgs::Waypoint>("dronenav/waypoint", 100);
  }

  void TearDown()
  {

  }

  void status_callback(const dronenav_msgs::Status::ConstPtr& msg)
  {
    status = *msg;
  }

  void wait_for_state(const std::string state, double time)
  {
    double t0 = ros::Time::now().toSec();
    while((status.state != state) &&
      ((ros::Time::now().toSec() - t0) < time));
  }

  ros::NodeHandle nh;

  /*Service clients*/
  ros::ServiceClient takeoff_client;
  ros::ServiceClient land_client;

  /*Subscribers*/
  ros::Subscriber status_sub;

  /*Publisher*/
  ros::Publisher waypoint_pub;
  ros::Publisher global_goal_pub;

  /*Variables*/
  dronenav_msgs::Status status;
};

#endif