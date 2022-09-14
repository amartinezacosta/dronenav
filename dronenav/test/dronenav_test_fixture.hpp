#ifndef DRONENAV_TEST_FIXTURE_HPP
#define DRONENAV_TEST_FIXTURE_HPP

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <dronenav_msgs/Takeoff.h>
#include <dronenav_msgs/Land.h>
#include <dronenav_msgs/Waypoint.h>
#include <dronenav_msgs/Status.h>

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
    waypoint_pub = nh.advertise<dronenav_msgs::Waypoint>("dronenav/waypoint", 100);

    /*Intialize waypoints*/
    waypoint[0].position.x = 3.0;
    waypoint[0].position.y = 3.0;
    waypoint[0].position.z = 3.0;
    waypoint[0].yaw = 0.785;

    waypoint[1].position.x = 3.0;
    waypoint[1].position.y = -3.0;
    waypoint[1].position.z = 3.0;
    waypoint[1].yaw = 0.0;

    waypoint[2].position.x = -3.0;
    waypoint[2].position.y = -3.0;
    waypoint[2].position.z = 3.0;
    waypoint[2].yaw = 4.712;

    waypoint[3].position.x = -3.0;
    waypoint[3].position.y = 3.0;
    waypoint[3].position.z = 3.0;
    waypoint[3].yaw = 3.141;

    waypoint[4].position.x = 3.0;
    waypoint[4].position.y = 3.0;
    waypoint[4].position.z = 3.0;
    waypoint[4].yaw = 1.570;
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
  ros::NodeHandle pvt_nh;

  /*Service clients*/
  ros::ServiceClient takeoff_client;
  ros::ServiceClient land_client;

  /*Subscribers*/
  ros::Subscriber status_sub;

  /*Publisher*/
  ros::Publisher waypoint_pub;

  /*Variables*/
  dronenav_msgs::Status status;
  dronenav_msgs::Waypoint waypoint[5];
};

#endif