#ifndef DRONENAV_TEST_FIXTURE_HPP
#define DRONENAV_TEST_FIXTURE_HPP

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <dronenav_msgs/Takeoff.h>
#include <dronenav_msgs/Land.h>
#include <dronenav_msgs/Waypoint.h>
#include <dronenav_msgs/Status.h>
#include <dronenav_msgs/GenerateWaypoints.h>

class DronenavTestFixture : public ::testing::Test
{
  public:
  void SetUp()
  {
    /*Publishers*/
    waypoint_pub = nh.advertise<dronenav_msgs::Waypoint>("dronenav/waypoint", 100);

    /*Subscribers*/
    status_sub = nh.subscribe<dronenav_msgs::Status>("dronenav/status", 50,
      &DronenavTestFixture::status_callback, this);

    /*Service clients*/
    takeoff_client = nh.serviceClient<dronenav_msgs::Takeoff>("dronenav/takeoff");
    land_client = nh.serviceClient<dronenav_msgs::Land>("dronenav/land");
    waypoint_generation_client = nh.serviceClient<
      dronenav_msgs::GenerateWaypoints>("dronenav/waypoint_generation/generate");

    /*South wall inspection*/
    viewport.x = 1.0;
    viewport.y = 0.0;
    viewport.z = 0.0;
    min.x = -40.0;
    min.y = -15.0;
    min.z = 2.0;
    max.x = -20.0;
    max.y = 15.0;
    max.z = 10.0;
    downsample = 20;
  }

  void TearDown()
  {

  }

  void status_callback(const dronenav_msgs::Status::ConstPtr& msg){ status = *msg;}
  void wait_for_state(const std::string state, double time)
  {
    double t0 = ros::Time::now().toSec();
    while((status.state != state) &&
      ((ros::Time::now().toSec() - t0) < time));
  }

  void wait_for_queue(double time)
  {
    double t0 = ros::Time::now().toSec();
    while(status.waypoint_queue_size && 
      ((ros::Time::now().toSec() - t0) < time));
  }

  public:
  ros::NodeHandle nh;

  /*Publishers*/
  ros::Publisher waypoint_pub;

  /*Subscribers*/
  ros::Subscriber status_sub;

  /*Service clients*/
  ros::ServiceClient takeoff_client;
  ros::ServiceClient land_client;
  ros::ServiceClient waypoint_generation_client;

  /*dronenav variables*/
  dronenav_msgs::Status status;

  /*Waypoint generation variables*/
  std::vector<dronenav_msgs::Waypoint> waypoints;
  geometry_msgs::Point viewport;
  geometry_msgs::Point min;
  geometry_msgs::Point max;
  int downsample;
};

#endif