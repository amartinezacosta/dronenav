#ifndef DRONENAV_TEST_FIXTURE_HPP
#define DRONENAV_TEST_FIXTURE_HPP

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dronenav_msgs/Takeoff.h>
#include <dronenav_msgs/Land.h>
#include <dronenav_msgs/Waypoint.h>
#include <dronenav_msgs/Status.h>
#include <dronenav_msgs/GenerateWaypoints.h>
#include <dronenav_msgs/GlobalGoal.h>
#include <dronenav_msgs/PlannerStatus.h>
#include <dronenav_msgs/TrackedCodes.h>

class DronenavTestFixture : public ::testing::Test
{
  public:
  void SetUp()
  {
    /*Publishers*/
    path_goal_pub = nh.advertise<dronenav_msgs::GlobalGoal>(
      "dronenav/global_planner/goal", 
      100);
    waypoint_pub = nh.advertise<dronenav_msgs::Waypoint>("dronenav/waypoint", 100);

    /*Subscribers*/
    dronenav_status_sub = nh.subscribe<dronenav_msgs::Status>(
      "dronenav/status", 50,
      &DronenavTestFixture::dronenav_status_callback, this);
    planner_status_sub = nh.subscribe<dronenav_msgs::PlannerStatus>(
      "dronenav/global_planner/status", 50,
      &DronenavTestFixture::planner_status_callback, this);
    qr_detection_sub = nh.subscribe<dronenav_msgs::TrackedCodes>(
      "dronenav/qr_tracker/tracked", 50,
      &DronenavTestFixture::qr_detection_callback, this);

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
    min.y = -8.0;
    min.z = 2.0;
    max.x = -20.0;
    max.y = 8.0;
    max.z = 10.0;
    downsample = 20;
  }

  void TearDown()
  {

  }

  void dronenav_status_callback(
    const dronenav_msgs::Status::ConstPtr& msg){ dronenav_status = *msg;}
  void planner_status_callback(
    const dronenav_msgs::PlannerStatus::ConstPtr& msg){ planner_status = *msg;}
  
  void qr_detection_callback(const dronenav_msgs::TrackedCodes::ConstPtr& msg)
  {
    /*Post detections to global planner*/
    for(dronenav_msgs::Code detection : msg->codes)
    {
      dronenav_msgs::GlobalGoal detection_goal;
      detection_goal.x = detection.position.x + 1.0*detection.normal.x;
      detection_goal.y = detection.position.y + 1.0*detection.normal.y;
      detection_goal.z = detection.position.z + 1.0*detection.normal.z;
      
      /*Priority should be bigger than 0*/
      detection_goal.priority = 1;

      /*Computer yaw angle from detection surface normal*/
      tf2::Vector3 normal(
        detection.normal.x,
        detection.normal.y,
        detection.normal.z);

      tf2::Vector3 direction(
        -detection.normal.x,
        -detection.normal.y, 
        -detection.normal.z);

      detection_goal.yaw = normal.angle(direction);

      path_goal_pub.publish(detection_goal);
    }
  }

  void wait_for_state(const std::string state, double time)
  {
    double t0 = ros::Time::now().toSec();
    while((dronenav_status.state != state) &&
      ((ros::Time::now().toSec() - t0) < time));
  }

  void wait_for_waypoint_queue(double time)
  {
    double t0 = ros::Time::now().toSec();
    while(dronenav_status.waypoint_queue_size && 
      ((ros::Time::now().toSec() - t0) < time));
  }

  void wait_for_planner_queue(double time)
  {
    double t0 = ros::Time::now().toSec();
    while(planner_status.queue_size && 
      ((ros::Time::now().toSec() - t0) < time));
  }

  public:
  ros::NodeHandle nh;

  /*Publishers*/
  ros::Publisher waypoint_pub;
  ros::Publisher path_goal_pub;

  /*Subscribers*/
  ros::Subscriber dronenav_status_sub;
  ros::Subscriber planner_status_sub;
  ros::Subscriber qr_detection_sub;

  /*Service clients*/
  ros::ServiceClient takeoff_client;
  ros::ServiceClient land_client;
  ros::ServiceClient waypoint_generation_client;

  /*dronenav variables*/
  dronenav_msgs::Status dronenav_status;
  dronenav_msgs::PlannerStatus planner_status;

  /*Waypoint generation variables*/
  std::vector<dronenav_msgs::Waypoint> waypoints;
  geometry_msgs::Point viewport;
  geometry_msgs::Point min;
  geometry_msgs::Point max;
  int downsample;
};

#endif