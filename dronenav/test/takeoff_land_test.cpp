#include <ros/ros.h>
#include <gtest/gtest.h>

#include <dronenav.hpp>
#include <navigation.hpp>
#include <landed.hpp>
#include <flying.hpp>
#include <hovering.hpp>
#include <positioning.hpp>
#include <reached.hpp>

TEST(dronenav_tests, takeoff_land)
{
  ros::NodeHandle nh;
  ros::NodeHandle pvt_nh;

  dronenav::Drone drone(nh, pvt_nh);
  drone.initiate();

  ros::ServiceClient takeoff_client = nh.serviceClient<dronenav_msgs::Takeoff>("dronenav/takeoff");
  ros::ServiceClient land_client = nh.serviceClient<dronenav_msgs::Land>("dronenav/land");

  /*Takeoff request*/
  dronenav_msgs::Takeoff takeoff_srv;
  takeoff_srv.request.x = 0.0;
  takeoff_srv.request.y = 0.0;
  takeoff_srv.request.z = 3.0;
  takeoff_srv.request.yaw = 0.0;

  takeoff_client.call(takeoff_srv);
  ros::spinOnce();

  ASSERT_TRUE(takeoff_srv.response.success);

  /*Wait until we have take off and in position*/
  ros::Duration(10.0).sleep();

  /*Land request*/
  dronenav_msgs::Land land_srv;
  land_srv.request.override_takeoff_pose = false;

  land_client.call(land_srv);
  ros::spinOnce();

  ASSERT_TRUE(land_srv.response.success);

  ros::Duration(10.0).sleep();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "takeoff_landing_test");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return result;
}
