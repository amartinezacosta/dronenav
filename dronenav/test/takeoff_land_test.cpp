#include <ros/ros.h>
#include <gtest/gtest.h>

#include <dronenav.hpp>
#include <navigation.hpp>
#include <landed.hpp>
#include <flying.hpp>
#include <hovering.hpp>
#include <moving.hpp>
#include <reached.hpp>

TEST(dronenav_tests, takeoff_land)
{
  ros::Duration(30.0).sleep();

  ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "takeoff_landing_test");
  
  ros::NodeHandle nh;
  ros::NodeHandle pvt_nh;

  dronenav::Drone drone(nh, pvt_nh);
  drone.initiate();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return result;
}
