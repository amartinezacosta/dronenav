#ifndef DRONENAV_TEST_FIXTURE_HPP
#define DRONENAV_TEST_FIXTURE_HPP

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <dronenav_msgs/Takeoff.h>
#include <dronenav_msgs/Land.h>
#include <dronenav_msgs/Waypoint.h>
#include <dronenav_msgs/Status.h>
#include <dronenav_msgs/GlobalGoal.h>
#include <dronenav_msgs/PlannerStatus.h>

#include <waypoint_generation.hpp>

#define NFAC_SECTION_0_SEGMENTATIONS  4

class DronenavTestFixture : public ::testing::Test
{
  public:
  void SetUp()
  {
    /*nfac section 0 viewports and wall segmentations*/
    
    /*North wall*/
    viewports[0].x = -1.0;
    viewports[0].y = 0.0;
    viewports[0].z = 0.0;

    min[0].x = 10.0;
    min[0].y = -15.0;
    min[0].z = 2.0;

    max[0].x = 30.0;
    max[0].y = 15.0;
    max[0].z = 27.0;

    downsamples[0] = 1;

    /*East wall*/
    viewports[1].x = 0.0;
    viewports[1].y = -1.0;
    viewports[1].z = 0.0;

    min[1].x = -25.0;
    min[1].y = 10.0;
    min[1].z = 2.0;

    max[1].x = 10.0;
    max[1].y = 20.0;
    max[1].z = 27.0;

    downsamples[1] = 20;

    /*South wall*/
    viewports[2].x = 1.0;
    viewports[2].y = 0.0;
    viewports[2].z = 0.0;

    min[2].x = -40.0;
    min[2].y = -15.0;
    min[2].z = 2.0;

    max[2].x = -20.0;
    max[2].y = 15.0;
    max[2].z = 27.0;

    downsamples[2] = 20;

    /*West wall*/
    viewports[3].x = 0.0;
    viewports[3].y = 1.0;
    viewports[3].z = 0.0;

    min[3].x = -25.0;
    min[3].y = -20.0;
    min[3].z = 2.0;

    max[3].x = 10.0;
    max[3].y = -14.0;
    max[3].z = 27.0;

    downsamples[3] = 40;
  }

  void TearDown()
  {

  }

  public:
  ros::NodeHandle nh;

  geometry_msgs::Point viewports[NFAC_SECTION_0_SEGMENTATIONS];
  geometry_msgs::Point min[NFAC_SECTION_0_SEGMENTATIONS];
  geometry_msgs::Point max[NFAC_SECTION_0_SEGMENTATIONS];
  int downsamples[NFAC_SECTION_0_SEGMENTATIONS];
};

#endif