#ifndef DRONENAV_TEST_FIXTURE_HPP
#define DRONENAV_TEST_FIXTURE_HPP

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <image_saver.hpp>
#include <video_saver.hpp>

#include <dronenav_msgs/ImageSave.h>
#include <dronenav_msgs/PointCloudSave.h>
#include <dronenav_msgs/VideoSave.h>

class DronenavTestFixture : public ::testing::Test
{
  public:
  void SetUp()
  {
    /*Service clients*/
    image_save_client = nh.serviceClient<dronenav_msgs::ImageSave>("dronenav/save/image");
    video_save_client = nh.serviceClient<dronenav_msgs::VideoSave>("dronenav/save/video");
    
    /*Subscribers*/

    /*Publishers*/
  }

  void TearDown()
  {

  }

  ros::NodeHandle nh;
  ros::NodeHandle pvt_nh;

  /*Service clients*/
  ros::ServiceClient image_save_client;
  ros::ServiceClient video_save_client;

  /*Subscribers*/

  /*Publisher*/
  
};

#endif