#ifndef IMAGE_SAVER_HPP_
#define IMAGE_SAVER_HPP_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>

#include <dronenav_msgs/ImageSave.h>

namespace dronenav_actions
{
  class ImageSaver
  {
    public:
    ImageSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~ImageSaver();

    private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    bool image_save_service(dronenav_msgs::ImageSaveRequest& rqt,
      dronenav_msgs::ImageSaveResponse& rsp);

    private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;

    /*Service server*/
    ros::ServiceServer m_image_save_service;

    /*Image transport subscriber*/
    image_transport::Subscriber m_image_sub;

    /*Variables*/
    int m_save_count;
    std::string m_image_name;

    /*Parameters*/
    std::string m_image_topic;
  };
}

#endif