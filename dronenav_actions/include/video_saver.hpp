#ifndef VIDEO_SAVER_HPP_
#define VIDEO_SAVER_HPP_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <dronenav_msgs/VideoSave.h>

namespace dronenav_actions
{
  class VideoSaver
  {
    public:
    VideoSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~VideoSaver();

    private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    bool video_save_service(dronenav_msgs::VideoSaveRequest& rqt,
      dronenav_msgs::VideoSaveResponse& rsp);

    private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;

    /*Service server*/
    ros::ServiceServer m_video_save_service;

    /*Subscribers*/
    image_transport::Subscriber m_image_sub;

    /*Variables*/
    double m_start, m_end;
    std::string m_video_name;
    cv::VideoWriter m_video_writer;
    std::vector<cv::Mat> m_frames;

    /*Parameters*/
    std::string m_image_topic;
  };
}

#endif