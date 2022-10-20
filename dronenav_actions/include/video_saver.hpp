#ifndef VIDEO_SAVER_HPP_
#define VIDEO_SAVER_HPP_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <dronenav_msgs/SaveVideoAction.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace dronenav_actions
{
  class VideoSaver
  {
    public:
    VideoSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~VideoSaver();

    private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void action_callback(const dronenav_msgs::SaveVideoGoalConstPtr& goal);

    private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;

    /*Action server*/
    actionlib::SimpleActionServer<
      dronenav_msgs::SaveVideoAction> m_action_server;

    /*Subscribers*/
    image_transport::Subscriber m_image_sub;

    /*Variables*/
    bool m_store_frame;
    double m_start;
    //cv::VideoWriter m_video_writer;
    cv::Mat m_frame;
    dronenav_msgs::SaveVideoFeedback m_feedback;
    dronenav_msgs::SaveVideoResult m_result; 

    /*Parameters*/
    std::string m_image_topic;
  };
}

#endif