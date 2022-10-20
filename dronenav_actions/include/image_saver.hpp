#ifndef IMAGE_SAVER_HPP_
#define IMAGE_SAVER_HPP_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <dronenav_msgs/SaveImageAction.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>

namespace dronenav_actions
{
  class ImageSaver
  {
    public:
    ImageSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~ImageSaver();

    private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void action_callback(const dronenav_msgs::SaveImageGoalConstPtr& goal);

    private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;

    /*Action server*/
    actionlib::SimpleActionServer<
      dronenav_msgs::SaveImageAction> m_action_server;

    /*Image transport subscriber*/
    image_transport::Subscriber m_image_sub;

    /*Variables*/
    bool m_store_image;
    cv::Mat m_image;
    dronenav_msgs::SaveImageFeedback m_feedback;
    dronenav_msgs::SaveImageResult m_result;

    /*Parameters*/
    std::string m_image_topic;
    double m_min_delay;
    double m_max_delay;
  };
}

#endif