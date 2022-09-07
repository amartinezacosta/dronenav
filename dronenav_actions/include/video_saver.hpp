#ifndef VIDEO_SAVER_HPP_
#define VIDEO_SAVER_HPP_

#include <ros/ros.h>

namespace dronenav_actions
{
  class VideoSaver
  {
    public:
    VideoSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~VideoSaver();

    private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;
  };
}

#endif