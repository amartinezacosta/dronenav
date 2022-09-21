#include <video_saver.hpp>

namespace dronenav_actions
{
    VideoSaver::VideoSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh)
      : m_start(0.0), m_end(0.0)
    {
      /*Parameters*/
      m_pvt_nh.param<std::string>("image_topic", m_image_topic, "camera/rgb/image_raw");

      /*Subscribers*/
      image_transport::ImageTransport it(m_nh);
      m_image_sub = it.subscribe(m_image_topic, 10, 
        &VideoSaver::image_callback, this);

      /*Service servers*/
      m_video_save_service = m_nh.advertiseService("dronenav/save/video", 
        &VideoSaver::video_save_service, this);
    }

    VideoSaver::~VideoSaver()
    {

    }

    void VideoSaver::image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
      if((m_start == 0.0) && (m_end == 0.0)) return;

      try
      {
        m_frames.push_back(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR("Image saver cv_bridge exception: %s", e.what());
        /*We tried to save this, lower count and return*/
        return;
      }

      if((ros::Time::now().toSec() - m_start) > m_end)
      {
        //Stop taking frames save video format
        for(cv::Mat frame : m_frames)
        {
          m_video_writer.write(frame);
        }

        //cleanup
        m_start = 0.0;
        m_end = 0.0;
        m_frames.clear();
        m_video_writer.release();
      }
    }
    
    bool VideoSaver::video_save_service(dronenav_msgs::VideoSaveRequest& rqt,
      dronenav_msgs::VideoSaveResponse& rsp)
    {
      m_start = ros::Time::now().toSec();
      m_end = rqt.duration;
      m_video_name = rqt.name;

      m_video_writer.open(m_video_name,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        10, 
        cv::Size(640, 480));

      rsp.success = true;

      return true;
    }
}