#include <video_saver.hpp>

namespace dronenav_actions
{
    VideoSaver::VideoSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh)
      : m_start(0.0), 
        m_store_frame(false),
        m_action_server(nh, "dronenav/save_video",
        boost::bind(&VideoSaver::action_callback, this, _1),
        false)
    {
      /*Parameters*/
      m_pvt_nh.param<std::string>("image_topic", m_image_topic, "camera/rgb/image_raw");

      /*Subscribers*/
      image_transport::ImageTransport it(m_nh);
      m_image_sub = it.subscribe(m_image_topic, 10, 
        &VideoSaver::image_callback, this);

      ROS_INFO_NAMED("dronenav_actions", "Starting Video Action Server");
      m_action_server.start();
    }

    VideoSaver::~VideoSaver()
    {

    }

    void VideoSaver::image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
      if(!m_store_frame) return;
      
      try
      {
        m_frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        // m_frames.push_back(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);
      }
      catch(cv_bridge::Exception& e)
      {
        m_frame.release();
        ROS_ERROR("Image saver cv_bridge exception: %s", e.what());
        return;
      }
    }
    
    void VideoSaver::action_callback(const dronenav_msgs::SaveVideoGoalConstPtr& goal)
    {
      m_store_frame = true;
      bool success = true;

      m_feedback.frame_count = 0;
      m_feedback.time = 0.0;
      m_result.duration = 0.0;

      cv::VideoWriter video_writer;
      video_writer.open(goal->file_name,
        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
        24, 
        cv::Size(640, 480));

      m_start = ros::Time::now().toSec();

      while((ros::Time::now().toSec() - m_start) < goal->duration)
      {
        if(m_action_server.isPreemptRequested() || !ros::ok())
        {
          m_action_server.setPreempted();
          success = false;
          break;
        }

        if(!m_frame.empty())
        {
          //m_frames.push_back(m_frame);
          video_writer.write(m_frame);
          m_feedback.frame_count++;
        }

        m_feedback.time = ros::Time::now().toSec() - m_start;
        m_action_server.publishFeedback(m_feedback);
        ros::Duration(0.0416).sleep();
      }

      video_writer.release();

      if(success)
      {
        m_result.duration = m_feedback.time;
        m_action_server.setSucceeded(m_result);
      }

      m_store_frame = false;
    }
}