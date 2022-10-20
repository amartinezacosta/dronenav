#include <image_saver.hpp>

namespace dronenav_actions
{
  ImageSaver::ImageSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh) :
    m_nh(nh),
    m_pvt_nh(pvt_nh),
    m_store_image(false),
    m_action_server(nh, "dronenav/save_image", 
      boost::bind(&ImageSaver::action_callback, this, _1), 
      false)
  {
    /*Parameters*/
    m_pvt_nh.param<std::string>("image_topic", m_image_topic, "camera/rgb/image_raw");
    m_pvt_nh.param("min_delay", m_min_delay, 1.0);
    m_pvt_nh.param("max_delay", m_max_delay, 30.0);

    /*Subscribers*/
    image_transport::ImageTransport it(m_nh);
    m_image_sub = it.subscribe(m_image_topic, 10, 
      &ImageSaver::image_callback, this);

    /*Start action server*/
    ROS_INFO_NAMED("dronenav_actions", "Starting Image Action Server");
    m_action_server.start();
  }

  ImageSaver::~ImageSaver()
  {

  }

  void ImageSaver::image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if(!m_store_image) return;

    try
    {
      m_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch(cv_bridge::Exception& e)
    {
      m_image.release();
      ROS_ERROR("Image saver cv_bridge exception: %s", e.what());
      return;
    }
  }

  void ImageSaver::action_callback(const dronenav_msgs::SaveImageGoalConstPtr& goal)
  {
    ROS_INFO_NAMED("dronenav_actions", "Image save action requested");

    m_store_image = true;
    bool success = true;
    double delay = goal->delay;

    if(delay > m_max_delay) delay = m_max_delay;
    if(delay < m_min_delay) delay = m_min_delay;

    m_feedback.count = 0;
    m_result.count = 0;

    /*Wait for the image callback to record the first image*/
    ros::Duration(1.0).sleep();

    for(int i = 0; i < goal->count; i++)
    {
      if(m_action_server.isPreemptRequested() || !ros::ok())
      {
        m_action_server.setPreempted();
        success = false;
        break;
      }

      if(!m_image.empty())
      { 
        std::stringstream file_name;
        file_name << goal->file_name << "_" << std::setw(3) <<
          std::setfill('0') << i << ".jpg";

        ROS_INFO("Saving image: %s", file_name.str().c_str());
        cv::imwrite(file_name.str(), m_image);
        m_feedback.count++;
      }

      m_action_server.publishFeedback(m_feedback);
      ros::Duration(goal->delay).sleep();
    }

    if(success) 
    {
       m_result.count = m_feedback.count;       
       m_action_server.setSucceeded(m_result);
    }

    m_store_image = false;
  }
}