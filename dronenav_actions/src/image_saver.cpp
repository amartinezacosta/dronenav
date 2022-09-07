#include <image_saver.hpp>

namespace dronenav_actions
{
  ImageSaver::ImageSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh) :
    m_nh(nh),
    m_pvt_nh(pvt_nh),
    m_save_count(0),
    m_it(nh)
  {
    /*Parameters*/
    m_pvt_nh.param<std::string>("image_topic", m_image_topic, "camera/rgb/raw");

    /*Subscribers*/
    m_image_sub = m_it.subscribe(m_image_topic, 10, 
      &ImageSaver::image_callback, this);

    /*Service servers*/
    m_image_save_service = m_nh.advertiseService("dronenav/save/image", 
      &ImageSaver::image_save_service, this);
  }

  ImageSaver::~ImageSaver()
  {

  }

  void ImageSaver::image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("Image saver cv_bridge exception: %s", e.what());
      return;
    }

    /*Save images*/
    if(m_save_count)
    {
      std::stringstream filename;
      filename << m_image_name << std::setw(3) <<
        std::setfill('0') << m_save_count++ << ".jpg";
      cv::imwrite(filename.str(), image);
      m_save_count--;
    }
  }

  bool ImageSaver::image_save_service(dronenav_msgs::ImageSaveRequest& rqt,
    dronenav_msgs::ImageSaveResponse& rsp)
  {
    m_save_count = rqt.count;
    m_image_name = rqt.name;
    rsp.success = true;

    return true;
  }
}