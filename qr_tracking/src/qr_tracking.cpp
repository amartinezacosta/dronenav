#include <qr_tracking.hpp>

namespace qr_tracking
{
  QRTracking::QRTracking(ros::NodeHandle nh, ros::NodeHandle pvt_nh) : 
    m_nh(nh),
    m_pvt_nh(pvt_nh),
    m_it(nh)
  {
    /*Parameters*/
    m_pvt_nh.param<std::string>("image_topic", m_image_topic, "camera/raw/rgb");

    /*Subscribers*/
    m_image_sub = m_it.subscribe(m_image_topic, 10, &QRTracking::image_callback, this);
  
    /*zbar configuration*/
    m_scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  }

  QRTracking::~QRTracking()
  {

  }

  void QRTracking::image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    zbar::Image z_image(image.cols, image.rows, "Y800", image.data);
    m_scanner.scan(z_image);

    for(zbar::Image::SymbolIterator symbol = z_image.symbol_begin();
      symbol != z_image.symbol_end(); ++symbol)
    {
      ROS_DEBUG_NAMED("QR Tracker", "Decoded %s, Symbol \"%s\"",
        symbol->get_type_name().c_str(), 
        symbol->get_data().c_str());
    }

    z_image.set_data(NULL, 0);
  }

}