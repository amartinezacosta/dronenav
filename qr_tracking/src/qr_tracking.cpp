#include <qr_tracking.hpp>

namespace qr_tracking
{
  QRTracking::QRTracking(ros::NodeHandle nh, ros::NodeHandle pvt_nh) : 
    m_nh(nh),
    m_pvt_nh(pvt_nh)
  {
    /*Parameters*/
    m_pvt_nh.param<std::string>("image_topic", m_image_topic, "camera/rgb/image_raw");

    /*Subscribers*/
    image_transport::ImageTransport it(m_nh);
    m_image_sub = it.subscribe(m_image_topic, 10, &QRTracking::image_callback, this);
    m_image_pub = it.advertise("dronenav/tracking/qrcode", 1);
  
    /*zbar configuration*/
    m_scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  }

  QRTracking::~QRTracking()
  {

  }

  void QRTracking::image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat image_gray;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::cvtColor(cv_ptr->image, image_gray, cv::COLOR_BGR2GRAY);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    zbar::Image z_image(image_gray.cols, image_gray.rows, "Y800", 
      (uchar*)image_gray.data, image_gray.cols*image_gray.rows);
    m_scanner.scan(z_image);

    for(zbar::Image::SymbolIterator symbol = z_image.symbol_begin();
      symbol != z_image.symbol_end(); ++symbol)
    {
      std::cout << "QR tracker, decoded= " << symbol->get_type_name()
        << "Symbol= " << symbol->get_data();
      
      ROS_INFO_NAMED("QR Tracker", "Decoded %s, Symbol \"%s\"",
        symbol->get_type_name().c_str(), 
        symbol->get_data().c_str());

      for(int i = 0; i < symbol->get_location_size(); i++)
      {
        int x = symbol->get_location_x(i);
        int y = symbol->get_location_y(i);

        cv::circle(cv_ptr->image, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1);
      }

      cv::putText(cv_ptr->image, symbol->get_data(), 
        cv::Point(50, 100), 
        cv::FONT_HERSHEY_SIMPLEX, 1.0, 
        cv::Scalar(0, 255, 0), 1); 
    }

    m_image_pub.publish(cv_ptr->toImageMsg());
  }

}