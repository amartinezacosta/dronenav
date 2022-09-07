#ifndef QR_TRACKING_HPP_
#define QR_TRACKING_HPP_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <zbar.h>

namespace qr_tracking
{
  class QRTracking
  {
    public:
    QRTracking(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~QRTracking();

    private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);

    private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;

    image_transport::ImageTransport m_it;
    image_transport::Subscriber m_image_sub;

    zbar::ImageScanner m_scanner;

    /*Parameters*/
    std::string m_image_topic;
  };
}

#endif
