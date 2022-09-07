#include <qr_tracking.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_tracking_node");
  ros::NodeHandle nh;
  ros::NodeHandle pvt_nh("~");

  qr_tracking::QRTracking qr_tracker(nh, pvt_nh);

  ros::spin();
  return 0;
}