#include <video_saver.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "video_saver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pvt_nh;

  /*Image saver*/
  dronenav_actions::VideoSaver saver(nh, pvt_nh);

  ros::spin();
  return 0;
}