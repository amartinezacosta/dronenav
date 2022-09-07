#include <pointcloud_saver.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_saver_node");
  ros::NodeHandle nh;
  ros::NodeHandle pvt_nh;

  /*Image saver*/
  dronenav_actions::PointCloudSaver saver(nh, pvt_nh);

  ros::spin();
  return 0;
}