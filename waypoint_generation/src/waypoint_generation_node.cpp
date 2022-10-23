#include <waypoint_generation.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_generation_node");
  ros::NodeHandle nh;
  ros::NodeHandle pvt_nh("~");

  waypoint_generation::WaypointGenerator generator(nh, pvt_nh);

  ros::spin();
  return 0;
}