#include <landed.hpp>

#include <takeoff_position.hpp>
#include <flying.hpp>

namespace dronenav
{
  Landed::Landed()
  {
      ROS_INFO_NAMED("dronenav", "LANDED STATE ENTRY");
  }

  Landed::~Landed()
  {
      ROS_INFO_NAMED("dronenav", "LANDED STATE ENTRY");
  }

  boost::statechart::result Landed::react(const EvTakeoff &)
  {
    ROS_INFO_NAMED("dronenav", "LANDED EvTakeoff EVENT");

    //Set target pose
    geometry_msgs::Point pos = context<Drone>().get_requested_takeoff_position();
    double yaw = context<Drone>().get_requested_takeoff_yaw();
    double speed = context<Drone>().get_cruise_speed();

    context<Drone>().set_cruise_speed(speed);

    context<Drone>().set_target_position(pos.x, pos.y, pos.z);
    context<Drone>().set_target_yaw(yaw);

    context<Drone>().arm();
    context<Drone>().offboard();

    return transit<Flying>();
  }
}