#include <ros/ros.h>

#include <landed.hpp>
#include <flying.hpp>
#include <hovering.hpp>

namespace dronenav
{
  Landed::Landed()
  {
      ROS_DEBUG_NAMED("dronenav", "LANDED STATE ENTRY");
  }

  Landed::~Landed()
  {
      ROS_DEBUG_NAMED("dronenav", "LANDED STATE ENTRY");
  }

  boost::statechart::result Landed::react(const EvTakeoff &)
  {
    ROS_DEBUG_NAMED("dronenav", "LANDED EvTakeoff EVENT");

    //Set target pose
    double height = context<Drone>().get_requested_height();
    double speed = context<Drone>().get_cruise_speed();

    context<Drone>().set_cruise_speed(speed);

    //TODO: Initial x y and yaw could be used as well
    context<Drone>().set_target_position(0.0, 0.0, height);
    context<Drone>().set_target_yaw(0.0);

    context<Drone>().arm();
    context<Drone>().offboard();

    return transit<Flying>();
  }
}