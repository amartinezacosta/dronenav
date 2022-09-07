#include <hovering.hpp>
#include <moving.hpp>

#include <ros/ros.h>

namespace dronenav
{
  Hovering::Hovering()
  {
    ROS_DEBUG_NAMED("dronenav", "HOVERING STATE ENTRY");
  }

  Hovering::~Hovering()
  {
    ROS_DEBUG_NAMED("dronenav", "HOVERING STATE EXIT");
  }

  boost::statechart::result Hovering::react(const EvWaypointReceived &)
  {
    ROS_DEBUG_NAMED("dronenav", "HOVERING EvWaypointReceived EVENT");

    //Get next waypoint on the queue
    if(context<Drone>().next())
    {
        //Set target to next waypoint
        dronenav_msgs::Waypoint waypoint = context<Drone>().get_current_waypoint();
        //First set the position, once reached, then set the yaw angle
        context<Drone>().set_target_position(waypoint.position);

        return transit<Moving>();
    }
    else
    {
        //No waypoints on the queue. Stay in this state
        return discard_event();
    }
  }
}