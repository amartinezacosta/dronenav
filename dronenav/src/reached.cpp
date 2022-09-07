#include <reached.hpp>
#include <hovering.hpp>

#include <ros/ros.h>

namespace dronenav
{
  Reached::Reached()
  {
      ROS_DEBUG_NAMED("dronenav", "REACHED STATE ENTRY");
  }

  Reached::~Reached()
  {
      ROS_DEBUG_NAMED("dronenav", "REACHED STATE ENTRY");
  }

  boost::statechart::result Reached::react(const EvWaypointDone &)
  {
    ROS_DEBUG_NAMED("dronenav", "REACHED EvWaypointDone EVENT");

    //Waypoint reached, performing inspection task
    dronenav_msgs::Waypoint waypoint = context<Drone>().get_current_waypoint();

    if(waypoint.action & dronenav_msgs::Waypoint::WAYPOINT_ACTION_IMAGE)
    {
        ROS_DEBUG_NAMED("dronenav", "Image waypoint action");
        context<Drone>().save_image();
    }

    if(waypoint.action & dronenav_msgs::Waypoint::WAYPOINT_ACTION_POINTCLOUD)
    {
        ROS_DEBUG_NAMED("dronenav", "Video waypoint action");
        context<Drone>().save_pointcloud();
    }

    if(waypoint.action & dronenav_msgs::Waypoint::WAYPOINT_ACTION_VIDEO)
    {
        ROS_DEBUG_NAMED("dronenav", "Video waypoint action");
        //context<Drone>().record_video();
    }

    if(waypoint.action & dronenav_msgs::Waypoint::WAYPOINT_ACTION_NONE)
    {
        ROS_DEBUG_NAMED("dronenav", "No action waypoint");
    }
    
    //Notify users waypoint reached event
    context<Drone>().publish_waypoint_reached();

    //Dequeue waypoint from queue
    context<Drone>().dequeue();
    //Post waypoint received event to check the queue
    post_event(EvWaypointReceived());

    return transit<Hovering>();
  }
}