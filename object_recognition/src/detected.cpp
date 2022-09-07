#include <detected.hpp>

namespace object_recognition
{
  Detected::Detected(my_context ctx) : my_base(ctx)
  {
    ROS_DEBUG_NAMED("Object Recognition HSM", 
      "DETECTED STATE ENTRY");

    Object target = context<ObjectDetection>().m_target_object;
    ROS_DEBUG_NAMED("Object Recognition HSM",
      "Object %s detected", target.name);
    
    /*Has this object seen before?*/

    /*Create waypoints around the object*/

    /*Subscribe to waypoint reached topic*/
    m_reached_sub = context<ObjectDetection>().m_nh.subscribe<
      dronenav_msgs::Waypoint>("dronenav/waypoints/reached", 50,
      &Detected::reached_callback, this);
  }

  Detected::~Detected()
  { 
    ROS_DEBUG_NAMED("Object Recognition HSM", 
      "DETECTED STATE EXIT");
  }

  void Detected::reached_callback(const dronenav_msgs::Waypoint::ConstPtr& msg)
  {
    if(msg->flags & dronenav_msgs::Waypoint::PATH_LAST_FLAG)
    {
      context<ObjectDetection>().process_event(EvObjectInspected());
    }
  }
}