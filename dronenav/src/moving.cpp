#include <moving.hpp>
#include <yawing.hpp>

namespace dronenav
{
  Moving::Moving(my_context ctx) : my_base(ctx)
  {
      ROS_DEBUG_NAMED("dronenav", "MOVING STATE ENTRY");
      
      double t_res = context<Drone>().get_moving_tick_res();
      moving_tick = context<Drone>().m_nh.createTimer(ros::Duration(t_res), 
              &Moving::moving_tick_callback, this);

      time = 0.0;
      moving_tick.start();
  }

  Moving::~Moving()
  {
      ROS_DEBUG_NAMED("dronenav", "MOVING STATE ENTRY");
  }

  void Moving::moving_tick_callback(const ros::TimerEvent &event)
  {
      time += context<Drone>().get_moving_tick_res();
      context<Drone>().process_event(EvMotionCheckTimeout());
  }

  boost::statechart::result Moving::react(const EvMotionCheckTimeout &)
  {
    ROS_DEBUG_ONCE_NAMED("dronenav", "MOVING EvMotionCheckoutTimeout EVENT");

    //Get current drone position and waypoint
    geometry_msgs::Point current = context<Drone>().get_current_position();
    geometry_msgs::Point target = context<Drone>().get_target_position(); 

    //Distance
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double dz = target.z - current.z;
    double distance_sqr = dx*dx + dy*dy + dz*dz;

    //Is the drone within specified distance radius
    if(distance_sqr < context<Drone>().get_reach_radius())
    {
        ROS_DEBUG_NAMED("dronenav", "Waypoint reached. Drone position x = %f, y = %f, z = %f", 
                current.x, current.y, current.z);
        ROS_DEBUG_NAMED("dronenav", "Position reached in %f seconds", time);
        
        return transit<Yawing>();   
    }
    //Have we timed out?
    else if(time >= 
            context<Drone>().waypoint_timeout())
    {
        ROS_WARN_NAMED("dronenav", "Position could not be reached, time = %f", time);
        
        /*TODO: throw an warning and quit this motion, maybe even stop the drone*/
    }

    //Wait another tick event
    return discard_event();
  }
}
