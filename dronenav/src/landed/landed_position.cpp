#include <landed_position.hpp>
#include <landed_yaw.hpp>

namespace dronenav
{
  LandedPositioning::LandedPositioning(my_context ctx) : my_base(ctx)
  {
    ROS_INFO_NAMED("dronenav", "LANDED_POSITIONING STATE ENTRY");
    
    context<Drone>().set_state("LANDED_POSITIONING");

    double t_res = context<Drone>().get_moving_tick_res();
    m_timer = context<Drone>().m_nh.createTimer(ros::Duration(t_res), 
            &LandedPositioning::tick_callback, this);

    m_t = 0.0;
    m_timer.start();
  }

  LandedPositioning::~LandedPositioning()
  {
    ROS_INFO_NAMED("dronenav", "LANDED_POSITIONING STATE ENTRY");
  }

  void LandedPositioning::tick_callback(const ros::TimerEvent &event)
  {
      m_t += context<Drone>().get_moving_tick_res();
      context<Drone>().process_event(EvLandedPositioningTimeout());
  }

  boost::statechart::result LandedPositioning::react(const EvLandedPositioningTimeout& evt)
  {
    ROS_INFO_ONCE_NAMED("dronenav", "LANDED_POSITIONING EvLandedPositioningTimeout EVENT");

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
        ROS_INFO_NAMED("dronenav", "Waypoint reached. Drone position x = %f, y = %f, z = %f", 
                current.x, current.y, current.z);
        ROS_INFO_NAMED("dronenav", "Position reached in %f seconds", m_t);
        
        return transit<LandedYawing>();   
    }
    //Have we timed out?
    else if(m_t >= context<Drone>().waypoint_timeout())
    {
        ROS_WARN_NAMED("dronenav", "Position could not be reached, time = %f", m_t);
        
        /*TODO: throw an warning and quit this motion, maybe even stop the drone*/
    }

    //Wait another tick event
    return discard_event();
  }
}
