#include <landed_yaw.hpp>
#include <landed_touchdown.hpp>

namespace dronenav
{
  LandedYawing::LandedYawing(my_context ctx) : my_base(ctx)
  {
    ROS_INFO_NAMED("dronenav", "LANDED_YAWING STATE ENTRY");

    context<Drone>().set_state("LANDED_YAWING");

    double t_res = context<Drone>().get_moving_tick_res();
    m_timer = context<Drone>().m_nh.createTimer(ros::Duration(t_res),
        &LandedYawing::tick_callback, this);

    double yaw = context<Drone>().get_current_waypoint().yaw;
    context<Drone>().set_target_yaw(yaw);

    m_t = 0.0;
    m_timer.start();
  }

  LandedYawing::~LandedYawing()
  {
    ROS_INFO_NAMED("dronenav", "LANDED_YAWING STATE ENTRY");
  }

  void LandedYawing::tick_callback(const ros::TimerEvent& evt)
  {
    m_t += context<Drone>().get_moving_tick_res();
    context<Drone>().process_event(EvLandedYawingTimeout());
  }

  boost::statechart::result LandedYawing::react(const EvLandedYawingTimeout&)
  {
    ROS_INFO_ONCE_NAMED("dronenav", "LANDED_YAWING EvMotionCheckoutTimeout EVENT");

    //TODO: Compare also yaw angle here
    double target = context<Drone>().get_target_yaw();
    double current = context<Drone>().get_current_yaw();

    double error = fabs(target - current);

    if(error < context<Drone>().get_yaw_min_error())
    {
        ROS_INFO_NAMED("dronenav", "Land yaw angle %f reached in %f s", 
            (current*180.0)/M_PI, m_t);
      
        return transit<LandedTouchdown>();
    }
    else if(m_t >= context<Drone>().waypoint_timeout())
    {
        ROS_WARN_NAMED("dronenav", "Yaw angle could not be reached, time = %f", m_t);
        //Throw error
    }

    //Wait another tick event
    return discard_event();
  }
}