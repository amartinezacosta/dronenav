#include <takeoff_yaw.hpp>
#include <hovering.hpp>

namespace dronenav
{
  TakeoffYawing::TakeoffYawing(my_context ctx) : my_base(ctx)
  {
    ROS_INFO_NAMED("dronenav", "YAWING STATE ENTRY");

    context<Drone>().set_state("TAKEOFF_YAWING");

    double t_res = context<Drone>().get_moving_tick_res();
    m_timer = context<Drone>().m_nh.createTimer(ros::Duration(t_res),
        &TakeoffYawing::tick_callback, this);

    double yaw = context<Drone>().get_current_waypoint().yaw;
    context<Drone>().set_target_yaw(yaw);

    m_t = 0.0;
    m_timer.start();
  }

  TakeoffYawing::~TakeoffYawing()
  {
    ROS_INFO_NAMED("dronenav", "YAWING STATE ENTRY");
  }

  void TakeoffYawing::tick_callback(const ros::TimerEvent& evt)
  {
    m_t += context<Drone>().get_moving_tick_res();
    context<Drone>().process_event(EvTakeoffYawingTimeout());
  }

  boost::statechart::result TakeoffYawing::react(const EvTakeoffYawingTimeout&)
  {
    ROS_INFO_ONCE_NAMED("dronenav", "YAWING EvMotionCheckoutTimeout EVENT");

    //TODO: Compare also yaw angle here
    double target = context<Drone>().get_target_yaw();
    double current = context<Drone>().get_current_yaw();

    double error = fabs(target - current);

    if(error < context<Drone>().get_yaw_min_error())
    {
        ROS_INFO_NAMED("dronenav", "Yaw angle reached. Drone yaw = %f", 
            (current*180.0)/M_PI);
        ROS_INFO_NAMED("dronenav", "Yaw angle reached in %f seconds", m_t);
        
        return transit<Hovering>();
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