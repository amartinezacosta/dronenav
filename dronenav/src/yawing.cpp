#include <yawing.hpp>
#include <reached.hpp>

namespace dronenav
{
  Yawing::Yawing(my_context ctx) : my_base(ctx)
  {
    ROS_INFO_NAMED("dronenav", "YAWING STATE ENTRY");

    double t_res = context<Drone>().get_moving_tick_res();
    yawing_tick = context<Drone>().m_nh.createTimer(ros::Duration(t_res),
        &Yawing::yawing_tick_callback, this);

    double yaw = context<Drone>().get_current_waypoint().yaw;
    context<Drone>().set_target_yaw(yaw);

    time = 0.0;
    yawing_tick.start();
  }

  Yawing::~Yawing()
  {
    ROS_INFO_NAMED("dronenav", "YAWING STATE ENTRY");
  }

  void Yawing::yawing_tick_callback(const ros::TimerEvent& evt)
  {
    time += context<Drone>().get_moving_tick_res();
    context<Drone>().process_event(EvYawingCheckTimeout());
  }

  boost::statechart::result Yawing::react(const EvYawingCheckTimeout&)
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
        ROS_INFO_NAMED("dronenav", "Yaw angle reached in %f seconds", time);
        
        //Post event to check the waypoint queue again
        post_event(EvWaypointDone());

        return transit<Reached>();
    }
    else if(time >= context<Drone>().waypoint_timeout())
    {
        ROS_WARN_NAMED("dronenav", "Yaw angle could not be reached, time = %f", time);
        //Throw error
    }

    //Wait another tick event
    return discard_event();
  }
}