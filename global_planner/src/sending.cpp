#include <sending.hpp>
#include <waiting.hpp>

namespace global_planner
{

  Sending::Sending(my_context ctx) : my_base(ctx)
  {
    ROS_DEBUG_NAMED("Global Planner HSM", 
      "SENDING STATE ENTRY");

    m_reached_sub = context<GlobalPlanner>().m_nh.subscribe<
      dronenav_msgs::Waypoint>("skipper/waypoint/reached", 50,
      &Sending::reached_callback, this);
  }

  Sending::~Sending()
  {
    ROS_DEBUG_NAMED("Global Planner HSM", 
      "SENDING STATE EXIT");
  }

  void Sending::reached_callback(const dronenav_msgs::Waypoint::ConstPtr& msg)
  {
    if(msg->flags & dronenav_msgs::Waypoint::PATH_LAST_FLAG)
    {
      context<GlobalPlanner>().process_event(EvPathDone());
    }
  }

  boost::statechart::result Sending::react(const EvPathDone& ev)
  {
    ROS_DEBUG_NAMED("Global Planner HSM", 
      "SENDING STATE EvPathDone EVENT");
    
    context<GlobalPlanner>().dequeue_goal();
    post_event(EvGoalReceived());

    return transit<Waiting>();
  }
}
