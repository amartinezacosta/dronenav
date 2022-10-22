#include <sending.hpp>
#include <waiting.hpp>

namespace global_planner
{

  Sending::Sending(my_context ctx) : my_base(ctx)
  {
    ROS_INFO_NAMED("global_planner", "SENDING STATE ENTRY");

    m_reached_sub = context<GlobalPlanner>().m_nh.subscribe<
      dronenav_msgs::Waypoint>("dronenav/waypoint/reached", 50,
      &Sending::reached_callback, this);

    /*Send path for navigation*/
    context<GlobalPlanner>().send_path();
  }

  Sending::~Sending()
  {
    ROS_INFO_NAMED("global_planner", "SENDING STATE EXIT"); 
  }

  void Sending::reached_callback(const dronenav_msgs::Waypoint::ConstPtr& msg)
  {
    if(msg->flags & dronenav_msgs::Waypoint::PATH_LAST_FLAG)
    {
      ROS_INFO_NAMED("global_planner", "Last flag path received");
      context<GlobalPlanner>().process_event(EvPathDone());
    }
  }

  boost::statechart::result Sending::react(const EvPathDone& ev)
  {
    ROS_INFO_NAMED("global_planner", "SENDING EvPathDone EVENT");
    
    context<GlobalPlanner>().dequeue_goal();
    post_event(EvGoalReceived());

    return transit<Waiting>();
  }
}
