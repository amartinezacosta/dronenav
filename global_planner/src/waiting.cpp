#include <waiting.hpp>
#include <planning.hpp>

namespace global_planner
{
  Waiting::Waiting()
  {
    ROS_INFO_NAMED("global_planner", "WAITING STATE ENTRY");
  }

  Waiting::~Waiting()
  {
    ROS_INFO_NAMED("global_planner", "WAITING STATE EXIT");
  }

  boost::statechart::result Waiting::react(const EvGoalReceived& ev)
  {
    ROS_INFO_NAMED("global_planner", "WAITING EvGoalReceived EVENT");
    
    if(!context<GlobalPlanner>().goal_queue_empty())
    {
      context<GlobalPlanner>().next_goal();
      return transit<Planning>();
    }
    else
    {
      return discard_event();
    }
  }
}
