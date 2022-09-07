#include <waiting.hpp>
#include <planning.hpp>

namespace global_planner
{
  Waiting::Waiting()
  {
    ROS_DEBUG_NAMED("Global Planner HSM", 
      "WAITING STATE ENTRY");
  }

  Waiting::~Waiting()
  {
    ROS_DEBUG_NAMED("Global Planner HSM", 
      "WAITING STATE EXIT");
  }

  boost::statechart::result Waiting::react(const EvGoalReceived& ev)
  {
    ROS_DEBUG_NAMED("Global Planner HSM", 
      "WAITING STATE EvGoalReceived EVENT");
    
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
