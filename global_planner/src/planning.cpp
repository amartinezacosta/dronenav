#include <planning.hpp>
#include <global_planner.hpp>
#include <waiting.hpp>
#include <sending.hpp>

namespace global_planner
{
  Planning::Planning(my_context ctx) : my_base(ctx)
  {
    ROS_INFO_NAMED("global_planner", "PLANNING STATE ENTRY");

    if(context<GlobalPlanner>().find_global_path())
    {
      post_event(EvPathFound());
    }
    else
    {
      // context<GlobalPlanner>().set_path_found(false);
      dronenav_msgs::GlobalGoal& goal = context<GlobalPlanner>().get_current_goal();

      ROS_WARN_NAMED("global_planner", "No path to x=%f, y=%f, z=%f was found",
        goal.x, goal.y, goal.z);

      context<GlobalPlanner>().dequeue_goal();

      post_event(EvPathNotFound());
      post_event(EvGoalReceived());
    }
  }

  Planning::~Planning()
  {
    ROS_INFO_NAMED("global_planner", "PLANNING STATE EXIT");
  }

  boost::statechart::result Planning::react(const EvPathFound& ev)
  {
    ROS_INFO_NAMED("global_planner", "PLANNING EvPathFound EVENT");

    return transit<Sending>();
  }

  boost::statechart::result Planning::react(const EvPathNotFound& ev)
  {
    ROS_INFO_NAMED("global_planner", "PLANNING EvPathNotFound EVENT");

    return transit<Waiting>();
  }
}