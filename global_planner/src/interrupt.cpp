#include <interrupt.hpp>
#include <waiting.hpp>

namespace global_planner
{
  Interrupt::Interrupt(my_context ctx) : my_base(ctx)
  {
    ROS_INFO_NAMED("global_planner", "INTERRUPT STATE ENTRY");
    
    dronenav_msgs::GlobalGoal new_goal = context<GlobalPlanner>().get_new_goal();
    context<GlobalPlanner>().enqueue_front(new_goal);

    post_event(EvInterruptHandled());
    post_event(EvGoalReceived());
  }
  
  Interrupt::~Interrupt()
  {
    ROS_INFO_NAMED("global_planner", "INTERRUPT STATE EXIT");
  }

  boost::statechart::result Interrupt::react(const EvInterruptHandled& ev)
  {
    ROS_INFO_NAMED("global_planner", "INTERRUPT EvInterruptHandled EVENT");

    return transit<Active>();
  }
}
