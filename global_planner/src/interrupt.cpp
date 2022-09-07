#include <interrupt.hpp>
#include <waiting.hpp>

namespace global_planner
{
  Interrupt::Interrupt(my_context ctx) : my_base(ctx)
  {
    dronenav_msgs::PathGoal new_goal = context<GlobalPlanner>().get_new_goal();
    context<GlobalPlanner>().enqueue_front(new_goal);

    post_event(EvInterruptHandled());
    post_event(EvGoalReceived());
  }
  
  Interrupt::~Interrupt()
  {

  }

  boost::statechart::result Interrupt::react(const EvInterruptHandled& ev)
  {
    return transit<Active>();
  }
}
