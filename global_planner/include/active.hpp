#ifndef ACTIVE_HPP_
#define ACTIVE_HPP_

#include <global_planner.hpp>
#include <interrupt.hpp>

#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>

namespace global_planner
{
  struct EvPathInterrupt : boost::statechart::event<EvPathInterrupt> {};

  struct Waiting;
  struct Active : boost::statechart::simple_state<Active, GlobalPlanner, Waiting>
  {
    public:
    Active()
    {
      ROS_DEBUG_NAMED("Global Planner HSM", 
        "ACTIVE STATE ENTRY");
    }

    ~Active()
    {
      ROS_DEBUG_NAMED("Global Planner HSM", 
        "ACTIVE STATE EXIT");
    }

    boost::statechart::result react(const EvPathInterrupt& ev)
    {
      ROS_DEBUG_NAMED("Global Planner HSM", 
        "ACTIVE STATE EvPathInterrupt EVENT");
      return transit<Interrupt>();
    }

    public:
    typedef boost::statechart::custom_reaction<EvPathInterrupt> reactions;

  };
}

#endif