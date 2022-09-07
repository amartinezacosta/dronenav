#ifndef INTERRUPT_HPP_
#define INTERRUPT_HPP_

#include <global_planner.hpp>

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>

namespace global_planner
{
  struct EvInterruptHandled : boost::statechart::event<EvInterruptHandled> {};

  struct Interrupt : boost::statechart::state<Interrupt, GlobalPlanner>
  {
    public:
    Interrupt(my_context ctx);
    ~Interrupt();

    boost::statechart::result react(const EvInterruptHandled& ev);

    public:
    typedef boost::statechart::custom_reaction<EvInterruptHandled> reactions;
  };
}

#endif

