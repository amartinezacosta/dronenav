#ifndef WAITING_HPP_
#define WAITING_HPP_

#include <active.hpp>

#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/event.hpp>

namespace global_planner
{
  struct EvGoalReceived : boost::statechart::event<EvGoalReceived> {};

  struct Waiting : boost::statechart::simple_state<Waiting, Active>
  {
    public:
    Waiting();
    ~Waiting();

    boost::statechart::result react(const EvGoalReceived& ev);

    public:
    typedef boost::statechart::custom_reaction<EvGoalReceived> reactions;
  };
}

#endif
