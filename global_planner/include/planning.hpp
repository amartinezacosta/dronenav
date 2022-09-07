#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include <active.hpp>

#include <boost/statechart/state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>

namespace global_planner
{
  struct EvPathFound : boost::statechart::event<EvPathFound> {};
  struct EvPathNotFound : boost::statechart::event<EvPathNotFound> {};

  struct Planning : boost::statechart::state<Planning, Active>
  {
    public:
    Planning(my_context ctx);
    ~Planning();

    boost::statechart::result react(const EvPathFound& ev);
    boost::statechart::result react(const EvPathNotFound& ev);

    typedef boost::mpl::list<
      boost::statechart::custom_reaction<EvPathFound>,
      boost::statechart::custom_reaction<EvPathNotFound> > reactions;
  };
}

#endif