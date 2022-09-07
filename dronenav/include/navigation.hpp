#ifndef NAVIGATION_HPP_
#define NAVIGATION_HPP_

#include <dronenav.hpp>

#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>

namespace dronenav
{
  struct Landed;
  struct Navigation : boost::statechart::simple_state<Navigation, Drone, Landed>
  {
      public:
      Navigation(){}
      ~Navigation(){}
  };
}

#endif