#ifndef HOVERING_HPP_
#define HOVERING_HPP_

#include <flying.hpp>

namespace dronenav
{
  struct EvWaypointReceived : boost::statechart::event<EvWaypointReceived> {};

  struct Hovering : boost::statechart::state<Hovering, Flying>
  {
      public:
      Hovering(my_context ctx);
      ~Hovering();

      boost::statechart::result react(const EvWaypointReceived &);
      typedef boost::statechart::custom_reaction<EvWaypointReceived> reactions;
  };
}

#endif
