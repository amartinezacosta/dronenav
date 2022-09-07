#ifndef REACHED_HPP_
#define REACHED_HPP_

#include <flying.hpp>
#include <yawing.hpp>

#include <std_msgs/Empty.h>

namespace dronenav
{
  struct Reached : boost::statechart::simple_state<Reached, Flying>
  {
      public:
      Reached();
      ~Reached();

      boost::statechart::result react(const EvWaypointDone &);
      typedef boost::statechart::custom_reaction<EvWaypointDone> reactions;
  };
}

#endif
