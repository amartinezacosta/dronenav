#ifndef LANDED_TOUCHDOWN_HPP
#define LANDED_TOUCHDOWN_HPP

#include <landed.hpp>
#include <flying.hpp>

#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>

namespace dronenav
{
  struct LandedTouchdown : boost::statechart::state<LandedTouchdown, Landed>
  {
    public:
    LandedTouchdown(my_context ctx);
    ~LandedTouchdown();

    //typedef boost::statechart::transition<EvTakeoff, Flying> reactions;
  };
}

#endif