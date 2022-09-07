#ifndef FLYING_HPP_
#define FLYING_HPP_

#include <navigation.hpp>
#include <landed.hpp>

namespace dronenav
{
  struct EvLand : boost::statechart::event<EvLand> {};

  struct Hovering;
  struct Flying : boost::statechart::simple_state<Flying, Navigation, Hovering>
  {
      public:
      Flying(){}
      ~Flying(){}
  
      boost::statechart::result react(const EvLand &)
      {
          //Land the drone
          context<Drone>().land();
          return transit<Landed>();
      }

      typedef boost::statechart::custom_reaction<EvLand> reactions;
  };
}

#endif
