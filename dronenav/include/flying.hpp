#ifndef FLYING_HPP_
#define FLYING_HPP_

#include <navigation.hpp>
#include <landed.hpp>

#include <boost/statechart/state.hpp>

namespace dronenav
{
  struct EvLand : boost::statechart::event<EvLand> {};

  struct TakeoffPositioning;
  struct Flying : boost::statechart::simple_state<Flying, Navigation, TakeoffPositioning>
  {
      public:
      Flying();
      ~Flying();
  
      boost::statechart::result react(const EvLand &);

      public:
      typedef boost::statechart::custom_reaction<EvLand> reactions;
  };
}

#endif
