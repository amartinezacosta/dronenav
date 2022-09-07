#ifndef LANDED_HPP_
#define LANDED_HPP_

#include <navigation.hpp>

namespace dronenav
{
  struct EvTakeoff : boost::statechart::event<EvTakeoff> {};

  struct Landed : boost::statechart::simple_state<Landed, Navigation>
  {
      public:
      Landed();
      ~Landed();

      boost::statechart::result react(const EvTakeoff &);
      typedef boost::statechart::custom_reaction<EvTakeoff> reactions;
  };
}

#endif