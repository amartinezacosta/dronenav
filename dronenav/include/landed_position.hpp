#ifndef LANDED_POSITION_HPP
#define LANDED_POSITION_HPP

#include <ros/ros.h>

#include <landed.hpp>

#include <boost/statechart/state.hpp>

namespace dronenav
{
  struct EvLandedPositioningTimeout : boost::statechart::event<EvLandedPositioningTimeout> {};

  struct LandedPositioning : boost::statechart::state<LandedPositioning, Landed>
  {
      public:
      LandedPositioning(my_context ctx);
      ~LandedPositioning();

      void tick_callback(const ros::TimerEvent &);

      boost::statechart::result react(const EvLandedPositioningTimeout &);
      typedef boost::statechart::custom_reaction<EvLandedPositioningTimeout> reactions;

      private:
      ros::Timer m_timer;
      double m_t;
  };
}

#endif