#ifndef LANDED_YAW_HPP
#define LANDED_YAW_HPP

#include <ros/ros.h>

#include <landed.hpp>

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>

namespace dronenav
{
  struct EvLandedYawingTimeout : boost::statechart::event<EvLandedYawingTimeout> {};

  struct LandedYawing : boost::statechart::state<LandedYawing, Landed>
  {
      public:
      LandedYawing(my_context ctx);
      ~LandedYawing();

      void tick_callback(const ros::TimerEvent &);

      boost::statechart::result react(const EvLandedYawingTimeout &);
      typedef boost::statechart::custom_reaction<EvLandedYawingTimeout> reactions;

      private:
      ros::Timer m_timer;
      double m_t;
  };
}

#endif