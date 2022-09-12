#ifndef TAKEOFF_YAW_HPP
#define TAKEOFF_YAW_HPP

#include <ros/ros.h>

#include <flying.hpp>

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>

namespace dronenav
{
  struct EvTakeoffYawingTimeout : boost::statechart::event<EvTakeoffYawingTimeout> {};

  struct TakeoffYawing : boost::statechart::state<TakeoffYawing, Flying>
  {
      public:
      TakeoffYawing(my_context ctx);
      ~TakeoffYawing();

      void tick_callback(const ros::TimerEvent &);

      boost::statechart::result react(const EvTakeoffYawingTimeout &);
      typedef boost::statechart::custom_reaction<EvTakeoffYawingTimeout> reactions;

      private:
      ros::Timer m_timer;
      double m_t;
  };
}

#endif