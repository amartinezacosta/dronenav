#ifndef POSITIONING_HPP_
#define POSITIONING_HPP_

#include <ros/ros.h>

#include <flying.hpp>

#include <boost/statechart/state.hpp>

namespace dronenav
{
  struct EvPoisitioningTimeout : boost::statechart::event<EvPoisitioningTimeout> {};

  struct Positioning : boost::statechart::state<Positioning, Flying>
  {
      public:
      Positioning(my_context ctx);
      ~Positioning();

      void tick_callback(const ros::TimerEvent &);

      boost::statechart::result react(const EvPoisitioningTimeout &);
      typedef boost::statechart::custom_reaction<EvPoisitioningTimeout> reactions;

      private:
      ros::Timer m_timer;
      double m_t;
  };
}

#endif
