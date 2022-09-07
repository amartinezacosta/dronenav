#ifndef MOVING_HPP_
#define MOVING_HPP_

#include <ros/ros.h>

#include <flying.hpp>

#include <boost/statechart/state.hpp>

namespace dronenav
{
  struct EvMotionCheckTimeout : boost::statechart::event<EvMotionCheckTimeout> {};

  struct Moving : boost::statechart::state<Moving, Flying>
  {
      public:
      Moving(my_context ctx);
      ~Moving();

      void moving_tick_callback(const ros::TimerEvent &);

      boost::statechart::result react(const EvMotionCheckTimeout &);
      typedef boost::statechart::custom_reaction<EvMotionCheckTimeout> reactions;

      private:
      ros::Timer moving_tick;
      double time;
  };
}

#endif
