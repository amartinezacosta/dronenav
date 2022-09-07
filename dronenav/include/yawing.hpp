#ifndef YAWING_HPP_
#define YAWING_HPP_

#include <ros/ros.h>

#include <flying.hpp>

#include <boost/statechart/state.hpp>

namespace dronenav
{
  struct EvYawingCheckTimeout : boost::statechart::event<EvYawingCheckTimeout> {};
  struct EvWaypointDone : boost::statechart::event<EvWaypointDone> {};

  struct Yawing : boost::statechart::state<Yawing, Flying>
  {
      public:
      Yawing(my_context ctx);
      ~Yawing();

      void yawing_tick_callback(const ros::TimerEvent&);

      boost::statechart::result react(const EvYawingCheckTimeout&);
      typedef boost::statechart::custom_reaction<EvYawingCheckTimeout> reactions;

      private:
      ros::Timer yawing_tick;
      double time;
  };
}

#endif