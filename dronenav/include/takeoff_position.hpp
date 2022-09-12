#ifndef TAKEOFF_POSITION_HPP
#define TAKEOFF_POSITION_HPP

#include <ros/ros.h>

#include <flying.hpp>

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>

namespace dronenav
{
  struct EvTakeoffPositioningTimeout : boost::statechart::event<EvTakeoffPositioningTimeout> {};

  struct TakeoffPositioning : boost::statechart::state<TakeoffPositioning, Flying>
  {
      public:
      TakeoffPositioning(my_context ctx);
      ~TakeoffPositioning();

      void tick_callback(const ros::TimerEvent &);

      boost::statechart::result react(const EvTakeoffPositioningTimeout &);
      typedef boost::statechart::custom_reaction<EvTakeoffPositioningTimeout> reactions;

      private:
      ros::Timer m_timer;
      double m_t;
  };
}

#endif