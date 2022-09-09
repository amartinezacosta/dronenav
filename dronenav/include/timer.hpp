#ifndef TIMER_HPP_
#define TIMER_HPP

#include <ros/ros.h>

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>

namespace dronenav
{
  struct EvTimeout : boost::statechart::event<EvTimeout> {};

  template <typename S1, typename S2>
  struct BasePositioningState : boost::statechart::state<S1, S2>
  {
    public:
    BasePositioningState(my_context ctx) : my_base(ctx)
    {
      double t_res = context<Drone>().get_moving_tick_res();
      m_timer = context<Drone>().m_nh.createTimer(ros::Duration(t_res),
        &BasePositioningState::timer_callback, this);
      
      m_t = 0.0;
      m_timer.start();
    }

    ~BasePositioningState()
    {

    }

    virtual void on_timeout(void)
    {

    }

    void timer_callback(const ros::TimerEvent& event)
    {
      m_t += context<Drone>().get_moving_tick_res();
      context<Drone>().process_event(EvTimeout());
    }

    boost::statechart::result react(const EvTimeout& evt)
    {
      on_timeout();
    }

    public:
    typedef boost::statechart::custom_reaction<EvTimeout> reactions;

    private:
    ros::Timer m_timer;
    double m_t;
  };
}

#endif