#ifndef SENDING_HPP_
#define SENDING_HPP_

#include <active.hpp>

#include <boost/statechart/state.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/custom_reaction.hpp>

namespace global_planner
{
  struct EvPathDone : boost::statechart::event<EvPathDone> {};

  struct Sending : boost::statechart::state<Sending, Active>
  {
    public:
    Sending(my_context ctx);
    ~Sending();

    boost::statechart::result react(const EvPathDone& ev);

    private:
    void reached_callback(const dronenav_msgs::Waypoint::ConstPtr& msg);

    private:
    ros::Subscriber m_reached_sub;
    dronenav_msgs::Path m_path;
  };
}

#endif
