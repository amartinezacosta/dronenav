#ifndef DETECTED_HPP_
#define DETECTED_HPP_

#include <object_recognition.hpp>

#include <boost/statechart/state.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>

namespace object_recognition
{
  struct EvObjectInspected : boost::statechart::event<EvObjectInspected> {};

  struct Detected : boost::statechart::state<Detected, ObjectDetection>
  {
    public:
    Detected(my_context ctx);
    ~Detected();

    boost::statechart::result react(const EvObjectInspected& ev);
    void reached_callback(const dronenav_msgs::Waypoint::ConstPtr& msg);

    private:
    ros::Subscriber m_reached_sub;
  };
}

#endif