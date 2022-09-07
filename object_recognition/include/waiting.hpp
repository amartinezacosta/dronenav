#ifndef WAITING_HPP_
#define WAITING_HPP_

#include <object_recognition.hpp>

#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/custom_reaction.hpp>

namespace object_recognition
{
  struct EvObjectDetected : boost::statechart::event<EvObjectDetected> {};
  
  struct Waiting : boost::statechart::simple_state<Waiting, ObjectDetection>
  {
    public:
    Waiting()
    {
      ROS_DEBUG_NAMED("Object Recognition HSM", 
        "WAITING STATE ENTRY");
    }

    ~Waiting()
    {
      ROS_DEBUG_NAMED("Object Recognition HSM", 
        "WAITING STATE EXIT");
    }

    boost::statechart::result react(const EvObjectDetected& ev)
    {
      return discard_event();
    }

    public:
    typedef boost::statechart::custom_reaction<EvObjectDetected> reactions;
  };
}


#endif