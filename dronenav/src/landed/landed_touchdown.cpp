#include <landed_touchdown.hpp>

namespace dronenav
{
  LandedTouchdown::LandedTouchdown(my_context ctx) : my_base(ctx)
  {
    ROS_INFO_NAMED("dronenav", "LANDED_TOUCHDOWN STATE ENTRY");

    context<Drone>().set_state("LANDED_TOUCHDOWN");

    context<Drone>().land();
  }

  LandedTouchdown::~LandedTouchdown()
  {
    ROS_INFO_NAMED("dronenav", "LANDED_TOUCHDOWN STATE EXIT");
  }
}