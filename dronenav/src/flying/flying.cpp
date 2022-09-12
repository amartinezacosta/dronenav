#include <flying.hpp>
#include <landed_position.hpp>

namespace dronenav
{
  Flying::Flying()
  {

  }

  Flying::~Flying()
  {

  }

  boost::statechart::result Flying::react(const EvLand& evt)
  {
    //Set target pose
    geometry_msgs::Point pos = context<Drone>().get_requested_takeoff_position();
    double yaw = context<Drone>().get_requested_takeoff_yaw();

    /*Position drone*/
    context<Drone>().set_target_position(pos.x, pos.y, pos.z);

    /*Orient drone*/
    context<Drone>().set_target_yaw(yaw);

    /*TODO: use downwards facing camera for precision landing*/

    //context<Drone>().land();
    return transit<LandedPositioning>();
  }
}