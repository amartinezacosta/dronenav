#include "dronenav_test_fixture.hpp"

#include <dronenav.hpp>
#include <navigation.hpp>
#include <landed_touchdown.hpp>
#include <landed_position.hpp>
#include <landed_yaw.hpp>
#include <landed.hpp>

#include <takeoff_position.hpp>
#include <takeoff_yaw.hpp>
#include <hovering.hpp>
#include <positioning.hpp>
#include <reached.hpp>
#include <flying.hpp>

TEST_F(DronenavTestFixture, dronenav_test)
{
  dronenav::Drone drone(nh, pvt_nh);
  drone.initiate();
  drone.connect();

  int tries;

  /*Takeoff request*/
  dronenav_msgs::Takeoff takeoff_srv;
  takeoff_srv.request.x = 0.0;
  takeoff_srv.request.y = 0.0;
  takeoff_srv.request.z = 3.0;
  takeoff_srv.request.yaw = 0.0;

  takeoff_client.call(takeoff_srv);
  ros::spinOnce();

  ASSERT_TRUE(takeoff_srv.response.success);

  /*Wait until we have take off and in position*/
  wait_for_state(dronenav_msgs::Status::HOVERING_STATE, 10.0);
  EXPECT_STREQ(status.state.c_str(), 
    dronenav_msgs::Status::HOVERING_STATE.c_str());

  /*Send a few waypoints*/
  waypoint_pub.publish(waypoint[0]);
  ros::Duration(10.0).sleep();

  EXPECT_NEAR(status.current_position.x, waypoint[0].position.x, 0.1);
  EXPECT_NEAR(status.current_position.y, waypoint[0].position.y, 0.1);
  EXPECT_NEAR(status.current_position.z, waypoint[0].position.z, 0.1);
  EXPECT_NEAR(status.current_yaw, waypoint[0].yaw, 0.1);

  /*Land request*/
  dronenav_msgs::Land land_srv;
  land_srv.request.override_takeoff_pose = false;
  land_client.call(land_srv);
  ros::spinOnce();

  ASSERT_TRUE(land_srv.response.success);

  wait_for_state(dronenav_msgs::Status::LANDED_TOUCHDOWN_STATE, 10.0);
  EXPECT_STREQ(status.state.c_str(), 
    dronenav_msgs::Status::LANDED_TOUCHDOWN_STATE.c_str());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dronenav_test");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return result;
}
