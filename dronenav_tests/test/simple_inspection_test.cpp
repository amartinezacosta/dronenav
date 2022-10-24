#include "dronenav_test_fixture.hpp"

TEST_F(DronenavTestFixture, simple_inspection_test)
{  
  dronenav_msgs::Takeoff takeoff_srv;
  takeoff_srv.request.x = 0.0;
  takeoff_srv.request.y = 0.0;
  takeoff_srv.request.z = 3.0;
  takeoff_srv.request.yaw = 0.0;

  takeoff_client.call(takeoff_srv);
  EXPECT_TRUE(takeoff_srv.response.success);

  wait_for_state(dronenav_msgs::Status::HOVERING_STATE, 20.0);
  EXPECT_STREQ(status.state.c_str(),
    dronenav_msgs::Status::HOVERING_STATE.c_str());

  /*Call waypoint generation*/
  dronenav_msgs::GenerateWaypoints generate_srv;
  generate_srv.request.viewport = viewport;
  generate_srv.request.min = min;
  generate_srv.request.max = max;
  generate_srv.request.downsample = downsample;

  waypoint_generation_client.call(generate_srv);
  EXPECT_GT(generate_srv.response.waypoints.size(), 0);

  /*Publish generated inspection waypoints to the drone
  and initialize mission*/
  waypoints = generate_srv.response.waypoints;
  for(dronenav_msgs::Waypoint waypoint : waypoints)
  {
    waypoint_pub.publish(waypoint);
  }

  /*Wait for queue to fill*/
  ros::Duration(5.0).sleep();

  /*Wait here for inspection to finish*/
  wait_for_queue(900);

  /*Request landing*/
  dronenav_msgs::Land land_srv;
  land_srv.request.override_takeoff_pose = false;
  land_client.call(land_srv);

  EXPECT_TRUE(land_srv.response.success);

  wait_for_state(dronenav_msgs::Status::LANDED_TOUCHDOWN_STATE, 10.0);
  EXPECT_STREQ(status.state.c_str(), 
    dronenav_msgs::Status::LANDED_TOUCHDOWN_STATE.c_str());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "simple_inspection_test");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return result;
}
