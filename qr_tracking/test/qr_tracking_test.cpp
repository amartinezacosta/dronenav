#include "dronenav_test_fixture.hpp"

TEST_F(DronenavTestFixture, qr_tracking_test)
{
  qr_tracking::QRTracking qr_tracker(nh, pvt_nh);

  dronenav_msgs::Takeoff takeoff_srv;
  takeoff_srv.request.x = 15.0;
  takeoff_srv.request.y = 14.0;
  takeoff_srv.request.z = 3.5;
  takeoff_srv.request.yaw = 1.57;

  takeoff_client.call(takeoff_srv);
  ASSERT_TRUE(takeoff_srv.response.success);

  wait_for_state(dronenav_msgs::Status::HOVERING_STATE, 30.0);
  EXPECT_STREQ(status.state.c_str(), 
    dronenav_msgs::Status::HOVERING_STATE.c_str());

  /*Wait here until we read a qr code*/
  ros::Duration(20.0).sleep();
  //EXPECT_STREQ(qr_tracker.get_message(), "WAYPOINT 2");

  /*Land request*/
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
