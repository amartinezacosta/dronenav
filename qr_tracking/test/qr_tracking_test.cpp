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
  EXPECT_TRUE(takeoff_srv.response.success);

  wait_for_state(dronenav_msgs::Status::HOVERING_STATE, 60.0);
  EXPECT_STREQ(status.state.c_str(), 
    dronenav_msgs::Status::HOVERING_STATE.c_str());

  /*Read qr code*/
  ros::Duration(10.0).sleep();

  /*Check that the QR code is in the right position*/
  EXPECT_EQ(tracked_codes.codes.size(), 1);
  EXPECT_STRCASEEQ(tracked_codes.codes[0].data.c_str(), "Waypoint 2");
  EXPECT_NEAR(tracked_codes.codes[0].x, 15.0, 1.0);
  EXPECT_NEAR(tracked_codes.codes[0].y, 14.0, 1.0);
  EXPECT_NEAR(tracked_codes.codes[0].z, 3.5, 1.0);

  /*Go to another QR code*/
  // dronenav_msgs::Waypoint wp;
  // wp.position.x = 41.0;
  // wp.position.y = 14.0;
  // wp.position.z = 16.0;
  // wp.yaw = 1.57;
  // waypoint_pub.publish(wp);
  // ros::Duration(50.0).sleep();

  /*Check that the QR code is in the right position*/

  /*Go to another QR code*/
  // wp.position.x = 32.0;
  // wp.position.y = -14.0;
  // wp.position.z = 2.0;
  // wp.yaw = -1.57;
  // waypoint_pub.publish(wp);
  // ros::Duration(50.0).sleep();

  /*Check that the QR code is in the right position*/

  /*Go to another QR code*/
  // wp.position.x = 32.0;
  // wp.position.y = -14.0;
  // wp.position.z = 26.0;
  // wp.yaw = -1.57;
  // waypoint_pub.publish(wp);
  // ros::Duration(50.0).sleep();

  /*Check that the QR code is in the right position*/

  /*Land request*/
    /*Land request*/
  dronenav_msgs::Land land_srv;
  land_srv.request.override_takeoff_pose = false;
  land_client.call(land_srv);

  EXPECT_TRUE(land_srv.response.success);

  wait_for_state(dronenav_msgs::Status::LANDED_TOUCHDOWN_STATE, 10.0);
  EXPECT_STREQ(status.state.c_str(), 
    dronenav_msgs::Status::LANDED_TOUCHDOWN_STATE.c_str());

  /*Log tracking information*/
  for(dronenav_msgs::Code& code : tracked_codes.codes)
  {
    ROS_INFO_NAMED("qr_tracking", "id=%i", code.id);
    ROS_INFO_NAMED("qr_tracking", "type=%s", code.type.c_str());
    ROS_INFO_NAMED("qr_tracking", "data=%s", code.data.c_str());
    ROS_INFO_NAMED("qr_tracking", "x=%f, y=%f, z=%f", code.x, code.y, code.z);
  }

  ros::Duration(10.0).sleep();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "qr_tracking_test");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return result;
}
