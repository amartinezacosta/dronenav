#include "dronenav_test_fixture.hpp"

TEST_F(DronenavTestFixture, dronenav_actions_test)
{
  dronenav_actions::VideoSaver video_saver(nh, pvt_nh);

  dronenav_msgs::VideoSave save_srv;
  save_srv.request.duration = 10.0;
  save_srv.request.name = "video_test.avi";
  video_save_client.call(save_srv);

  EXPECT_TRUE(save_srv.response.success);

  ros::Duration(30.0).sleep();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dronenav_actions_video_test");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return result;
}
