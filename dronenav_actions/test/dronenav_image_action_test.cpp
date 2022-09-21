#include "dronenav_test_fixture.hpp"

TEST_F(DronenavTestFixture, dronenav_actions_test)
{
  dronenav_actions::ImageSaver image_saver(nh, pvt_nh);

  dronenav_msgs::ImageSave save_srv;
  save_srv.request.count = 10;
  save_srv.request.name = "image_test";
  image_save_client.call(save_srv);

  EXPECT_TRUE(save_srv.response.success);

  ros::Duration(30.0).sleep();
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
