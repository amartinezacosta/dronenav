#include "dronenav_test_fixture.hpp"

TEST_F(DronenavTestFixture, image_save_test)
{
  actionlib::SimpleActionClient<
    dronenav_msgs::SaveImageAction> action_client("dronenav/save_image", true);

  /*Wait for server to start*/
  action_client.waitForServer();

  /*Done waiting, send goal*/
  dronenav_msgs::SaveImageGoal goal;
  goal.count = 3;
  goal.file_name = "dronenav_action_image_test";
  goal.delay = 3.0;
  action_client.sendGoal(goal);

  bool timeout = action_client.waitForResult(ros::Duration(30.0));
  EXPECT_TRUE(timeout);

  if(timeout)
  {
    actionlib::SimpleClientGoalState state = action_client.getState();
    EXPECT_STREQ(state.toString().c_str(), "SUCCEEDED");

    dronenav_msgs::SaveImageResultConstPtr result = action_client.getResult();
    EXPECT_EQ(goal.count, result->count);
  }
}

TEST_F(DronenavTestFixture, pointcloud_save_test)
{
  actionlib::SimpleActionClient<
    dronenav_msgs::SavePointCloudAction> action_client("dronenav/save_pointcloud", true);

  /*Wait for server to start*/
  action_client.waitForServer();

  /*Done waiting, send goal*/
  dronenav_msgs::SavePointCloudGoal goal;
  goal.count = 3;
  goal.file_name = "dronenav_action_pointcloud_test";
  goal.delay = 1.0;
  action_client.sendGoal(goal);

  bool timeout = action_client.waitForResult(ros::Duration(30.0));
  EXPECT_TRUE(timeout);

  if(timeout)
  {
    actionlib::SimpleClientGoalState state = action_client.getState();
    EXPECT_STREQ(state.toString().c_str(), "SUCCEEDED");

    dronenav_msgs::SavePointCloudResultConstPtr result = action_client.getResult();
    EXPECT_EQ(goal.count, result->count);
  }
}

TEST_F(DronenavTestFixture, video_save_test)
{
  actionlib::SimpleActionClient<
    dronenav_msgs::SaveVideoAction> action_client("dronenav/save_video", true);

  /*Wait for server to start*/
  action_client.waitForServer();

  /*Done waiting, send goal*/
  dronenav_msgs::SaveVideoGoal goal;
  goal.duration = 5.0;
  goal.file_name = "dronenav_action_video_test.avi";
  action_client.sendGoal(goal);

  bool timeout = action_client.waitForResult(ros::Duration(30.0));
  EXPECT_TRUE(timeout);

  if(timeout)
  {
    actionlib::SimpleClientGoalState state = action_client.getState();
    EXPECT_STREQ(state.toString().c_str(), "SUCCEEDED");

    dronenav_msgs::SaveVideoResultConstPtr result = action_client.getResult();
    EXPECT_NEAR(goal.duration, result->duration, 0.5);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dronenav_actions_test");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return result;
}
