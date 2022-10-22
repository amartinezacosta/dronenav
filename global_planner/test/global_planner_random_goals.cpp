#include "dronenav_test_fixture.hpp"

TEST_F(DronenavTestFixture, global_planner_random_goals)
{
  /*Takeoff*/
  dronenav_msgs::Takeoff takeoff_srv;
  takeoff_srv.request.x = 0.0;
  takeoff_srv.request.y = 0.0;
  takeoff_srv.request.z = 3.0;
  takeoff_srv.request.yaw = 0.0;

  takeoff_client.call(takeoff_srv);
  EXPECT_TRUE(takeoff_srv.response.success);

  wait_for_state(dronenav_msgs::Status::HOVERING_STATE, 60.0);
  EXPECT_STREQ(status.state.c_str(), 
    dronenav_msgs::Status::HOVERING_STATE.c_str());

  ros::Duration(3.0).sleep();

  /*Main experiment here---------------------------------------------------------*/
  dronenav_msgs::GlobalGoal goal;
  dronenav_msgs::Waypoint waypoint;

  for(int i = 0; i < GOAL_SAMPLE_COUNT; i++)
  {
    goal.x = random_coordinate(-28.0, 28.0);
    goal.y = random_coordinate(-15.0, 15.0);
    goal.z = random_coordinate(3.0, 27.0);
    goal.priority = 0;
    goal.yaw = 0.0;
  
    global_goal_pub.publish(goal);
  }

  ros::Duration(5.0).sleep();

  /*Wait for drone to navigate to all paths*/
  while(global_planner_status.queue_size) ros::Duration(1.0).sleep();

  /*Land request*/
  dronenav_msgs::Land land_srv;
  land_srv.request.override_takeoff_pose = false;
  land_client.call(land_srv);

  EXPECT_TRUE(land_srv.response.success);

  wait_for_state(dronenav_msgs::Status::LANDED_TOUCHDOWN_STATE, 10.0);
  EXPECT_STREQ(status.state.c_str(), 
    dronenav_msgs::Status::LANDED_TOUCHDOWN_STATE.c_str());

  ros::Duration(10.0).sleep();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "global_planner_random_goals");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();

  return result;
}
