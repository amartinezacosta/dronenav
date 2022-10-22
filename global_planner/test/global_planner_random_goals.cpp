#include "dronenav_test_fixture.hpp"

TEST_F(DronenavTestFixture, global_planner_random_goals)
{
  /*Main experiment here------------------------------------------------------*/
  ros::NodeHandle pvt_nh("~");
  global_planner::GlobalPlanner planner(nh, pvt_nh);
  dronenav_msgs::GlobalGoal goal;
  geometry_msgs::Point start;

  ros::Duration(3.0).sleep();

  for(int i = 0; i < GOAL_SAMPLE_COUNT; i++)
  {
    start.x = random_coordinate(-22.0, 28.0);
    start.y = random_coordinate(-15.0, 15.0);
    start.z = random_coordinate(3.0, 27.0);

    goal.x = random_coordinate(-22.0, 28.0);
    goal.y = random_coordinate(-15.0, 15.0);
    goal.z = random_coordinate(3.0, 27.0);
    goal.priority = 0;
    goal.yaw = 0.0;
  
    planner.set_start(start);
    planner.find_global_path(goal);
  }

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
