#include "dronenav_test_fixture.hpp"

TEST_F(DronenavTestFixture, waypoint_back_wall)
{  
  ros::NodeHandle pvt_nh("~");
  waypoint_generation::WaypointGenerator generator(nh, pvt_nh);

  ros::Duration(3.0).sleep();

  std::vector<dronenav_msgs::Waypoint> waypoints[NFAC_SECTION_0_SEGMENTATIONS];
  for(int i = 0; i < NFAC_SECTION_0_SEGMENTATIONS; i++)
  {
    ros::WallTime start, end;
    start = ros::WallTime::now();

    generator.generate_waypoints(waypoints[i], 
      min[i], 
      max[i], 
      viewports[i],
      downsamples[i]);

    end = ros::WallTime::now();
    double execution_time = (end - start).toNSec() * 1e-6;

    /*Write results to a file*/
    std::stringstream file_name;
    file_name << "nfac_wall_waypoints_" << i << ".csv";
    
    std::ofstream file;
    file.open(file_name.str());

    file << "x,y,z\n"; 
    for(dronenav_msgs::Waypoint waypoint : waypoints[i])
    {
      file << waypoint.position.x << "," 
           << waypoint.position.y << ","
           << waypoint.position.z << std::endl;
    }

    file.close();
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
