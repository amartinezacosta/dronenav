#ifndef WAYPOINT_GENERATION_HPP_
#define WAYPOINT_GENERATION_HPP_

#include <ros/ros.h>

//#include <dronenav/WaypointGenerate.h>
#include <dronenav_msgs/GlobalGoal.h>
#include <dronenav_msgs/Waypoint.h>

#include <visualization_msgs/Marker.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

namespace waypoint_generation
{
  enum WaypointRegion
  {
    X_POSITIVE,
    X_NEGATIVE,
    Y_POSITIVE,
    Y_NEGATIVE
  };

  class WaypointGenerator
  {
    public:
    WaypointGenerator(ros::NodeHandle nh, 
      ros::NodeHandle pvt_nh);

    void generate_waypoints(
      std::vector<dronenav_msgs::Waypoint>& waypoints,
      geometry_msgs::Point min,
      geometry_msgs::Point max,
      geometry_msgs::Point viewport,
      int downsample);

    private:
    void octomap_callback(const octomap_msgs::Octomap &msg)
    {
      octomap::AbstractOcTree *octree = octomap_msgs::fullMsgToMap(msg);
      m_tree = dynamic_cast<octomap::OcTree*>(octree);
    }

    void publish_waypoints(void);

    void draw_normals(std::vector<dronenav_msgs::Waypoint>& waypoints);
    void draw_waypoints(std::vector<dronenav_msgs::Waypoint>& waypoints);

    private:
    //Nodes
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;

    //Subscribers
    ros::Subscriber m_octomap_sub;

    //Publishers
    ros::Publisher m_global_goal_pub;
    ros::Publisher m_marker_pub;

    //Service servers
    ros::ServiceServer m_waypoint_generator_server;

    //Other
    octomap::OcTree *m_tree;

    //Parameters
    double m_floor_threshold;
    double m_ceil_threshold;
    int m_depth_level;
    int m_downsample;
    double m_standoff_multiplier;
    bool m_draw_normals;
    bool m_draw_waypoints;
  };
}

#endif