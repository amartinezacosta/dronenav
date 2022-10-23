#include <waypoint_generation.hpp>

namespace waypoint_generation
{
  WaypointGenerator::WaypointGenerator(ros::NodeHandle nh, 
      ros::NodeHandle pvt_nh) :
      m_nh(nh),
      m_pvt_nh(pvt_nh),
      m_tree(nullptr)
  {
      //Parameters
      m_pvt_nh.param("floor_threshold", m_floor_threshold, 2.0);
      m_pvt_nh.param("ceil_threshold", m_ceil_threshold, 25.0);
      m_pvt_nh.param("depth", m_depth_level, 16);
      m_pvt_nh.param("draw_normals", m_draw_normals, true);
      m_pvt_nh.param("draw_waypoints", m_draw_waypoints, true);
      m_pvt_nh.param("downsample", m_downsample, 100);
      m_pvt_nh.param("standoff_multiplier", m_standoff_multiplier, 2.0);

      //Subscribers
      m_octomap_sub = m_nh.subscribe("octomap_full", 10, 
          &WaypointGenerator::octomap_callback, this);

      //Publishers
      m_marker_pub = m_nh.advertise<
        visualization_msgs::Marker>("visualization_marker", 0);
  }

  void WaypointGenerator::generate_waypoints(
    std::vector<dronenav_msgs::Waypoint>& waypoints,
    geometry_msgs::Point min,
    geometry_msgs::Point max,
    geometry_msgs::Point viewport,
    const int downsample)
  {
    if(m_tree == nullptr)
    {
      ROS_ERROR("Octomap is invalid");
      return;
    }

    int downsample_index = 0;
    octomap::point3d octomap_viewport(
      viewport.x, 
      viewport.y, 
      viewport.z);

    /*Clamp z values to floor and ceil threshold*/
    if(min.z < m_floor_threshold) min.z = m_floor_threshold;
    if(max.z > m_ceil_threshold) max.z = m_ceil_threshold;

    for(octomap::OcTree::leaf_iterator it = m_tree->begin_leafs(), 
      end = m_tree->end_leafs(); it != end; it++)
    {
      if((downsample_index++ % downsample) != 0) continue;

      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      double depth = it.getDepth(); 

      if(m_tree->isNodeOccupied(*it)  &&
        (x > min.x) && (x < max.x)    &&         
        (y > min.y) && (y < max.y)    &&
        (z > min.z) && (z < max.z)    &&
        (depth <= m_depth_level))
      {
        std::vector<octomap::point3d> normals;
        if(m_tree->getNormals(it.getCoordinate(), normals))
        {
          for(octomap::point3d normal : normals)
          {
            double dot = normal.dot(octomap_viewport);

            if(dot > 0.9)
            {
              /*Create a new waypoint*/
              dronenav_msgs::Waypoint waypoint;
              waypoint.position.x = x;
              waypoint.position.y = y;
              waypoint.position.z = z;

              waypoint.normal.x = normal.x();
              waypoint.normal.y = normal.y();
              waypoint.normal.z = normal.z();

              waypoint.yaw = octomap_viewport.angleTo(normal);
              waypoint.action = 0;
              waypoint.flags = 0;

              waypoints.push_back(waypoint);
            }
          } 
        }
      }
    }

    if(m_draw_normals) draw_normals(waypoints);
    if(m_draw_waypoints) draw_waypoints(waypoints);

    /*Publish waypoints*/
  }

  void WaypointGenerator::draw_normals(
    std::vector<dronenav_msgs::Waypoint>& waypoints)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "normals";
    marker.pose.orientation.w = 1.0;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;

    for(dronenav_msgs::Waypoint& waypoint : waypoints)
    {
      geometry_msgs::Point n;
      n.x = waypoint.position.x + m_standoff_multiplier * waypoint.normal.x;
      n.y = waypoint.position.y + m_standoff_multiplier * waypoint.normal.y;
      n.z = waypoint.position.z + m_standoff_multiplier * waypoint.normal.z;

      marker.points.push_back(waypoint.position);
      marker.points.push_back(n);
    }

    if(!marker.points.empty()) m_marker_pub.publish(marker);
  }

  void WaypointGenerator::draw_waypoints(
    std::vector<dronenav_msgs::Waypoint>& waypoints)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "waypoints";
    marker.pose.orientation.w = 1.0;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    for(dronenav_msgs::Waypoint waypoint : waypoints)
    {
      geometry_msgs::Point n;
      n.x = waypoint.position.x + m_standoff_multiplier * waypoint.normal.x;
      n.y = waypoint.position.y + m_standoff_multiplier * waypoint.normal.y;
      n.z = waypoint.position.z + m_standoff_multiplier * waypoint.normal.z;

      marker.points.push_back(n);
    }

    if(!marker.points.empty()) m_marker_pub.publish(marker);
  }
}