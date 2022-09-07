#include "global_planner.hpp"

#include "waiting.hpp"

namespace global_planner
{
    GlobalPlanner::GlobalPlanner(ros::NodeHandle nh, ros::NodeHandle pvt_nh) : 
        m_nh(nh), 
        m_pvt_nh(pvt_nh)
    {
        //Parameters
        m_pvt_nh.param("depth", m_depth_level, 13);
        m_pvt_nh.param("neighbor_max_count", m_neighbor_max_count, 4);
        m_pvt_nh.param("start_radius_multiplier", m_start_radius_mult, 2.0);
        m_pvt_nh.param("goal_radius_multiplier", m_goal_radius_mult, 2.0);
        m_pvt_nh.param("floor_threshold", m_floor_threshold, 0.5);
        m_pvt_nh.param("ceil_threshold", m_ceil_threshold, 3.0);
        m_pvt_nh.param("draw_vertices", m_draw_vertices, true);
        m_pvt_nh.param("yaw_angle_path", m_yaw_angle_path, 0.0);
        m_pvt_nh.param("yaw_towards_path", m_yaw_towards_path, false);
        m_pvt_nh.param("draw_edges", m_draw_edge, true);
        m_pvt_nh.param("draw_goal", m_draw_goal, true);
        m_pvt_nh.param("draw_start", m_draw_start, true);
        m_pvt_nh.param("draw_path", m_draw_path, true);
        m_pvt_nh.param("log_path_waypoints", m_log_path_waypoints, false);
        
        //Subscribers
        m_octomap_sub = m_nh.subscribe("octomap_full", 10, &GlobalPlanner::octomap_callback, this);
        m_position_sub = m_nh.subscribe("mavros/local_position/pose", 200, &GlobalPlanner::position_callback, this);
        m_rviz_goal_sub = m_nh.subscribe("/clicked_point", 10, &GlobalPlanner::rviz_goal_callback, this);
        m_goal_sub = m_nh.subscribe("dronenav/global_goal", 100, &GlobalPlanner::goal_callback, this);

        //Publishers
        m_path_planner_status_pub = m_nh.advertise<dronenav_msgs::PlannerStatus>("dronenav_msgs/global_planner/status", 50);
        m_graph_marker_pub = m_nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
        m_waypoint_pub = m_nh.advertise<dronenav_msgs::Waypoint>("dronenav/waypoints", 100);

        //Status timer
        m_status_timer = m_nh.createTimer(ros::Duration(0.1), 
            &GlobalPlanner::status_tick_callback, this);
    }

    void GlobalPlanner::rviz_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
    {
        //Define goal
        m_new_goal.x = msg->point.x;
        m_new_goal.y = msg->point.y;
        m_new_goal.z = msg->point.z;
        
        //TODO: is there a way to get direction from rviz point selection?
        m_new_goal.yaw = 0.0;
        m_new_goal.priority = 0;

        //Enqueue goal
        enqueue_back(m_new_goal);
        process_event(EvGoalReceived());
    }

    void GlobalPlanner::goal_callback(const dronenav_msgs::PathGoal::ConstPtr& msg)
    {
        //Priority checking must happen here!
        m_new_goal = *msg;
        if(m_new_goal.priority > m_current_goal.priority)
        {
            //Interrrupt current path
            process_event(EvPathInterrupt());
            return;
        }

        //Enqueue goal
        enqueue_back(m_new_goal);
        process_event(EvGoalReceived());
    }

    void GlobalPlanner::status_tick_callback(const ros::TimerEvent& evt)
    {
        dronenav_msgs::PlannerStatus status;
        status.queue_size = m_goal_queue.size();
        status.path_found = m_path_found;

        m_path_planner_status_pub.publish(status);
    }

    bool GlobalPlanner::find_global_path(void)
    {
        ROS_INFO("Attempting to find path from provided navigation point...");
        
        ros::WallTime start, end;
        start = ros::WallTime::now();

        std::vector<AStarVertex> vertices; 
        //Step 1. Find vertices withing bounding box
        update_vertices(m_current_goal, vertices);
        //Step 2. Find edges between vertices
        update_edges(vertices);
        //Step 3. Find path
        bool found = find_path(m_current_goal.priority, vertices);

        end = ros::WallTime::now();
        double execution_time = (end - start).toNSec() * 1e-6;
        ROS_INFO("Path planned execution time (ms): %f", execution_time);

        return found;
    }

    void GlobalPlanner::update_vertices(dronenav_msgs::PathGoal& goal,
        std::vector<AStarVertex>& vertices)
    {
        //Start and goal always go first
        vertices.push_back(AStarVertex(m_start.x, 
            m_start.y,
            m_start.z,
            m_depth_level));

        //Start and goal always go first
        vertices.push_back(AStarVertex(goal.x, 
            goal.y,
            goal.z,
            m_depth_level));

        double min_x = (m_start.x < goal.x) ? m_start.x : goal.x;
        double min_y = (m_start.y < goal.y) ? m_start.y : goal.y;
        double min_z = (m_start.z < goal.z) ? m_start.z : goal.z;

        double max_x = (m_start.x > goal.x) ? m_start.x : goal.x;
        double max_y = (m_start.y > goal.y) ? m_start.y : goal.y;
        double max_z = (m_start.z > goal.z) ? m_start.z : goal.z;

        octomap::point3d min(min_x, min_y, min_z);
        octomap::point3d max(max_x, max_y, max_z);

        for(octomap::OcTree::leaf_bbx_iterator it = m_tree->begin_leafs_bbx(min, max),
            end = m_tree->end_leafs_bbx(); it != end; it++)
        {
            double depth = it.getDepth();
            double z = it.getZ();

            if( (depth < m_depth_level) &&
                (z > m_floor_threshold) &&
                (z < m_ceil_threshold))
            {
                double x = it.getX();
                double y = it.getY();

                vertices.push_back(AStarVertex(x, y, z, depth));
            }
        }

        if(m_draw_vertices) draw_vertices(vertices);
        if(m_draw_start) draw_start();
        if(m_draw_goal) draw_goal(goal);
    }

    void GlobalPlanner::update_edges(std::vector<AStarVertex>& vertices)
    {
        //Find neighbors for start and goal
        find_neighbors(vertices, &vertices[0]);
        find_neighbors(vertices, &vertices[1]);

        //Find neighbors for the rest of the vertices
        for(int i = 2; i < vertices.size(); i++)
        {
            find_neighbors(vertices, &vertices[i]);
        }

        if(m_draw_edge) draw_edges(vertices);
    }

    void GlobalPlanner::find_neighbors(std::vector<AStarVertex>& vertices, 
        AStarVertex *vertex)
    {
        double x = vertex->get_x();
        double y = vertex->get_y();
        double z = vertex->get_z();
        int depth = vertex->get_depth();
        
        double vertex_size = m_tree->getResolution() * (1 << (16 - depth));
        octomap::point3d min(x - vertex_size, y - vertex_size, z - vertex_size);
        octomap::point3d max(x + vertex_size, y + vertex_size, z + vertex_size);

        for(octomap::OcTree::leaf_bbx_iterator it = m_tree->begin_leafs_bbx(min, max),
            end = m_tree->end_leafs_bbx(); it != end; it++)
        {
            double nx = it.getX();
            double ny = it.getY();
            double nz = it.getZ();
            double ndepth = it.getDepth();
            AStarVertex query(nx, ny, nz, ndepth);

            AStarVertex *neighbor = find_vertex(vertices, query);
            if(neighbor != NULL)
            { 
                vertex->add_neighbor(neighbor); 
                neighbor->add_neighbor(vertex);

                if(vertex->get_neighbors().size() > 
                    m_neighbor_max_count)
                    break;
            }
        }
    }

    AStarVertex* GlobalPlanner::find_vertex(std::vector<AStarVertex>& vertices, 
        AStarVertex query)
    {
        AStarVertex* found = NULL;

        std::vector<AStarVertex>::iterator it = 
            find(vertices.begin(), vertices.end(), query);
        if(it != vertices.end())
        {
            int index = it - vertices.begin();
            found = &vertices[index];
        }

        return found;
    }

    bool GlobalPlanner::find_path(int priority,
        std::vector<AStarVertex>& vertices)
    {    
        std::vector<AStarVertex*> open_set;

        AStarVertex *start = &vertices[0];
        AStarVertex *goal = &vertices[1];
        
        //Utility lambda functions
        auto heuristics = [](AStarVertex* v1, AStarVertex* v2)
        {
            double dx = v2->get_x() - v1->get_x();
            double dy = v2->get_y() - v1->get_y();
            double dz = v2->get_z() - v1->get_z();

            return sqrt(dx*dx + dy*dy + dz*dz);
        };

        auto in_openset = [](std::vector<AStarVertex*> set, AStarVertex *query)
        {
            return (std::find(set.begin(), set.end(), query) != set.end());
        };

        start->set_g(0.0);
        double h_start = heuristics(start, goal);
        start->set_h(h_start);
        start->set_f(h_start);

        open_set.push_back(start);

        while(!open_set.empty())
        {
            std::sort(open_set.begin(), open_set.end(), [](AStarVertex *lhs, AStarVertex *rhs)
                    {
                        return lhs->get_f() < rhs->get_f();
                    });

            AStarVertex *current = open_set[0];

            if(current == goal)
            {
                reconstruct_path(priority, current);
                return true;
            }

            open_set.erase(open_set.begin());
            
            std::vector<Vertex*> neighbors = current->get_neighbors();
            for(int i = 0; i < neighbors.size(); i++)
            {
                AStarVertex *neighbor = (AStarVertex*)neighbors[i];
                double tentive_g = current->get_g() + heuristics(neighbor, current);

                if(tentive_g < neighbor->get_g())
                {
                    neighbor->set_parent(current);
                    neighbor->set_g(tentive_g);
                    
                    double h = heuristics(neighbor, goal);
                    neighbor->set_h(h);
                    neighbor->set_f(tentive_g + h);
                    
                    if(!in_openset(open_set, neighbor))
                    {
                        open_set.push_back(neighbor);
                    }
                }
            }
        }

        return false;
    }

    void GlobalPlanner::reconstruct_path(int priority,
        AStarVertex *current)
    {
        std::vector<dronenav_msgs::Waypoint> waypoints;
        dronenav_msgs::Waypoint waypoint;

        //Last point contains the action
        waypoint.position.x = current->get_x();
        waypoint.position.y = current->get_y();
        waypoint.position.z = current->get_z();
        //Set the final yaw angle to the one requested by the user
        waypoint.yaw = 0.0; 
        //TODO: make this a parameter
        waypoint.action = dronenav_msgs::Waypoint::WAYPOINT_ACTION_IMAGE | 
                          dronenav_msgs::Waypoint::WAYPOINT_ACTION_POINTCLOUD;
        
        //Last item gets flagged
        waypoint.flags = dronenav_msgs::Waypoint::PATH_LAST_FLAG;
        waypoints.push_back(waypoint);

        while((current = current->get_parent()) != nullptr)
        {
            //Last point contains the action
            waypoint.position.x = current->get_x();
            waypoint.position.y = current->get_y();
            waypoint.position.z = current->get_z();
            //Will be changed
            waypoint.yaw = 0.0;
            waypoint.action = dronenav_msgs::Waypoint::WAYPOINT_ACTION_NONE;
            waypoint.flags = 0;
            waypoints.push_back(waypoint);
        }

        //Push current position, will be used later to set the intial yaw angle
        waypoint.position.x = m_start.x;
        waypoint.position.y = m_start.y;
        waypoint.position.z = m_start.z;
        waypoint.yaw = 0.0;
        waypoints.push_back(waypoint);

        //Clear and set current path by reversing the order of the waypoints
        m_current_path.waypoints.clear();
        for(int i = (waypoints.size() - 1); i >= 0; i--)
        {
            m_current_path.waypoints.push_back(waypoints[i]);
        }

        //Compute yaw angles so that the drone always faces forward
        //if requested by the user
        for(int i = 0; i < m_current_path.waypoints.size() - 1; i++)
        {
            dronenav_msgs::Waypoint& current = m_current_path.waypoints[i];
            dronenav_msgs::Waypoint& next = m_current_path.waypoints[i+1];

            current.yaw = compute_yaw(current, next);
        }

        //Signal waypoint queue flush
        m_current_path.waypoints.front().flags = dronenav_msgs::Waypoint::FLUSH_QUEUE;
        //Last waypoint contains the target yaw of the drone
        m_current_path.waypoints.back().yaw = m_current_goal.yaw;

        //draw path
        if(m_draw_path) draw_path();
        if(m_log_path_waypoints) log_path_waypoints();
    }

    double GlobalPlanner::compute_yaw(dronenav_msgs::Waypoint &current, 
        dronenav_msgs::Waypoint& next)
    {
        double yaw;
        if(m_yaw_towards_path)
        {
            double dx = next.position.x - current.position.x;
            double dy = next.position.y - current.position.y;
            yaw = atan2(dy, dx);
        }
        else
        {
            yaw = m_yaw_angle_path;
        }

        return yaw;
    }

    void GlobalPlanner::log_path_waypoints(void)
    {
        for(int i = 0; i < m_current_path.waypoints.size(); i++)
        {
            dronenav_msgs::Waypoint& waypoint = m_current_path.waypoints[i];
            ROS_INFO("waypoint[%i] -> x=%f, y=%f, z=%f, yaw=%f",
                i, 
                waypoint.position.x, 
                waypoint.position.y, 
                waypoint.position.z,
                waypoint.yaw);
        }
    }

    void GlobalPlanner::draw_vertices(std::vector<AStarVertex>& vertices)
    {
        ///Graph vertex
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "vertices";
        marker.pose.orientation.w = 1.0;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for(Vertex vertex : vertices)
        {
            geometry_msgs::Point point;
            point.x = vertex.get_x();
            point.y = vertex.get_y();
            point.z = vertex.get_z();

            marker.points.push_back(point);
        }
        
        if(marker.points.size()) 
            m_graph_marker_pub.publish(marker);
    }

    void GlobalPlanner::draw_edges(std::vector<AStarVertex>& vertices)
    {
        //Graph edges markers
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "edges";
        marker.pose.orientation.w = 1.0;
        marker.id = 1;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.scale.z = 0.005;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        for(AStarVertex vertex : vertices)
        {
            geometry_msgs::Point vpoint;
            vpoint.x = vertex.get_x();
            vpoint.y = vertex.get_y();
            vpoint.z = vertex.get_z();

            for(Vertex *neighbor : vertex.get_neighbors())
            {
                geometry_msgs::Point npoint;
                npoint.x = neighbor->get_x();
                npoint.y = neighbor->get_y();
                npoint.z = neighbor->get_z();
                
                marker.points.push_back(vpoint);
                marker.points.push_back(npoint);
            }
        }

        if(marker.points.size())
            m_graph_marker_pub.publish(marker);
    }

    void GlobalPlanner::draw_goal(dronenav_msgs::PathGoal& goal)
    {
        //Graph goal marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "goal";
        marker.id = 3;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = goal.x;
        marker.pose.position.y = goal.y;
        marker.pose.position.z = goal.z;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.20;
        marker.scale.y = 0.20;
        marker.scale.z = 0.20;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        m_graph_marker_pub.publish(marker);
    }

    void GlobalPlanner::draw_start(void)
    {
        //Graph start marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "start";
        marker.id = 2;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = m_start.x;
        marker.pose.position.y = m_start.y;
        marker.pose.position.z = m_start.z;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.20;
        marker.scale.y = 0.20;
        marker.scale.z = 0.20;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        m_graph_marker_pub.publish(marker);
    }

    void GlobalPlanner::draw_path(void)
    {
        //Graph path marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "global_path";
        marker.id = 4;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.20;
        marker.scale.y = 0.20;
        marker.scale.z = 0.20;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        //Path marker
        geometry_msgs::Point start, end;
        //Between drone current's position and first waypoint
        start.x = m_start.x;
        start.y = m_start.y;
        start.z = m_start.z;

        end.x = m_current_path.waypoints[0].position.x;
        end.y = m_current_path.waypoints[0].position.y;
        end.z = m_current_path.waypoints[0].position.z;

        marker.points.push_back(end);
        marker.points.push_back(start);

        for(int i = 0; i < m_current_path.waypoints.size() - 1; i++)
        {    
            start.x = m_current_path.waypoints[i].position.x;
            start.y = m_current_path.waypoints[i].position.y;
            start.z = m_current_path.waypoints[i].position.z;

            end.x = m_current_path.waypoints[i + 1].position.x;
            end.y = m_current_path.waypoints[i + 1].position.y;
            end.z = m_current_path.waypoints[i + 1].position.z;
            
            marker.points.push_back(start);
            marker.points.push_back(end);
        }

        m_graph_marker_pub.publish(marker);
    }
}
