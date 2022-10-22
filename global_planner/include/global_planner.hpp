#ifndef GLOBAL_PLANNER_HPP_
#define GLOBAL_PLANNER_HPP_

#include <ros/ros.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <dronenav_msgs/Waypoint.h>
#include <dronenav_msgs/Path.h>
#include <dronenav_msgs/GlobalGoal.h>
#include <dronenav_msgs/PlannerStatus.h>

#include <queue>

#include <vertex.hpp>

#include <boost/statechart/state_machine.hpp>

namespace global_planner
{
    struct Active;
    struct GlobalPlanner : boost::statechart::state_machine<GlobalPlanner, Active>
    {
        public:
        GlobalPlanner(ros::NodeHandle nh, ros::NodeHandle pvt_nh);

        bool goal_queue_empty(void){ return m_goal_queue.empty(); }
        void next_goal(void){ m_current_goal = m_goal_queue.front(); }
        void dequeue_goal(void){ m_goal_queue.pop_front(); }

        void enqueue_front(dronenav_msgs::GlobalGoal& waypoint){ m_goal_queue.push_front(waypoint); }
        void enqueue_back(dronenav_msgs::GlobalGoal& waypoint){ m_goal_queue.push_back(waypoint); }

        dronenav_msgs::Path& get_current_path(void){ return m_current_path; }
        dronenav_msgs::GlobalGoal& get_current_goal(void){ return m_current_goal; }
        dronenav_msgs::GlobalGoal& get_new_goal(void){ return m_new_goal; }

        bool find_global_path(void);
        ros::NodeHandle& get_node_handle(void) {return m_nh; }
        void send_path(void)
        {
            for(dronenav_msgs::Waypoint waypoint : m_current_path.waypoints)
            {
                m_waypoint_pub.publish(waypoint);
            }
        }

        private:
        void octomap_callback(const octomap_msgs::Octomap &msg)
        {
            octomap::AbstractOcTree *octree = octomap_msgs::fullMsgToMap(msg);
            m_tree = dynamic_cast<octomap::OcTree*>(octree);
            if(m_tree == nullptr) ROS_ERROR("Octomap is invalid");
        }

        void position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
        { 
            m_start = msg->pose.position; 
        }

        void goal_callback(const dronenav_msgs::GlobalGoal::ConstPtr& msg);
        void rviz_goal_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
        void path_done_callback(const std_msgs::Empty::ConstPtr& msg);
        // void status_tick_callback(const ros::TimerEvent& evt);

        void draw_vertices(std::vector<AStarVertex>& vertices);
        void draw_edges(std::vector<AStarVertex>& vertices);
        void draw_goal(dronenav_msgs::GlobalGoal& goal);
        void draw_start(void);
        void draw_path(void);
        void log_path_waypoints(void);
        
        AStarVertex* find_vertex(std::vector<AStarVertex>& vertices, 
            AStarVertex vertex);
        void find_neighbors(std::vector<AStarVertex>& vertices,
            AStarVertex *vertex);
        bool find_path(int priority, std::vector<AStarVertex>& vertices);
        void reconstruct_path(int priority, AStarVertex *current);
        double compute_yaw(dronenav_msgs::Waypoint& current, dronenav_msgs::Waypoint& next);

        void update_vertices(dronenav_msgs::GlobalGoal& goal,
            std::vector<AStarVertex>& vertices);
        void update_edges(std::vector<AStarVertex>& vertices);

        public:
        ros::NodeHandle m_nh;

        private:
        ros::NodeHandle m_pvt_nh;

        geometry_msgs::Point m_start;
        dronenav_msgs::GlobalGoal m_current_goal;
        dronenav_msgs::GlobalGoal m_new_goal;
        dronenav_msgs::Path m_current_path;
        std::deque<dronenav_msgs::GlobalGoal> m_goal_queue;
        dronenav_msgs::PlannerStatus m_status;
        //ros::Timer m_status_timer;
        //bool m_path_found;

        octomap::OcTree *m_tree;

        ros::Subscriber m_octomap_sub;
        ros::Subscriber m_position_sub;
        ros::Subscriber m_goal_sub;
        ros::Subscriber m_rviz_goal_sub;
        ros::Subscriber m_path_done_sub;

        ros::Publisher m_graph_marker_pub;
        ros::Publisher m_waypoint_pub;
        ros::Publisher m_path_planner_status_pub;

        //Node parameters
        int m_depth_level;
        int m_neighbor_max_count;
        double m_yaw_angle_path;
        double m_floor_threshold;
        double m_ceil_threshold;
        double m_goal_radius_mult;
        double m_start_radius_mult;
        bool m_yaw_towards_path;
        bool m_draw_edge;
        bool m_draw_vertices;
        bool m_draw_goal;
        bool m_draw_start;
        bool m_draw_path;
        bool m_log_path_waypoints;
    };
}

#endif
