#ifndef DRONENAV_HPP_
#define DRONENAV_HPP_

#include <ros/ros.h>

#include <dronenav_msgs/Waypoint.h>
#include <dronenav_msgs/Path.h>
// #include <dronenav_msgs/PathGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <dronenav_msgs/Takeoff.h>
#include <dronenav_msgs/Land.h>
#include <dronenav_msgs/Status.h>
//#include <dronenav_msgs/PointCloudSave.h>

#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/statechart/state_machine.hpp>

#include <queue>

namespace dronenav
{
  struct Navigation;
  struct Drone : boost::statechart::state_machine<Drone, Navigation>
  {
    public:
    Drone(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~Drone();

    void connect(void);
    void arm(void);
    void offboard(void);

    dronenav_msgs::Waypoint get_current_waypoint(void){ return m_waypoint; }
    void flush(void)
    {
        std::queue<dronenav_msgs::Waypoint> empty;
        std::swap(m_waypoint_queue, empty);
    }
    void enqueue(dronenav_msgs::Waypoint waypoint);
    bool next(void);
    bool dequeue(void);
    void publish_waypoint_reached(void)
    {
        dronenav_msgs::Waypoint msg = get_current_waypoint();
        m_waypoint_reached.publish(msg);
    }

    geometry_msgs::Point& get_requested_takeoff_position(void){ return m_takeoff_position; }
    double& get_requested_takeoff_yaw(void) { return m_takeoff_yaw; }
    void land(void);
    void save_image(void);
    void record_video(void);
    void save_pointcloud(void);

    void set_target_position(double x, double y, double z)
    {
        m_target_position.x = x;
        m_target_position.y = y;
        m_target_position.z = z;
    }

    void set_target_position(geometry_msgs::Point position)
    { 
        m_target_position = position; 
    }
    
    void set_target_yaw(double yaw){ m_target_yaw = yaw; }
    bool set_cruise_speed(double speed);
    void set_state(std::string state){ m_dronenav_status.state = state; }

    geometry_msgs::Point get_current_position(void){ return m_current_position; }
    geometry_msgs::Point get_target_position(void){ return m_target_position; }
    double get_current_yaw(void){ return m_current_yaw; }
    double get_target_yaw(void){ return m_target_yaw; }
    
    //Parameter getters
    double get_reach_radius(void){ return m_reach_radius; }
    double get_yaw_min_error(void){ return m_yaw_min_error; }
    double get_moving_tick_res(void){ return m_moving_tick_resolution; }
    double get_cruise_speed(void){return m_cruise_speed; }
    double waypoint_timeout(void){ return m_waypoint_timeout; }

    private:
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    { 
        m_current_position = msg->pose.position;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        m_current_yaw = yaw;
    }
    void state_callback(const mavros_msgs::State::ConstPtr& msg);
    void waypoint_callback(const dronenav_msgs::Waypoint::ConstPtr &msg);
    void status_timer_callback(const ros::TimerEvent& evt);

    bool takeoff_service(dronenav_msgs::Takeoff::Request &request, 
            dronenav_msgs::Takeoff::Response &response);
    bool land_service(dronenav_msgs::Land::Request &request,
            dronenav_msgs::Land::Response &response);

    void pose_stream(void);

    public:
    ros::NodeHandle m_nh;

    private:
    ros::NodeHandle m_pvt_nh;
    
    //Subscribers
    ros::Subscriber m_state_sub;
    ros::Subscriber m_pose_sub;
    ros::Subscriber m_waypoint_sub;

    //Publishers
    ros::Publisher m_target_pose_pub;
    ros::Publisher m_drone_status_pub;
    ros::Publisher m_waypoint_reached;

    //Service clients
    ros::ServiceClient m_arm_client;
    ros::ServiceClient m_mode_client;
    ros::ServiceClient m_param_set_client;
    ros::ServiceClient m_image_save_client;
    ros::ServiceClient m_pointcloud_save_client;

    //Service server
    ros::ServiceServer m_takeoff_server;
    ros::ServiceServer m_land_server;

    //Timers
    ros::Timer m_status_timer;

    mavros_msgs::State m_mavros_state;
    dronenav_msgs::Status m_dronenav_status;
    geometry_msgs::Point m_current_position;
    geometry_msgs::Point m_target_position;
    double m_target_yaw;
    double m_current_yaw;
    dronenav_msgs::Waypoint m_waypoint;
    boost::thread m_pose_thread;
    bool m_stream_run;
    std::queue<dronenav_msgs::Waypoint> m_waypoint_queue;
    dronenav_msgs::Path current_path; 
    
    //Service request variables
    geometry_msgs::Point m_takeoff_position;
    double m_takeoff_yaw;

    //Parameters
    double m_cruise_speed;
    double m_yaw_rate;
    int m_connect_tries;
    int m_arm_tries;
    int m_set_cruise_speed_tries;
    int m_offboard_tries;
    double m_reach_radius;
    double m_yaw_min_error;
    double m_waypoint_timeout;
    double m_moving_tick_resolution;
  };
}

#endif