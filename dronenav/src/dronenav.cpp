#include <dronenav.hpp>

#include <landed.hpp>
#include <flying.hpp>
#include <hovering.hpp>
#include <positioning.hpp>

namespace dronenav
{
  Drone::Drone(ros::NodeHandle nh, ros::NodeHandle pvt_nh) :
      m_nh(nh),
      m_pvt_nh(pvt_nh)
  {
    ROS_INFO_NAMED("dronenav", "Drone State Machine Initialized");

    //Parameters
    m_pvt_nh.param("connect_tries", m_connect_tries, 10);
    m_pvt_nh.param("arm_tries", m_arm_tries, 10);
    m_pvt_nh.param("offboard_mode_tries", m_offboard_tries, 10);
    m_pvt_nh.param("reach_radius_sqr", m_reach_radius, 0.2);
    m_pvt_nh.param("waypoint_timeout", m_waypoint_timeout, 10.0);
    m_pvt_nh.param("moving_tick_resolution", m_moving_tick_resolution, 0.1);
    m_pvt_nh.param("cruise_speed", m_cruise_speed, 10.0);
    m_pvt_nh.param("set_cruise_speed_tries", m_set_cruise_speed_tries, 10);
    m_pvt_nh.param("yaw_rate", m_yaw_rate, 0.35);
    m_pvt_nh.param("yaw_min_error", m_yaw_min_error, 0.1);

    //Subscribers
    m_state_sub = m_nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Drone::state_callback, this);
    m_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Drone::pose_callback, this);
    m_waypoint_sub = m_nh.subscribe<dronenav_msgs::Waypoint>("dronenav/waypoint", 100, &Drone::waypoint_callback, this);

    //Publishers
    m_target_pose_pub = m_nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    //_target_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    m_drone_status_pub = m_nh.advertise<dronenav_msgs::Status>("dronenav/status", 30);
    m_waypoint_reached = m_nh.advertise<dronenav_msgs::Waypoint>("dronenav/waypoint/reached", 50);

    //Service clients
    m_arm_client = m_nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    m_mode_client = m_nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    m_param_set_client = m_nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
    //m_pointcloud_save_client = m_nh.serviceClient<dronenav_msgs::PointCloudSave>("dronenav/save/pointcloud");
    //m_image_save_client = m_nh.serviceClient<dronenav_msgs::PointCloudSave>("dronenav/save/image");
    //m_video_save_client = m_nh.serviceClient<dronenav_msgs::PointCloudSave>("dronenav/save/video");
    
    //Service servers
    m_takeoff_server = m_nh.advertiseService("dronenav/takeoff", &Drone::takeoff_service, this);
    m_land_server = m_nh.advertiseService("dronenav/land", &Drone::land_service, this);

    //Timers
    m_status_timer = m_nh.createTimer(ros::Duration(0.1), &Drone::status_timer_callback, this);

    //Initialize pose stream thread
    m_stream_run = true;
    m_pose_thread = boost::thread(boost::bind(&Drone::pose_stream, this));
  }

  Drone::~Drone(void)
  {
    /*Close pose stream thread here!*/
    m_stream_run = false;
    m_pose_thread.join();
  }

  void Drone::connect(void)
  {
    int tries = m_connect_tries;
    bool connected = false;

    ROS_INFO_NAMED("dronenav", "Connecting to flight control unit");
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    while(tries)
    {
        if(m_mavros_state.connected)
        {
            ROS_INFO_NAMED("dronenav", "Flight control unit connected");
            connected = true;
            break;
        }
        else
        {
            tries--;
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
    }

    if(!connected)
    {
        ROS_ERROR_NAMED("dronenav", "Flight control unit is not connected");
        //TODO: transition to exception state
    }
  }


  void Drone::arm(void)
  {
    int tries = m_arm_tries;
    bool armed = false;

    mavros_msgs::CommandBool msg;
    msg.request.value = true;

    m_arm_client.call(msg);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    ROS_INFO_NAMED("dronenav", "Arming vehicle");
    while(tries)
    {
        if(m_mavros_state.armed)
        {
            ROS_INFO_NAMED("dronenav", "Vehicle armed");
            armed = true;
            break;
        }
        else
        {
            tries--;
            m_arm_client.call(msg);
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
    }

    if(!armed) 
    {
        ROS_ERROR_NAMED("dronenav", "Failed to arm vehicle");
        //TODO: transition to exception state
    }
  }
  void Drone::offboard(void)
  {
    int tries = m_arm_tries;
    bool enabled = false;

    mavros_msgs::SetMode msg;
    msg.request.custom_mode = "OFFBOARD";

    m_mode_client.call(msg);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    ROS_INFO_NAMED("dronenav", "Switching to offboard control");
    while(tries)
    {
        if(m_mavros_state.mode == "OFFBOARD")
        {
            enabled = true;
            ROS_DEBUG_NAMED("dronenav", "Offboard control enabled");
            break;
        }
        else
        {
            tries--;
            m_mode_client.call(msg);
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
    }

    if(!enabled) 
    {
        ROS_ERROR_NAMED("dronenav", "Failed to switch to offboard mode");
        //TODO: transition to exception state
    }
  }

  bool Drone::set_cruise_speed(double cruise_speed)
  {
    int tries = m_set_cruise_speed_tries;
    bool set = false;

    mavros_msgs::ParamSet srv;
    srv.request.param_id = "MPC_XY_VEL_MAX";
    srv.request.value.real = cruise_speed;
    srv.request.value.integer = 0;

    m_param_set_client.call(srv);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    ROS_INFO_NAMED("dronenav", "Changing cruise speed to %f m/s", cruise_speed);
    while(tries)
    {
        if(srv.response.success)
        {
            set = true;
            ROS_INFO_NAMED("dronenav", "Cruise speed set");
            break;
        }
        else
        {
            tries--;
            m_param_set_client.call(srv);
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
    }

    if(!set) 
    {
        ROS_ERROR_NAMED("dronenav", "Could not set cruise speed! Response %f", 
            srv.response.value.real);
        //TODO: transition to exception state
    }

    return set;
  }

  void Drone::status_timer_callback(const ros::TimerEvent& evt)
  {
    m_dronenav_status.current_position = get_current_position();
    m_dronenav_status.target_position = get_target_position();
    m_dronenav_status.current_yaw = get_current_yaw();
    m_dronenav_status.target_yaw = get_target_yaw();
    m_dronenav_status.waypoint_queue_size = m_waypoint_queue.size();

    m_drone_status_pub.publish(m_dronenav_status);
  }

  void Drone::state_callback(const mavros_msgs::State::ConstPtr& msg)
  {
    m_mavros_state = *msg;

    //TODO: states can be used to cause transitions. Find
    //out what states could be useful
  }

  void Drone::waypoint_callback(const dronenav_msgs::Waypoint::ConstPtr& msg)
  {
    if((*msg).flags & dronenav_msgs::Waypoint::FLUSH_QUEUE)
    {
        flush();
    }
    
    //Enqueue new waypoint
    enqueue(*msg);
    //Process the waypoint received event
    process_event(EvWaypointReceived());
  }

  void Drone::land(void)
  {
    mavros_msgs::SetMode msg;
    msg.request.custom_mode = "AUTO.LAND";

    m_mode_client.call(msg);
    ros::spinOnce();
  }

  void Drone::save_image(void)
  {
    actionlib::SimpleActionClient<
      dronenav_msgs::SaveImageAction> action_client("dronenav/save_image", true);
    
    /*Wait for server to start*/
    action_client.waitForServer();

    /*Send goal*/
    dronenav_msgs::SaveImageGoal goal;
    std::stringstream file_name;
    file_name << "dronenav_image_" << ros::Time::now().toSec(); 

    goal.file_name = file_name.str();
    goal.count = 1;
    goal.delay = 0.0;
    action_client.sendGoal(goal);

    bool timeout = action_client.waitForResult(ros::Duration(5.0));
    if(timeout)
    {
      actionlib::SimpleClientGoalState state = action_client.getState();
      ROS_INFO_NAMED("dronenav", "Image action state = %s", state.toString().c_str());

      dronenav_msgs::SaveImageResultConstPtr result = action_client.getResult();
      ROS_INFO_NAMED("dronenav", "Image action result count = %i", result->count);
    }
    else
    {
      ROS_WARN_NAMED("dronenav", "Image action timeout");
    }
  }

  void Drone::save_pointcloud(void)
  {
    actionlib::SimpleActionClient<
      dronenav_msgs::SavePointCloudAction> action_client("dronenav/save_pointcloud", true);
    
    /*Wait for server to start*/
    action_client.waitForServer();

    /*Send goal*/
    dronenav_msgs::SavePointCloudGoal goal;
    std::stringstream file_name;
    file_name << "dronenav_pointcloud_" << ros::Time::now().toSec(); 

    goal.file_name = file_name.str();
    goal.count = 1;
    goal.delay = 0.0;
    action_client.sendGoal(goal);

    bool timeout = action_client.waitForResult(ros::Duration(5.0));
    if(timeout)
    {
      actionlib::SimpleClientGoalState state = action_client.getState();
      ROS_INFO_NAMED("dronenav", "Pointcloud action state = %s", state.toString().c_str());

      dronenav_msgs::SavePointCloudResultConstPtr result = action_client.getResult();
      ROS_INFO_NAMED("dronenav", "Pointcloud action result count = %i", result->count);
    }
    else
    {
      ROS_WARN_NAMED("dronenav", "Pointcloud action timeout");
    }
  }

  void Drone::enqueue(dronenav_msgs::Waypoint waypoint)
  {
    m_waypoint_queue.push(waypoint);
  }

  bool Drone::next(void)
  {
    bool result = false;
    if(!m_waypoint_queue.empty())
    {
        m_waypoint = m_waypoint_queue.front();
        result = true;
    }

    return result;
  }

  bool Drone::dequeue(void)
  {
    bool result = false;
    if(!m_waypoint_queue.empty())
    {
        m_waypoint_queue.pop();
        result = true;
    }

    return result;
  }

  void Drone::pose_stream(void)
  {
    ROS_INFO_NAMED("dronenav", "Pose stream thread initialized");
    ros::Rate rate(20.0);

    int sequence = 0;

    while(m_stream_run)
    {
        //Publish drone pose
        mavros_msgs::PositionTarget msg;
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_VX  |
                        mavros_msgs::PositionTarget::IGNORE_VY  |
                        mavros_msgs::PositionTarget::IGNORE_VZ  |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "body_link";
        msg.header.seq = sequence++;
        msg.position = m_target_position;
        msg.yaw = m_target_yaw;

        m_target_pose_pub.publish(msg);
        rate.sleep();
    }
  }

  bool Drone::takeoff_service(dronenav_msgs::Takeoff::Request& rqt, 
          dronenav_msgs::Takeoff::Response& rsp)
  {
    ROS_INFO_NAMED("dronenav", "Takeoff service requested x=%f, y=%f, z=%f, yaw=%f", 
        rqt.x, rqt.y, rqt.z, rqt.yaw);

    m_takeoff_position.x = rqt.x;
    m_takeoff_position.y = rqt.y;
    m_takeoff_position.z = rqt.z;
    m_takeoff_yaw = rqt.yaw;
    
    rsp.success = true;

    process_event(EvTakeoff());
    
    return true;
  }

  bool Drone::land_service(dronenav_msgs::Land::Request &rqt,
          dronenav_msgs::Land::Response &rsp)
  {
    ROS_INFO_NAMED("dronenav", 
    "Landing service requested, override takeoff pose: %s, x=%f, y=%f, z=%f, yaw=%f",
    rqt.override_takeoff_pose ? "true" : "false", rqt.x, rqt.y, rqt.z, rqt.yaw);

    if(rqt.override_takeoff_pose)
    {
        m_takeoff_position.x = rqt.x;
        m_takeoff_position.y = rqt.y;
        m_takeoff_position.z = rqt.z;
        m_takeoff_yaw = rqt.yaw;
    }

    rsp.success = true;

    process_event(EvLand());

    return true;
  }
}
