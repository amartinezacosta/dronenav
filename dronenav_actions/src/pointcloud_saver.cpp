#include <pointcloud_saver.hpp>

namespace dronenav_actions
{
    PointCloudSaver::PointCloudSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh) :
      m_nh(nh),
      m_pvt_nh(pvt_nh),
      m_store_cloud(false),
      m_action_server(nh, "dronenav/save_pointcloud",
        boost::bind(&PointCloudSaver::action_callback, this, _1), 
        false)
    {
      /*Parameters*/
      m_pvt_nh.param<std::string>("pointcloud_topic", m_pointcloud_topic, "camera/depth/points");

      /*Subscribers*/
      m_pointcloud_sub = m_nh.subscribe(m_pointcloud_topic, 10, 
        &PointCloudSaver::pointcloud_callback, this);

      /*Start action server*/
      ROS_INFO_NAMED("dronenav_actions", "Starting Point Cloud Action Server");
      m_action_server.start();
    }

    PointCloudSaver::~PointCloudSaver()
    {

    }

    void PointCloudSaver::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
      if(!m_store_cloud) return;
      pcl::fromROSMsg(*msg, m_point_cloud);
    }

    void PointCloudSaver::action_callback(const dronenav_msgs::SavePointCloudGoalConstPtr& goal)
    { 
      ROS_INFO_NAMED("dronenav_actions", "Point-cloud save action requested");

      m_store_cloud = true;
      bool success = true;
      double delay = goal->delay;

      if(delay > m_max_delay) delay = m_max_delay;
      if(delay < m_min_delay) delay = m_min_delay;

      m_feedback.count = 0;
      m_result.count = 0;

      ros::Duration(1.0).sleep();

      for(int i = 0; i < goal->count; i++)
      {
        if(m_action_server.isPreemptRequested() || !ros::ok())
        {
          m_action_server.setPreempted();
          success = false;
          break;
        }

        /*Save point-cloud*/
        if(!m_point_cloud.empty())
        {
          std::stringstream file_name;
          file_name << goal->file_name << "_" << std::setw(3) <<
            std::setfill('0') << i << ".pcd";

          ROS_INFO_NAMED("dronenav_actions", "Saving Point-Cloud: %s",
            file_name.str().c_str());
          pcl::io::savePCDFileASCII(file_name.str(), m_point_cloud);
          m_feedback.count++;
        }

        m_action_server.publishFeedback(m_feedback);
        ros::Duration(goal->delay).sleep();
      }

      if(success)
      {
        m_result.count = m_feedback.count;
        m_action_server.setSucceeded(m_result);
      }

      m_store_cloud = false;
    }
}