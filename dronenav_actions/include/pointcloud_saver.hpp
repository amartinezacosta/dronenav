#ifndef POINTCLOUD_SAVER_HPP_
#define POINTCLOUD_SAVER_HPP_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <dronenav_msgs/SavePointCloudAction.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace dronenav_actions
{
  class PointCloudSaver
  {
    public:
    PointCloudSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~PointCloudSaver();

    private:
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void action_callback(const dronenav_msgs::SavePointCloudGoalConstPtr& goal);

    private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;

    /*Action server*/
    actionlib::SimpleActionServer<
      dronenav_msgs::SavePointCloudAction> m_action_server;

    /*Point cloud subscription*/
    ros::Subscriber m_pointcloud_sub;

    /*Variables*/
    bool m_store_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> m_point_cloud;
    dronenav_msgs::SavePointCloudFeedback m_feedback;
    dronenav_msgs::SavePointCloudResult m_result;

    /*Parameters*/
    std::string m_pointcloud_topic;
    double m_min_delay;
    double m_max_delay;
  };
}

#endif