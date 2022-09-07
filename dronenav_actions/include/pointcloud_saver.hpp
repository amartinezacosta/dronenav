#ifndef POINTCLOUD_SAVER_HPP_
#define POINTCLOUD_SAVER_HPP_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <dronenav_msgs/PointCloudSave.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace dronenav_actions
{
  class PointCloudSaver
  {
    public:
    PointCloudSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~PointCloudSaver();

    private:
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    bool pointcloud_save_service(dronenav_msgs::PointCloudSaveRequest& rqt,
      dronenav_msgs::PointCloudSaveResponse& rsp);

    private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;

    ros::Subscriber m_pointcloud_sub;
    ros::ServiceServer m_pointcloud_saver_server;

    /*Variables*/
    int m_save_count;
    std::string m_pointcloud_name;

    /*Parameters*/
    std::string m_pointcloud_topic;
  };
}

#endif