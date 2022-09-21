#include <pointcloud_saver.hpp>

namespace dronenav_actions
{
    PointCloudSaver::PointCloudSaver(ros::NodeHandle nh, ros::NodeHandle pvt_nh)
    {
      /*Parameters*/
      m_pvt_nh.param<std::string>("pointcloud_topic", m_pointcloud_topic, "camera/depth/points");

      /*Subscribers*/
      m_pointcloud_sub = m_nh.subscribe("dronenav/save/pointcloud", 10, 
        &PointCloudSaver::pointcloud_callback, this);

      /*Service servers*/
      m_pointcloud_saver_server = m_nh.advertiseService("dronenav/save/pointcloud", 
        &PointCloudSaver::pointcloud_save_service, this);
    }

    PointCloudSaver::~PointCloudSaver()
    {

    }

    void PointCloudSaver::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
      /*Is the count bigger than 0?*/
      if(!m_save_count) return; 
      
      sensor_msgs::PointCloud pc;
      sensor_msgs::convertPointCloud2ToPointCloud(*msg, pc);

      pcl::PointCloud<pcl::PointXYZ> cloud;
      cloud.width = pc.points.size();
      cloud.height = 1;
      cloud.is_dense = false;
      cloud.points.resize(cloud.width * cloud.height);

      for(int i = 0; i < pc.points.size(); i++)
      {
        cloud.points[i].x = pc.points[i].x;
        cloud.points[i].y = pc.points[i].y;
        cloud.points[i].z = pc.points[i].z;
      }

      std::stringstream filename;
      filename << m_pointcloud_name << std::setw(3) <<
        std::setfill('0') << m_save_count << ".xyz";

      pcl::io::savePCDFileASCII(filename.str(), cloud);

      m_save_count--;
    }

    bool PointCloudSaver::pointcloud_save_service(dronenav_msgs::PointCloudSaveRequest& rqt,
      dronenav_msgs::PointCloudSaveResponse& rsp)
    { 
      m_save_count = rqt.count;
      m_pointcloud_name = rqt.name;
      rsp.success = true;

      return true;
    }
}