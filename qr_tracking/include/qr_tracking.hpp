#ifndef QR_TRACKING_HPP_
#define QR_TRACKING_HPP_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Vector3.h>

#include <visualization_msgs/Marker.h>

#include <dronenav_msgs/Code.h>
#include <dronenav_msgs/TrackedCodes.h>

#include <zbar.h>

#include <map>

#include "code.hpp"

namespace qr_tracking
{
  class QRTracking
  {
    public:
    QRTracking(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
    ~QRTracking();

    private:
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void detect_codes(cv_bridge::CvImagePtr& cv_ptr, 
      std::vector<Code>& detections);
    void transform_code_coordinates(std::vector<Code>& detections);
    void track_codes(std::vector<Code>& detections);
    void publish_tracked_codes(void);
    void show_detections(cv_bridge::CvImagePtr& cv_ptr, 
      std::vector<Code>& detections);
    void show_detection_markers(std::vector<Code>& detections);
    void show_map_markers(void);

    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
      sensor_msgs::convertPointCloud2ToPointCloud(*msg, m_depth);
    }

    private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_pvt_nh;

    /*Subscribers*/
    image_transport::Subscriber m_image_sub;
    ros::Subscriber m_pointcloud_sub;

    /*Publishers*/
    image_transport::Publisher m_image_pub;
    ros::Publisher m_marker_pub;
    ros::Publisher m_tracked_codes_pub;

    /*Variables*/
    zbar::ImageScanner m_scanner;
    sensor_msgs::PointCloud m_depth;
    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener *m_tfListener;
    std::vector<Code> m_prev_detections;
    int m_frame_count;
    int m_track_id;
    std::map<int, Code> m_tracking_objects;

    /*Parameters*/
    std::string m_image_topic;
    std::string m_pointcloud_topic;
    std::string m_frame_id;
    std::string m_base_frame_id;
    int m_image_width;
    bool m_show_detections;
    bool m_show_detection_markers;
    bool m_show_map_markers;
  };
}

#endif
