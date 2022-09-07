#ifndef DEEPNET_HPP_
#define DEEPNET_HPP_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/calib3d.hpp>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <Eigen/Dense>

#include <tf2/LinearMath/Vector3.h>

#include <dronenav_msgs/Waypoint.h>
#include <dronenav_msgs/PathGoal.h>

#include <object.hpp>

#include <string>
#include <vector>
#include <fstream>

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state.hpp>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>



// struct EvObjectDetected : boost::statechart::event<EvObjectDetected> {};
// struct EvObjectInspected : boost::statechart::event<EvObjectInspected> {};

namespace object_recognition
{
  struct Waiting;
  struct ObjectDetection : boost::statechart::state_machine<ObjectDetection, Waiting>
  {
      public:
      ObjectDetection(ros::NodeHandle nh, ros::NodeHandle pvt_nh);
      ~ObjectDetection(){ delete m_tfListener; }

      private:
      void image_callback(const sensor_msgs::ImageConstPtr &msg);
      void detect_objects(cv_bridge::CvImagePtr cv_ptr);
      void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
      {
          sensor_msgs::convertPointCloud2ToPointCloud(*msg, m_depth);
      }

      void segment_pointcloud(sensor_msgs::PointCloud &output, 
          int u, int v, int width, int height);
      void transform_minmax(pcl::PointXYZ& min_in, pcl::PointXYZ& max_in,
          geometry_msgs::Vector3& min_out, geometry_msgs::Vector3& max_out);
      void pointcloud_minmax(sensor_msgs::PointCloud& cloud_in, 
          pcl::PointXYZ& min, pcl::PointXYZ& max);
      Object construct_object_box(geometry_msgs::Vector3& min, 
          geometry_msgs::Vector3& max, std::string class_name);
      void object_bounding_box(std::string& class_name, 
          int u, int v, int width, int height);

      void draw_object(Object& object);
      void draw_front_face(visualization_msgs::Marker& marker,
          geometry_msgs::Vector3& min, 
          geometry_msgs::Vector3& max);
      void draw_minmax(geometry_msgs::Vector3& min,
          geometry_msgs::Vector3& max);
      void draw_segmentation(sensor_msgs::PointCloud& cloud);

      void read_classes_file(void);

      public:
      ros::NodeHandle m_nh;
      Object m_target_object;
      ros::Publisher m_global_goal_pub;

      private:
      ros::NodeHandle m_pvt_nh;

      image_transport::Subscriber m_image_sub;
      ros::Subscriber m_pointcloud_sub;
      
      image_transport::Publisher m_image_pub;
      ros::Publisher m_marker_pub;
      ros::Publisher m_targets_pub;

      tf2_ros::Buffer m_tfBuffer;
      tf2_ros::TransformListener *m_tfListener;
      sensor_msgs::PointCloud m_depth;
      std::vector<std::string> m_classes;
      cv::dnn::Net m_net_model;

      //Parameter variables
      double m_confidence;
      double m_sensor_range;
      double m_new_object_distance;
      int m_image_width;
      int m_image_height;
      bool m_draw_objects;
      bool m_draw_segmentation;

      std::string m_camera_topic;
      std::string m_pointcloud_topic;
      std::string m_model;
      std::string m_config;
      std::string m_interface;
      std::string m_classes_file;
      std::string m_frame_id;
      std::string m_base_frame_id;
  };
}
// struct Waiting : boost::statechart::simple_state<Waiting, ObjectDetection>
// {
//     Waiting()
//     {
//         ROS_INFO("Waiting for object to be detected");
//     }

//     ~Waiting(){}

//     typedef boost::statechart::transition<EvObjectDetected, Detected> reactions;
// };

// struct Detected : boost::statechart::state<Detected, ObjectDetection>
// {
//     Detected(my_context ctx) : my_base(ctx)
//     {
//         //Get current object
//         Object target = context<ObjectDetection>().m_target_object;
//         ROS_INFO("Object %s detected", target.name.c_str());

//         //Has this object been seen before?

//         //Create waypoints around the normal of the object
//         dronenav_msgs::PathGoal object_goals[4];
//         for(int i = 0; i < 4; i++)
//         {
//             object_goals[i].x = target.normals[i].x();
//             object_goals[i].y = target.normals[i].y();
//             object_goals[i].z = target.normals[i].z();

//             tf2::Vector3 front_yaw(
//                 -target.normals[i].x(),
//                 -target.normals[i].y(),
//                 -target.normals[i].z());

//             object_goals[i].yaw = target.normals[i].angle(front_yaw);
//             //TODO: attach priority to objects
//             object_goals[i].priority = 1;

//             context<ObjectDetection>().m_global_goal_pub.publish(object_goals[i]);
//         }

//         m_waypoint_count = 0;
//         m_waypoint_reached_sub = context<ObjectDetection>().m_nh.subscribe<
//             dronenav_msgs::Waypoint>("skipper/waypoint_reached", 50,
//             &Detected::waypoint_reached_callback, this);
//     }

//     ~Detected()
//     {
//         ROS_INFO("Detected STATE EXIT");
//     }
    
//     void waypoint_reached_callback(const dronenav_msgs::Waypoint::ConstPtr& msg)
//     {
//         if((*msg).flags & dronenav_msgs::Waypoint::PATH_LAST_FLAG)
//         {
//             m_waypoint_count++;
//             if(m_waypoint_count >= 4)
//             {
//                 context<ObjectDetection>().process_event(EvObjectInspected());
//             }
//         }
//     }

//     typedef boost::statechart::transition<EvObjectInspected, Waiting> reactions;

//     private:
//     ros::Subscriber m_waypoint_reached_sub;
//     int m_waypoint_count;
// };

#endif