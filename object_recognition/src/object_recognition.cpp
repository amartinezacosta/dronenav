#include <object_recognition.hpp>
#include <waiting.hpp>

namespace object_recognition
{
  ObjectDetection::ObjectDetection(ros::NodeHandle nh, ros::NodeHandle pvt_nh)
      : m_nh(nh),
        m_pvt_nh(pvt_nh)
  {
      //Get paramaters
      m_pvt_nh.param<std::string>("image_topic", m_camera_topic, "camera/rgb/image_raw");
      m_pvt_nh.param<std::string>("pointcloud_topic", m_pointcloud_topic, "camera/depth/points");
      m_pvt_nh.param<std::string>("model", m_model, "");
      m_pvt_nh.param<std::string>("config", m_config, "");
      m_pvt_nh.param<std::string>("interface", m_interface, "Tensorflow");
      m_pvt_nh.param<std::string>("classes", m_classes_file, "");
      m_pvt_nh.param<std::string>("frame_id", m_frame_id, "map");
      m_pvt_nh.param<std::string>("base_frame_id", m_base_frame_id, "base_link");
      m_pvt_nh.param("confidence", m_confidence, 0.4);
      m_pvt_nh.param("image_width", m_image_width, 640);
      m_pvt_nh.param("image_height", m_image_height, 480);
      m_pvt_nh.param("sensor_range", m_sensor_range, 6.0);
      m_pvt_nh.param("draw_objects", m_draw_objects, false);
      m_pvt_nh.param("draw_segmentation", m_draw_segmentation, false);

      image_transport::ImageTransport it(m_nh);

      //Initalize opencv deep neural net model
      m_net_model = cv::dnn::readNet(m_model, m_config, m_interface);
      read_classes_file();

      //Publishers
      m_marker_pub = m_nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
      m_global_goal_pub = m_nh.advertise<dronenav_msgs::PathGoal>("dronenav_msgs/global_goal", 50);
      m_image_pub = it.advertise("dronenav_msgs/net_image", 1);

      //Subscribers
      m_pointcloud_sub = m_nh.subscribe(m_pointcloud_topic, 10, &ObjectDetection::pointcloud_callback, this);
      m_image_sub = it.subscribe(m_camera_topic, 1, &ObjectDetection::image_callback, this);

      m_tfListener = new tf2_ros::TransformListener(m_tfBuffer, m_nh);    
  }

  void ObjectDetection::image_callback(const sensor_msgs::ImageConstPtr &msg)
  {
      cv_bridge::CvImagePtr cv_ptr;
      try 
      {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      }
      catch(cv_bridge::Exception &e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      detect_objects(cv_ptr);
  }

  void ObjectDetection::object_bounding_box(std::string& class_name, 
      int u, int v, int width, int height)
  {   
      //Segment point cloud
      sensor_msgs::PointCloud segmented;
      segment_pointcloud(segmented, u, v, width, height);

      if(!segmented.points.empty())
      {
          //Get min max points
          pcl::PointXYZ min, max;
          pointcloud_minmax(segmented, min, max);

          //Transform points
          geometry_msgs::Vector3 min_transformed, max_transformed;
          transform_minmax(min, max, 
              min_transformed, max_transformed);    
      
          //Check if points lie on the same plane
          if( (min_transformed.x != max_transformed.x) || 
              (min_transformed.y != max_transformed.y) ||
              (min_transformed.z != max_transformed.z) )
          {
              //Build bounding box and normals    
              m_target_object = construct_object_box(min_transformed, 
                  max_transformed, class_name);

              //Post event
              process_event(EvObjectDetected());
          }
          else
          {
              ROS_WARN("Bounding box flat on at least one dimension");
          }
      }
      else
      {
          ROS_INFO("Object out of depth sensor range");
      }
  }

  void ObjectDetection::segment_pointcloud(sensor_msgs::PointCloud& output, 
      int u, int v, int width, int height)
  {

      for(int i = u; i < (u + width); i++)
      {
          for(int j = v; j < (v + height); j++)
          {
              geometry_msgs::Point32 p;
              //Actual equation is i+j*image_width
              int index = i+j*m_image_width;
              if(index > (m_depth.points.size() - 1)) return;

              p.x = m_depth.points[index].x;
              p.y = m_depth.points[index].y;
              p.z = m_depth.points[index].z;

              if( (p.z < m_sensor_range)) 
                  output.points.push_back(p);
          }
      }

      //Visualize pointcloud segmentation
      if(m_draw_segmentation) draw_segmentation(output);
  }

  void ObjectDetection::transform_minmax(pcl::PointXYZ& min_in, pcl::PointXYZ& max_in,
      geometry_msgs::Vector3& min_out, geometry_msgs::Vector3& max_out)
  {
      geometry_msgs::TransformStamped transform;
      try
      {
          transform = m_tfBuffer.lookupTransform(m_frame_id, 
              m_base_frame_id, 
              ros::Time(0));
      }
      catch(tf2::TransformException &ex)
      {
          ROS_WARN("Could not find a transform between %s and %s",
              m_frame_id.c_str(), m_base_frame_id.c_str());
          return;
      }

      geometry_msgs::Vector3 vmax_in, vmin_in;
      vmin_in.x = min_in.x;
      vmin_in.y = min_in.y;
      vmin_in.z = min_in.z;
      
      vmax_in.x = max_in.x;
      vmax_in.y = max_in.y;
      vmax_in.z = max_in.z;

      tf2::doTransform(vmin_in, min_out, transform);
      tf2::doTransform(vmax_in, max_out, transform);

      min_out.x += transform.transform.translation.x;
      min_out.y += transform.transform.translation.y;
      min_out.z += transform.transform.translation.z;

      max_out.x += transform.transform.translation.x;
      max_out.y += transform.transform.translation.y;
      max_out.z += transform.transform.translation.z;
  }

  void ObjectDetection::pointcloud_minmax(sensor_msgs::PointCloud& cloud_in, 
      pcl::PointXYZ& min, pcl::PointXYZ& max)
  {
      //Get min and max 3d points
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      for(geometry_msgs::Point32& point : cloud_in.points)
      {
          pcl::PointXYZ p(point.x, point.y, point.z);
          pcl_cloud.push_back(p);
      }

      pcl::getMinMax3D(pcl_cloud, min, max);
  }

  Object ObjectDetection::construct_object_box(geometry_msgs::Vector3& min, 
      geometry_msgs::Vector3& max, std::string class_name)
  {
      Object object;

      //Object class name
      object.name = class_name;

      //Object box vertices from min max point cloud points
      object.vertices[0].x = min.x;
      object.vertices[0].y = min.y;
      object.vertices[0].z = min.z;

      object.vertices[1].x = min.x;
      object.vertices[1].y = min.y;
      object.vertices[1].z = max.z;

      object.vertices[2].x = max.x;
      object.vertices[2].y = min.y;
      object.vertices[2].z = max.z;

      object.vertices[3].x = max.x;
      object.vertices[3].y = min.y;
      object.vertices[3].z = min.z;

      object.vertices[4].x = min.x;
      object.vertices[4].y = max.y;
      object.vertices[4].z = min.z;

      object.vertices[5].x = min.x;
      object.vertices[5].y = max.y;
      object.vertices[5].z = max.z;

      object.vertices[6].x = max.x;
      object.vertices[6].y = max.y;
      object.vertices[6].z = max.z;

      object.vertices[7].x = max.x;
      object.vertices[7].y = max.y;
      object.vertices[7].z = min.z;

      //Computer center of the object
      object.cx = (max.x + min.x) / 2;
      object.cy = (max.y + min.y) / 2;
      object.cz = (max.z + min.z) / 2;

      //Computer normals of front, back, left and right planes
      //Front face normal
      //front face normal
      tf2::Vector3 p(
          object.vertices[1].x - object.vertices[0].x, 
          object.vertices[1].y - object.vertices[0].y,
          object.vertices[1].z - object.vertices[0].z);
      
      tf2::Vector3 q(
          object.vertices[3].x - object.vertices[0].x, 
          object.vertices[3].y - object.vertices[0].y,
          object.vertices[3].z - object.vertices[0].z);

      object.normals[0] = p.cross(q);
      object.normals[0].normalize();

      //Left face normal
      tf2::Vector3 r(
          object.vertices[5].x - object.vertices[4].x,
          object.vertices[5].y - object.vertices[4].y,
          object.vertices[5].z - object.vertices[4].z);

      tf2::Vector3 s(
          object.vertices[0].x - object.vertices[4].x,
          object.vertices[0].y - object.vertices[4].y,
          object.vertices[0].z - object.vertices[4].z);

      object.normals[1] = r.cross(s);
      object.normals[1].normalize();

      //Back face normal
      tf2::Vector3 t(
          object.vertices[5].x - object.vertices[4].x,
          object.vertices[5].y - object.vertices[4].y,
          object.vertices[5].z - object.vertices[4].z);

      tf2::Vector3 u(
          object.vertices[7].x - object.vertices[4].x,
          object.vertices[7].y - object.vertices[4].y,
          object.vertices[7].z - object.vertices[4].z);

      object.normals[2] = u.cross(t);
      object.normals[2].normalize();

      //Right side normal
      tf2::Vector3 v(
          object.vertices[6].x - object.vertices[7].x,
          object.vertices[6].y - object.vertices[7].y,
          object.vertices[6].z - object.vertices[7].z);

      tf2::Vector3 w(
          object.vertices[3].x - object.vertices[7].x,
          object.vertices[3].y - object.vertices[7].y,
          object.vertices[3].z - object.vertices[7].z);

      object.normals[3] = w.cross(v);
      object.normals[3].normalize();

      //Draw object output
      draw_object(object);

      return object;
  }

  void ObjectDetection::detect_objects(cv_bridge::CvImagePtr cv_ptr)
  {
      cv::Mat frame = cv_ptr->image;
      cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(300, 300),
                      cv::Scalar(127.5, 127.5, 127.5), true, false);

      m_net_model.setInput(blob);
      cv::Mat output = m_net_model.forward();
      cv::Mat detection(output.size[2], output.size[3],
                          CV_32F, output.ptr<float>());
      
      for(int i = 0; i < detection.rows; i++)
      {
          int class_id = static_cast<int>(detection.at<float>(i, 1));
          float confidence = detection.at<float>(i, 2);

          if(confidence > m_confidence)
          {
              int u = static_cast<int>(detection.at<float>(i, 3)*frame.cols);
              int v = static_cast<int>(detection.at<float>(i, 4)*frame.rows);

              int width = static_cast<int>(detection.at<float>(i, 5)*frame.cols - u);
              int height = static_cast<int>(detection.at<float>(i, 6)*frame.rows - v);

              std::string class_name = " ";
              if(!m_classes.empty())
              {
                  class_name = m_classes[class_id-1];
              }
              
              //Create a bounding box around this object
              object_bounding_box(class_name, u, v, width, height);
              
              //Image output for debugging
              cv::rectangle(frame, cv::Point(u, v), cv::Point(u+width, v+height),
                          cv::Scalar(255, 255, 255), 2);
              cv::putText(frame, class_name, cv::Point(u, v-5), cv::FONT_HERSHEY_SIMPLEX,
                          1.0, cv::Scalar(0, 255, 255), 1);
              
          }
      }

      m_image_pub.publish(cv_ptr->toImageMsg());
  }

  void ObjectDetection::read_classes_file(void)
  {
      std::fstream file(m_classes_file);

      if(file.is_open())
      {
          ROS_INFO("Reading class labels: %s", m_classes_file.c_str());
          std::string line;
          while(std::getline(file, line))
          {
              m_classes.push_back(line);
          }
      }
      else
      {
          ROS_WARN("Could not open file: %s. Class labels will not be displayed",  
              m_classes_file.c_str());
      }

      file.close();
  }

  void ObjectDetection::draw_object(Object& object)
  {
      //Object bounding box------------
      visualization_msgs::Marker box;
      box.header.frame_id = "map";
      box.header.stamp = ros::Time();
      box.ns = "object_box";
      box.id = 21;
      box.type = visualization_msgs::Marker::LINE_LIST;
      box.action = visualization_msgs::Marker::ADD;   

      box.pose.orientation.w = 1.0;
      box.scale.x = 0.05;
      box.scale.y = 0.05;
      box.scale.z = 0.05;
      box.color.a = 1.0;
      box.color.r = 0.0;
      box.color.g = 1.0;
      box.color.b = 0.0;

      //Front face
      box.points.push_back(object.vertices[0]);
      box.points.push_back(object.vertices[1]);

      box.points.push_back(object.vertices[1]);
      box.points.push_back(object.vertices[2]);

      box.points.push_back(object.vertices[2]);
      box.points.push_back(object.vertices[3]);

      box.points.push_back(object.vertices[3]);
      box.points.push_back(object.vertices[0]);

      box.points.push_back(object.vertices[1]);
      box.points.push_back(object.vertices[3]);

      //Top face
      box.points.push_back(object.vertices[5]);
      box.points.push_back(object.vertices[1]);

      box.points.push_back(object.vertices[6]);
      box.points.push_back(object.vertices[2]);

      //Back face
      box.points.push_back(object.vertices[4]);
      box.points.push_back(object.vertices[5]);

      box.points.push_back(object.vertices[5]);
      box.points.push_back(object.vertices[6]);

      box.points.push_back(object.vertices[6]);
      box.points.push_back(object.vertices[7]);

      box.points.push_back(object.vertices[7]);
      box.points.push_back(object.vertices[4]);

      box.points.push_back(object.vertices[5]);
      box.points.push_back(object.vertices[7]);

      //Bottom face
      box.points.push_back(object.vertices[4]);
      box.points.push_back(object.vertices[0]);

      box.points.push_back(object.vertices[7]);
      box.points.push_back(object.vertices[3]);

      //Center of the object-------------------
      visualization_msgs::Marker center;
      center.header.frame_id = "map";
      center.header.stamp = ros::Time();
      center.ns = "object_center";
      center.id = 22;
      center.type = visualization_msgs::Marker::SPHERE;
      center.action = visualization_msgs::Marker::ADD;   

      center.pose.position.x = object.cx;
      center.pose.position.y = object.cy;
      center.pose.position.z = object.cz;
      center.pose.orientation.w = 1.0;
      center.scale.x = 0.5;
      center.scale.y = 0.5;
      center.scale.z = 0.5;
      center.color.b = 0.0;

      //Object normals-------------------
      visualization_msgs::Marker normals;
      normals.header.frame_id = "map";
      normals.header.stamp = ros::Time();
      normals.ns = "object_normals";
      normals.id = 23;
      normals.type = visualization_msgs::Marker::LINE_LIST;
      normals.action = visualization_msgs::Marker::ADD;   

      normals.pose.orientation.w = 1.0;
      normals.scale.x = 0.05;
      normals.scale.y = 0.05;
      normals.scale.z = 0.05;
      normals.color.a = 1.0;
      normals.color.r = 0.0;
      normals.color.g = 1.0;
      normals.color.b = 1.0;

      geometry_msgs::Point p1, p2;
      double x_min = object.vertices[0].x;
      double y_min = object.vertices[0].y;

      double x_max = object.vertices[6].x;
      double y_max = object.vertices[6].y;

      //Front face normal
      p1.x = object.cx;
      p1.y = y_min;
      p1.z = object.cz;

      p2.x = p1.x + 1.5*object.normals[0].x();
      p2.y = p1.y + 1.5*object.normals[0].y();
      p2.z = p1.z + 1.5*object.normals[0].z();

      normals.points.push_back(p1);
      normals.points.push_back(p2);

      //Left side normal
      p1.x = x_min;
      p1.y = object.cy;
      p1.z = object.cz;

      p2.x = p1.x + 1.5*object.normals[1].x();
      p2.y = p1.y + 1.5*object.normals[1].y();
      p2.z = p1.z + 1.5*object.normals[1].z();

      normals.points.push_back(p1);
      normals.points.push_back(p2);

      //Back face normal
      p1.x = object.cx;
      p1.y = y_max;
      p1.z = object.cz;

      p2.x = p1.x + 1.5*object.normals[2].x();
      p2.y = p1.y + 1.5*object.normals[2].y();
      p2.z = p1.z + 1.5*object.normals[2].z();

      normals.points.push_back(p1);
      normals.points.push_back(p2);

      //Right side normal
      p1.x = x_max;
      p1.y = object.cy;
      p1.z = object.cz;

      p2.x = p1.x + 1.5*object.normals[3].x();
      p2.y = p1.y + 1.5*object.normals[3].y();
      p2.z = p1.z + 1.5*object.normals[3].z();

      normals.points.push_back(p1);
      normals.points.push_back(p2);

      //Publish markers
      m_marker_pub.publish(normals);
      m_marker_pub.publish(box);
      m_marker_pub.publish(center);
  }

  void ObjectDetection::draw_minmax(geometry_msgs::Vector3& min,
      geometry_msgs::Vector3& max)
  {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "minmax_object";
      marker.id = 22;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::Marker::ADD;   

      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      
      geometry_msgs::Point p1, p2;
      p1.x = min.x;
      p1.y = min.y;
      p1.z = min.z;

      p2.x = max.x;
      p2.y = max.y;
      p2.z = max.z;

      m_marker_pub.publish(marker);
  }

  void ObjectDetection::draw_segmentation(sensor_msgs::PointCloud& cloud)
  {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "segmentation";
      marker.id = 20;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::Marker::ADD;   

      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      for(geometry_msgs::Point32 point : cloud.points)
      {
          geometry_msgs::Point p;
          p.x = point.x;
          p.y = point.y;
          p.z = point.z;
          marker.points.push_back(p);
      }

      m_marker_pub.publish(marker);
  }

}