#include <qr_tracking.hpp>

namespace qr_tracking
{
  QRTracking::QRTracking(ros::NodeHandle nh, ros::NodeHandle pvt_nh) : 
    m_nh(nh),
    m_pvt_nh(pvt_nh),
    m_frame_count(0),
    m_track_id(0)
  {
    /*Parameters*/
    m_pvt_nh.param<std::string>("image_topic", m_image_topic, "camera/rgb/image_raw");
    m_pvt_nh.param<std::string>("pointcloud_topic", m_pointcloud_topic, "camera/depth/points");
    m_pvt_nh.param<std::string>("frame_id", m_frame_id, "map");
    m_pvt_nh.param<std::string>("base_frame_id", m_base_frame_id, "camera_link");
    m_pvt_nh.param("image_width", m_image_width, 640);
    m_pvt_nh.param("show_detections", m_show_detections, true);
    m_pvt_nh.param("show_detection_markers", m_show_detection_markers, true);
    m_pvt_nh.param("show_map_markers", m_show_map_markers, true);

    /*Subscribers*/
    image_transport::ImageTransport it(m_nh);
    m_image_sub = it.subscribe(m_image_topic, 10, &QRTracking::image_callback, this);  
    m_pointcloud_sub = m_nh.subscribe(m_pointcloud_topic, 5, &QRTracking::pointcloud_callback, this);

    /*Publisher*/
    m_image_pub = it.advertise("dronenav/qr_tracker/image", 1);
    m_marker_pub = m_nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    m_tracked_codes_pub = m_nh.advertise<dronenav_msgs::TrackedCodes>("dronenav/qr_tracker/tracked", 10);

    /*zbar configuration*/
    m_scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

    /*TF2 listener*/
    m_tfListener = new tf2_ros::TransformListener(m_tfBuffer, m_nh);
  }

  QRTracking::~QRTracking()
  {
    delete m_tfListener;
  }

  void QRTracking::image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    /*Detection*/
    std::vector<Code> current_detections;
    detect_codes(cv_ptr, current_detections);
  
    /*Transform point*/
    transform_code_coordinates(current_detections);

    /*Tracking*/
    track_codes(current_detections);

    m_prev_detections = current_detections;
  }

  void QRTracking::detect_codes(cv_bridge::CvImagePtr& cv_ptr, 
    std::vector<Code>& detections)
  {
    cv::Mat image_gray;
    cv::cvtColor(cv_ptr->image, image_gray, cv::COLOR_BGR2GRAY);
    
    zbar::Image z_image(image_gray.cols, image_gray.rows, "Y800", 
      (uchar*)image_gray.data, image_gray.cols*image_gray.rows);
    m_scanner.scan(z_image);

    for(zbar::Image::SymbolIterator symbol = z_image.symbol_begin();
      symbol != z_image.symbol_end(); ++symbol)
    {
      /*Code was detected, extract information*/
      Code detected;
      detected.type = symbol->get_type_name();
      detected.data = symbol->get_data();

      for(int i = 0; i < symbol->get_location_size(); i++)
      {
        geometry_msgs::Point point;
        point.x = symbol->get_location_x(i);
        point.y = symbol->get_location_y(i);

        detected.cx += point.x;
        detected.cy += point.y;

        detected.points.push_back(point);
      }

      detected.cx /= detected.points.size();
      detected.cy /= detected.points.size();

      /*Save detection*/
      detections.push_back(detected);
    }

    if(m_show_detections) { show_detections(cv_ptr, detections); }
  }

  void QRTracking::transform_code_coordinates(std::vector<Code>& detections)
  {
    /*For every detection*/
    for(Code& detection : detections)
    {
      /*Find 3 dimensional coordinates from point cloud*/
      int index = detection.cx+detection.cy*m_image_width;
      if(index > (m_depth.points.size() - 1)) return;
    
      geometry_msgs::Vector3 v_in, v_out;
      v_in.x = m_depth.points[index].x;
      v_in.y = m_depth.points[index].y;
      v_in.z = m_depth.points[index].z;

      /*Transform coordinates from camera to map frame*/
      geometry_msgs::TransformStamped transform;
      try
      {
        transform = m_tfBuffer.lookupTransform(m_frame_id,
          m_base_frame_id, ros::Time(0));
      }
      catch(tf2::TransformException& ex)
      {
        ROS_WARN_ONCE_NAMED("qr_tracking", "Could not find a transform between %s and %s",
          m_frame_id.c_str(), m_base_frame_id.c_str());
        continue;
      }

      tf2::doTransform(v_in, v_out, transform);
      detection.x = v_out.x + transform.transform.translation.x;
      detection.y = v_out.y + transform.transform.translation.y;
      detection.z = v_out.z + transform.transform.translation.z;
    }

    /*Visualize in rviz*/
    if(m_show_detection_markers && !detections.empty())
    { 
      show_detection_markers(detections);
    }
  }

  void QRTracking::track_codes(std::vector<Code>& detections)
  {
    /*If the dictionary is empty, just add the codes and exit*/
    if(m_tracking_objects.empty())
    {
      for(Code& current : detections)
      {
        for(Code& previous : m_prev_detections)
        {
          if(current == previous)
          {
            m_tracking_objects[m_track_id++] = current;
          }
        }
      }      
    }
    /*There is stuff on the dictionary, make sure that we are not repeating data
    here*/
    else
    {
      for(Code& detection : detections)
      {
        int counter = 0;

        for(auto& track : m_tracking_objects)
        {
          /*If it is equal to any detection then ignore this match*/
          if(track.second == detection)
          {
            /*New object, add it to the dictionary*/
            continue;
          }
          /*Else if it is not equal, then add a counter*/
          else
          {
            counter++;
          }
        }

        /*If this detection does not belong to any track on the list, 
        then the counter should be equal to the number of tracked objects,
        which means that this object does not belong to any previously tracked 
        object. In other words, all tracked objects said no to this detection*/
        if(m_tracking_objects.size() == counter)
        {
          m_tracking_objects[m_track_id++] = detection;
        }
      }

      /*Publish tracked markers markers*/
      if(m_show_map_markers) show_map_markers();
    }

    /*Publish code markers message*/
    publish_tracked_codes();
  }

  void QRTracking::publish_tracked_codes(void)
  {
    dronenav_msgs::TrackedCodes tracked_codes;
    for(auto& tracked : m_tracking_objects)
    {
      dronenav_msgs::Code code;
      //Id
      code.id = tracked.first;

      //3D real world coordinates
      code.x = tracked.second.x;
      code.y = tracked.second.y;
      code.z = tracked.second.z;

      //Pixel coordinates
      code.cu = tracked.second.cx;
      code.cv = tracked.second.cy;
      for(geometry_msgs::Point point : tracked.second.points)
      {
        code.u.push_back(point.x);
        code.v.push_back(point.z);
      }

      code.data = tracked.second.data;
      code.type = tracked.second.type;
      
      tracked_codes.codes.push_back(code);
    }

    m_tracked_codes_pub.publish(tracked_codes);
  }

  void QRTracking::show_detections(cv_bridge::CvImagePtr& cv_ptr, 
    std::vector<Code>& detections)
  {
    for(Code detection : detections)
    {
      int n = detection.points.size() - 1;
      cv::Point p1(detection.points[0].x, 
                   detection.points[0].y);

      cv::Point p2(detection.points[n].x,
                   detection.points[n].y);

      cv::line(cv_ptr->image, p1, p2, cv::Scalar(0, 255, 0), 3);

      for(int i = 0; i < detection.points.size() - 1; i++)
      {
        cv::Point p1(detection.points[i].x, 
                     detection.points[i].y);

        cv::Point p2(detection.points[i+1].x,
                     detection.points[i+1].y);

        cv::line(cv_ptr->image, p1, p2, cv::Scalar(0, 255, 0), 3);
      }

      cv::Point origin(detection.cx, detection.cy);
      cv::Point text_origin(origin.x - 100, origin.y - 100);

      cv::circle(cv_ptr->image, origin, 10, cv::Scalar(255, 0, 0), -1);
      cv::putText(cv_ptr->image, detection.data, origin, cv::FONT_HERSHEY_SIMPLEX,
        1.0, cv::Scalar(0, 255, 00), 1); 
    }

    m_image_pub.publish(cv_ptr->toImageMsg());
  }

  void QRTracking::show_detection_markers(std::vector<Code>& detections)
  {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "qr_tracking";
      marker.id = 0;
      marker.lifetime = ros::Duration(0.5);
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::Marker::ADD;   

      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

    for(Code& detection : detections)
    {
      geometry_msgs::Point p;
      p.x = detection.x;
      p.y = detection.y;
      p.z = detection.z;
      
      marker.points.push_back(p);
    }

    m_marker_pub.publish(marker);
  }

  void QRTracking::show_map_markers(void)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "qr_tracking";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;   

    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    for(auto& tracked : m_tracking_objects)
    {
      geometry_msgs::Point p;
      p.x = tracked.second.x;
      p.y = tracked.second.y;
      p.z = tracked.second.z;

      marker.points.push_back(p);
    }

    m_marker_pub.publish(marker);
  }
}

