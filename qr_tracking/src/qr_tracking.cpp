#include <qr_tracking.hpp>

namespace qr_tracking
{
  QRTracking::QRTracking(ros::NodeHandle nh, ros::NodeHandle pvt_nh) : 
    m_nh(nh),
    m_pvt_nh(pvt_nh)
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

    /*Sort configure*/
    m_sort.set_distance_threshold(0.1);
    m_sort.set_hits_min_threshold(20);
    m_sort.set_missed_max_threshold(20);

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
    std::vector<dronenav_msgs::Code> current_detections;
    detect_codes(cv_ptr, current_detections);
  
    /*Transform point*/
    transform_code_coordinates(current_detections);

    /*Track codes*/
    m_tracking_objects.clear();
    m_sort.update(current_detections, 
        m_tracking_objects);

    /*Publish tracked codes*/
    publish_tracked_codes();
    /*Show map markers*/
    show_map_markers();
  }

  void QRTracking::detect_codes(cv_bridge::CvImagePtr& cv_ptr, 
    std::vector<dronenav_msgs::Code>& detections)
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
      dronenav_msgs::Code detected;
      detected.type = symbol->get_type_name();
      detected.data = symbol->get_data();

      for(int i = 0; i < symbol->get_location_size(); i++)
      {
        detected.u.push_back(symbol->get_location_x(i));
        detected.v.push_back(symbol->get_location_y(i));
      }

      /*Resize corners vector*/
      detected.corners.resize(detected.u.size());

      /*Save detection*/
      detections.push_back(detected);
    }

    if(m_show_detections) { show_detections(cv_ptr, detections); }
  }

  void QRTracking::transform_code_coordinates(
    std::vector<dronenav_msgs::Code>& detections)
  {
    /*Get the current transformation between frames*/
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
      return;
    }

    /*For every detection*/
    for(dronenav_msgs::Code& detection : detections)
    {
      detection.position.x = 0.0f;
      detection.position.y = 0.0f;
      detection.position.z = 0.0f;

      /*For every corner point*/
      for(int i = 0; i < detection.u.size(); i++)
      {
        uint32_t u = detection.u[i];
        uint32_t v = detection.v[i];

        int index = u + v*m_image_width;
        if(index > (m_depth.points.size() - 1)) return;

        geometry_msgs::Vector3 v_in;
        v_in.x = m_depth.points[index].x;
        v_in.y = m_depth.points[index].y;
        v_in.z = m_depth.points[index].z;

        /*Transform point*/
        tf2::doTransform(v_in, detection.corners[i], transform);
        detection.corners[i].x += transform.transform.translation.x;
        detection.corners[i].y += transform.transform.translation.y;
        detection.corners[i].z += transform.transform.translation.z;

        detection.position.x += detection.corners[i].x;
        detection.position.y += detection.corners[i].y;
        detection.position.z += detection.corners[i].z;
      }

      /*Get the center point of the detection*/
      detection.position.x /= detection.corners.size();
      detection.position.y /= detection.corners.size();
      detection.position.z /= detection.corners.size();
    }

    /*Visualize in rviz*/
    if(m_show_detection_markers && !detections.empty())
    { 
      show_detection_surfaces(detections);
    }
  }

  bool QRTracking::codes_equal(dronenav_msgs::Code& c1, 
    dronenav_msgs::Code& c2)
  {
    double dx = c2.position.x - c1.position.x;
    double dy = c2.position.y - c1.position.y;
    double dz = c2.position.z - c1.position.z;
  
    double dl = dx*dx + dy*dy + dz*dz;

    return ((dl < 10.0) ? true : false);
  }

  void QRTracking::publish_tracked_codes(void)
  {
    dronenav_msgs::TrackedCodes tracked;
    for(dronenav_msgs::Code& code : m_tracking_objects)
    {
      tracked.codes.push_back(code);
    }

    m_tracked_codes_pub.publish(tracked);
  }

  void QRTracking::show_detections(cv_bridge::CvImagePtr& cv_ptr, 
    std::vector<dronenav_msgs::Code>& detections)
  {
    for(dronenav_msgs::Code detection : detections)
    {
      int n = detection.u.size() - 1;
      cv::Point p1(detection.u[0], 
                   detection.v[0]);

      cv::Point p2(detection.u[n],
                   detection.v[n]);

      cv::line(cv_ptr->image, p1, p2, cv::Scalar(0, 255, 0), 3);

      for(int i = 0; i < n; i++)
      {
        cv::Point p1(detection.u[i], 
                     detection.v[i]);

        cv::Point p2(detection.u[i+1],
                     detection.v[i+1]);

        cv::line(cv_ptr->image, p1, p2, cv::Scalar(0, 255, 0), 3);
      }

      cv::putText(cv_ptr->image, detection.data, 
        cv::Point(detection.u[0] - 10, detection.v[0] - 10), 
        cv::FONT_HERSHEY_SIMPLEX,
        1.0, cv::Scalar(0, 255, 00), 1); 
    }

    m_image_pub.publish(cv_ptr->toImageMsg());
  }

  void QRTracking::show_detection_surfaces(
    std::vector<dronenav_msgs::Code>& detections)
  {
    /*Surface detection rectangular marker*/
    visualization_msgs::Marker surface;
    surface.header.frame_id = "map";
    surface.header.stamp = ros::Time();
    surface.ns = "qr_tracking";
    surface.id = 0;
    surface.lifetime = ros::Duration(0.5);
    surface.type = visualization_msgs::Marker::LINE_LIST;
    surface.action = visualization_msgs::Marker::ADD;   

    surface.pose.orientation.w = 1.0;
    surface.scale.x = 0.1;
    surface.scale.y = 0.1;
    surface.scale.z = 0.1;
    surface.color.a = 1.0;
    surface.color.r = 1.0;
    surface.color.g = 0.0;
    surface.color.b = 0.0;

    /*Center point of detection surface*/
    visualization_msgs::Marker center;
    center.header.frame_id = "map";
    center.header.stamp = ros::Time();
    center.ns = "qr_tracking";
    center.id = 1;
    center.lifetime = ros::Duration(0.5);
    center.type = visualization_msgs::Marker::SPHERE_LIST;
    center.action = visualization_msgs::Marker::ADD;   

    center.pose.orientation.w = 1.0;
    center.scale.x = 0.1;
    center.scale.y = 0.1;
    center.scale.z = 0.1;
    center.color.a = 1.0;
    center.color.r = 1.0;
    center.color.g = 0.0;
    center.color.b = 0.0;
    
    for(dronenav_msgs::Code& detection : detections)
    {
      /*First few lines*/
      geometry_msgs::Point p1, p2;
      for(int i = 0; i < detection.corners.size() - 1; i++)
      {
        p1.x = detection.corners[i].x;
        p1.y = detection.corners[i].y;
        p1.z = detection.corners[i].z;
      
        p2.x = detection.corners[i+1].x;
        p2.y = detection.corners[i+1].y;
        p2.z = detection.corners[i+1].z;

        surface.points.push_back(p1);
        surface.points.push_back(p2);
      }

      /*Line that closes the rectangle*/
      p1.x = detection.corners.front().x;
      p1.y = detection.corners.front().y;
      p1.z = detection.corners.front().z;
    
      p2.x = detection.corners.back().x;
      p2.y = detection.corners.back().y;
      p2.z = detection.corners.back().z;

      surface.points.push_back(p1);
      surface.points.push_back(p2);

      /*Center point*/
      center.points.push_back(detection.position);
    }    

    m_marker_pub.publish(surface);
    m_marker_pub.publish(center);
  }

  void QRTracking::show_map_markers(void)
  {
    visualization_msgs::Marker surface;
    surface.header.frame_id = "map";
    surface.header.stamp = ros::Time();
    surface.ns = "qr_tracking";
    surface.id = 2;
    surface.type = visualization_msgs::Marker::LINE_LIST;
    surface.action = visualization_msgs::Marker::ADD;   

    surface.pose.orientation.w = 1.0;
    surface.scale.x = 0.1;
    surface.scale.y = 0.1;
    surface.scale.z = 0.1;
    surface.color.a = 1.0;
    surface.color.r = 1.0;
    surface.color.g = 0.65;
    surface.color.b = 0.0;

    /*Center point of detection surface*/
    visualization_msgs::Marker center;
    center.header.frame_id = "map";
    center.header.stamp = ros::Time();
    center.ns = "qr_tracking";
    center.id = 3;
    center.type = visualization_msgs::Marker::SPHERE_LIST;
    center.action = visualization_msgs::Marker::ADD;   

    center.pose.orientation.w = 1.0;
    center.scale.x = 0.1;
    center.scale.y = 0.1;
    center.scale.z = 0.1;
    center.color.a = 1.0;
    center.color.r = 1.0;
    center.color.g = 0.65;
    center.color.b = 0.0;

    for(dronenav_msgs::Code& tracked : m_tracking_objects)
    {
      /*First few lines*/
      geometry_msgs::Point p1, p2;
      for(int i = 0; i < tracked.corners.size() - 1; i++)
      {
        p1.x = tracked.corners[i].x;
        p1.y = tracked.corners[i].y;
        p1.z = tracked.corners[i].z;
      
        p2.x = tracked.corners[i+1].x;
        p2.y = tracked.corners[i+1].y;
        p2.z = tracked.corners[i+1].z;

        surface.points.push_back(p1);
        surface.points.push_back(p2);
      }

      /*Line that closes the rectangle*/
      p1.x = tracked.corners.front().x;
      p1.y = tracked.corners.front().y;
      p1.z = tracked.corners.front().z;
    
      p2.x = tracked.corners.back().x;
      p2.y = tracked.corners.back().y;
      p2.z = tracked.corners.back().z;

      surface.points.push_back(p1);
      surface.points.push_back(p2);

      /*Center point*/
      center.points.push_back(tracked.position);
    }   

    if(!surface.points.empty()) m_marker_pub.publish(surface);
    if(!center.points.empty()) m_marker_pub.publish(center);
  }
}

