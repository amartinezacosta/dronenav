#include <sort.hpp>

namespace qr_tracking
{
  Sort::Sort() : 
    m_distance_threshold(10.0),
    m_missed_max_threshold(10),
    m_hits_min_threshold(10),
    m_sort_id(0)
  {

  }

  Sort::~Sort()
  {

  }

  void Sort::euclidean_cost_matrix(std::vector<std::vector<double>>& cost_matrix,
    std::vector<dronenav_msgs::Code>& detections,
    std::vector<dronenav_msgs::Code>& trackers)
  {
    cost_matrix.resize(detections.size(),
      std::vector<double>(trackers.size()));

    for(int i = 0; i < detections.size(); i++)
    {
      for(int j = 0; j < trackers.size(); j++)
      {
        cost_matrix[i][j] = euclidean_distance(detections[i], trackers[j]);
      }
    }
  }

  double Sort::euclidean_distance(dronenav_msgs::Code& c1, 
    dronenav_msgs::Code& c2)
  {
    double dx = c2.position.x - c1.position.x;
    double dy = c2.position.y - c1.position.y;
    double dz = c2.position.z - c1.position.z;

    return (sqrt(dx*dx + dy*dy + dz*dz));
  }

  // double QRTracking::boxes_iou(dronenav_msgs::Code &box_a, 
  //   dronenav_msgs::Code &box_b)
  // {
  //   if((box_a.corners.size() != 4) || 
  //      (box_b.corners.size() != 4))
  //     return 0.0f;

  //   double x_a = MAX(box_a.corners[0].x, box_b.corners[0].x);
  //   double y_a = MAX(box_a.corners[0].y, box_b.corners[0].y);
  //   double x_b = MAX(box_a.corners[2].x, box_b.corners[2].x);
  //   double y_b = MAX(box_a.corners[2].y, box_b.corners[2].y);

  //   double inter_area = (x_b - x_a) * (y_b - y_a);
  //   double box_a_area = (box_a.corners[2].x - box_a.corners[0].x) * 
  //                       (box_a.corners[2].y - box_a.corners[0].y);
  //   double box_b_area = (box_b.corners[2].x - box_b.corners[0].x) * 
  //                       (box_b.corners[2].y - box_b.corners[0].y);

  //   return (inter_area / (box_a_area + box_b_area - inter_area));
  // }

  double Sort::linear_assignment(std::vector<std::vector<double>>& cost_matrix,
    std::vector<int>& assignments)
  {
    HungarianAlgorithm hungarian;
    return hungarian.Solve(cost_matrix, assignments);
  }

  void Sort::associate_detections_to_trackers(
    std::vector<dronenav_msgs::Code>& detections,
    std::vector<dronenav_msgs::Code>& trackers,
    std::map<int,dronenav_msgs::Code>& matched_detections,
    std::vector<dronenav_msgs::Code>& unmatched_detections)
  {
    /*Construct Euclidean distance cost matrix*/
    std::vector<std::vector<double>> cost_matrix;
    euclidean_cost_matrix(cost_matrix, detections, trackers);
  
    /*Linear assignment using the hungarian algorithm*/
    std::vector<int> assignments;
    linear_assignment(cost_matrix, assignments);

    /*Fill out matched and unmatched detections from assignments*/
    for(int i = 0; i < cost_matrix.size(); i++)
    {
      /*Detection was not assigned and object was further than the threshold*/
      if(assignments[i] < 0)
      {
        unmatched_detections.push_back(detections[i]);
      }
      else
      {
        /*Filter out matches that are too far away*/
        if(cost_matrix[i][assignments[i]] < m_distance_threshold)
          matched_detections[assignments[i]] = detections[i];
      }
    }
  }

  void Sort::update(std::vector<dronenav_msgs::Code>& detections,
    std::vector<dronenav_msgs::Code>& trackers)
  {
    std::map<int,dronenav_msgs::Code> matched_detections;
    std::vector<dronenav_msgs::Code> unmatched_detections;

    /*Predict*/
    for(dronenav_msgs::Code& track : m_tracked_codes)
    {
      if(track.time_since_update > 0)
      {
        track.hit_streak = 0;
      }
      track.time_since_update++;
    }

    /*Compute detection to tracker associations*/
    if(!detections.empty())
    {
      associate_detections_to_trackers(detections,
        m_tracked_codes,
        matched_detections,
        unmatched_detections);
    }

    /*Update trackers with matched detections*/
    for(auto& match : matched_detections)
    {
      int id = match.first;
      /*Update marker*/
      m_tracked_codes[id].position = match.second.position;
      m_tracked_codes[id].corners = match.second.corners;
      m_tracked_codes[id].u = match.second.u;
      m_tracked_codes[id].v = match.second.v;

      m_tracked_codes[id].hit_streak++;
      m_tracked_codes[id].time_since_update = 0;
    }

    /*Create and initialize new trackers*/
    for(dronenav_msgs::Code& detection : unmatched_detections)
    {
      detection.id = m_sort_id++;
      m_tracked_codes.push_back(detection);
    }

    /*We have missed n amount of frames AND we did not accumulate enough hits
    to keep this tracker. Delete it, most likely a false positive*/
    for(std::vector<dronenav_msgs::Code>::iterator it = m_tracked_codes.begin();
      it != m_tracked_codes.end();)
    {
      /*We have not update this tracker in a while, delete it*/
      if(it->time_since_update > m_missed_max_threshold)
      {
        it = m_tracked_codes.erase(it);
      }
      else
      {
        it++;
      }
    }

    for(dronenav_msgs::Code& track : m_tracked_codes)
    {
      if((track.time_since_update < m_missed_max_threshold) &&
         (track.hit_streak > m_hits_min_threshold))
      {
        /*This tracks should be stored globally and be persistent*/
        trackers.push_back(track);
      }
    }
  }
}