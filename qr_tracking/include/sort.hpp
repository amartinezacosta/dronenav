#ifndef SORT_HPP
#define SORT_HPP

#include <dronenav_msgs/Code.h>

#include "Hungarian.h"

namespace qr_tracking
{
  class Sort
  {
    public:
    Sort();
    ~Sort();

    void update(std::vector<dronenav_msgs::Code>& detections,
      std::vector<dronenav_msgs::Code>& trackers);

    void set_distance_threshold(double distance){ m_distance_threshold = distance; }
    void set_missed_max_threshold(int max){ m_missed_max_threshold = max; }
    void set_hits_min_threshold(int min){ m_hits_min_threshold = min; }

    private:
    void euclidean_cost_matrix(std::vector<std::vector<double>>& cost_matrix,
      std::vector<dronenav_msgs::Code>& detections,
      std::vector<dronenav_msgs::Code>& trackers);
    double euclidean_distance(dronenav_msgs::Code& c1, 
      dronenav_msgs::Code& c2);
    double linear_assignment(std::vector<std::vector<double>>& cost_matrix,
      std::vector<int>& assignments);
    void associate_detections_to_trackers(
      std::vector<dronenav_msgs::Code>& detections,
      std::vector<dronenav_msgs::Code>& trackers,
      std::map<int,dronenav_msgs::Code>& matched_detections,
      std::vector<dronenav_msgs::Code>& unmatched_detections);

    private:
    std::vector<dronenav_msgs::Code> m_tracked_codes;
    double m_distance_threshold;
    int m_missed_max_threshold;
    int m_hits_min_threshold;
    int m_sort_id;
  };
}
#endif