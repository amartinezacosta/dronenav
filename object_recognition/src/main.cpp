#include <ros/ros.h>

#include <waiting.hpp>
#include <detected.hpp>
#include <object_recognition.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_recognition_node");
  ros::NodeHandle nh;
  ros::NodeHandle pvt_nh("~");

  object_recognition::ObjectDetection detection(nh, pvt_nh);
  detection.initiate();

  ros::spin();

  return 0;
}