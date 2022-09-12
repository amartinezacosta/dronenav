#include <ros/ros.h>

#include <dronenav.hpp>
#include <navigation.hpp>
#include <landed_touchdown.hpp>
#include <landed.hpp>
#include <takeoff_position.hpp>
#include <flying.hpp>
#include <hovering.hpp>
#include <positioning.hpp>
#include <reached.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "skipper");
    ros::NodeHandle nh;
    ros::NodeHandle pvt_nh("~");

    dronenav::Drone drone(nh, pvt_nh);
    drone.initiate();
    drone.connect();

    ros::spin();
}