#include <ros/ros.h>

#include <dronenav.hpp>
#include <navigation.hpp>
#include <landed.hpp>
#include <flying.hpp>
#include <hovering.hpp>
#include <moving.hpp>
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