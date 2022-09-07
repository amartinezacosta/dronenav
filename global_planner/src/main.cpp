#include "global_planner.hpp"
#include "waiting.hpp"
#include "planning.hpp"
#include "sending.hpp"
#include "active.hpp"
#include "interrupt.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pvt_nh("~");

    global_planner::GlobalPlanner planner(nh, pvt_nh);
    planner.initiate();
    ros::spin();

    return 0;
}
