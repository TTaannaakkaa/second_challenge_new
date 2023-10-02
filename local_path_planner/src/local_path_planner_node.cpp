#include "local_path_planner/local_path_planner.h"

int main(int argc, char** argv)
{
    ROS_WARN("local_path_planner is started");
    ros::init(argc, argv, "local_path_planner");
    DWAPlanner dwa_planner;
    dwa_planner.process();
    return 0;
}