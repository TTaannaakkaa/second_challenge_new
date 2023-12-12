#include "local_path_planner/local_path_planner.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_path_planner");
    DWA dwa_planner;
    dwa_planner.process();
    return 0;
}