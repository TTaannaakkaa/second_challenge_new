#include "global_path_planner/global_path_planner.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "global_path_palnner");
    Astar astar;
    astar.process();

    return 0;
}