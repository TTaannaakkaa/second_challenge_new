#include "local_goal_creator/local_goal_creator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCrreator localgoalcreator;
    localgoalcreator.process();

    return 0;
}