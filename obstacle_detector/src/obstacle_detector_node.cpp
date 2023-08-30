#include "obstacle_detector/obstacle_detector.h"

/**
 * @brief main function 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detector");
    ObstacleDetector obstacle_detector;
    obstacle_detector.process();
    
    return 0;
}