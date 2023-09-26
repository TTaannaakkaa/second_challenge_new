/**
 * @brief main function 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */

#include "local_map_creator/local_map_creator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map_creator");
    LocalMapCreator localmap_creator;
    localmap_creator.process();
    
    return 0;
}