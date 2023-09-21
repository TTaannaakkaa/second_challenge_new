/**
 * @file local_map_creator.h
 * @author Takuma Tanaka
 * @brief header file for local_map_creator class
 * @version 0.1
 * @date 2023-09-12
 * 
 */

#ifndef LOCAL_MAP_CREATOR_H
#define LOCAL_MAP_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

class LocalMapCreator
{
public:
    LocalMapCreator();
    void process();

private:

    void obs_poses_callback(const geometry_msgs::PoseArrayConstPtr& msg);

    bool is_in_local_map(const double x, const double y);
    int get_glid_index(const double x, const double y);
    int xy_to_index(const double x, const double y);

    void init_local_map();
    void update_local_map();

    int hz_;
    double local_map_size_;
    double local_map_resolution_;

    bool is_get_obs_poses_ = false;
    

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_obs_poses_;

    ros::Publisher pub_local_map_;

    nav_msgs::OccupancyGrid local_map_;
    geometry_msgs::PoseArray obs_poses_;

};

#endif // LOCAL_MAP_CREATOR_H