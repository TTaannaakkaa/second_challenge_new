/**
 * @file local_goal_creator.cpp
 * @author Takuma Tanaka
 * @brief  This file is the source file for the local_goal_creator class.
 * @version 0.1
 * @date 2023-09-12
 * 
 */

#include "local_map_creator/local_map_creator.h"

LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("map_size", local_map_size_);
    private_nh_.getParam("map_reso", local_map_resolution_);

    sub_obs_poses_ = nh_.subscribe("/obstacle_pose", 1, &LocalMapCreator::obs_poses_callback, this);
    pub_local_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);

    local_map_.header.frame_id = "base_link";

    local_map_.info.resolution = local_map_resolution_;
    local_map_.info.width = int(round(local_map_size_ / local_map_resolution_));
    local_map_.info.height = int(round(local_map_size_ / local_map_resolution_));
    local_map_.info.origin.position.x = -local_map_size_ / 2.0;
    local_map_.info.origin.position.y = -local_map_size_ / 2.0;

    local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
}

void LocalMapCreator::obs_poses_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    obs_poses_ = *msg;
    is_get_obs_poses_ = true;
}

void LocalMapCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(is_get_obs_poses_)
        {
            init_local_map();
            update_local_map();
            pub_local_map_.publish(local_map_);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void LocalMapCreator::init_local_map()
{
    local_map_.data.clear();
    local_map_.data.assign(local_map_.info.width * local_map_.info.height, -1);

}

void LocalMapCreator::update_local_map()
{

    for(int i = 0; i < obs_poses_.poses.size(); i++)
    {
        const double x = obs_poses_.poses[i].position.x;
        const double y = obs_poses_.poses[i].position.y;
        const double dist = hypot(x, y);
        const double angle = atan2(y, x);

            for(double dist_from_robot=0.0; (dist_from_robot<dist && is_in_local_map(dist_from_robot, angle)); dist_from_robot+=local_map_resolution_)
            {
                const int grid_index = get_grid_index(dist_from_robot,angle);
                local_map_.data[grid_index] = 0;
            }
            if(is_in_local_map(dist, angle))
            {
                const int grid_index = xy_to_index(x,y);
                local_map_.data[grid_index] = 100;
            }
    }
}

bool LocalMapCreator::is_in_local_map(const double dist, const double angle)
{
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    if(index_x < local_map_.info.width && index_x >= 0 && index_y < local_map_.info.height && index_y >= 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int LocalMapCreator::get_grid_index(const double dist, const double angle)
{
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);
    int index = xy_to_index(x, y);
    return index;
}

int LocalMapCreator::xy_to_index(const double x, const double y)
{
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    int index = index_y * local_map_.info.width + index_x;
    return index;
}

