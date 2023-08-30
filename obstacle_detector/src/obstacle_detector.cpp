#include "obstacle_detector/obstacle_detector.h"

ObstacleDetector::ObstacleDetector()
    : private_nh_("~")
{
    private_nh_.param("hz", hz_);
    private_nh_.param("lasar_step", lasar_step_);
    private_nh_.param("ignore_distance", ignore_distance_);
    private_nh_.param("ignore_angle_range_list", ignore_angle_range_list_);

    lasar_scan_sub_ = nh_.subscribe("/scan", 1, &ObstacleDetector::lasar_scan_callback, this);
    obstacle_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/obstacle_pose", 1);

    obstacle_pose_array_.header.frame_id = robot_frame_;
}
/**
 * @brief main function
 */
void ObstacleDetector::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        if(flag_lasar_scan_)
        {
            scan_obstacle();
            obstacle_pose_pub_.publish(obstacle_pose_array_);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 * @brief callback function of laser scan
 * @param msg 
 */
void ObstacleDetector::lasar_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    lasar_scan_ = *msg;
    flag_lasar_scan_ = true;
}

/**
 * @brief scan obstacle
 * 
 */
void ObstacleDetector::scan_obstacle()
{
    obstacle_pose_array_.poses.clear();
    for(int i=0;i<lasar_scan_.ranges.size();i+=lasar_step_)
    {
        if(is_ignore_scan(lasar_scan_.angle_min + lasar_scan_.angle_increment * i))
        {
            continue;
        }
        if(lasar_scan_.ranges[i] < ignore_distance_)
        {
            geometry_msgs::Pose obs_pose;
            obs_pose.position.x = lasar_scan_.ranges[i] * cos(lasar_scan_.angle_min + lasar_scan_.angle_increment * i);
            obs_pose.position.y = lasar_scan_.ranges[i] * sin(lasar_scan_.angle_min + lasar_scan_.angle_increment * i);
            obstacle_pose_array_.poses.push_back(obs_pose);
        }
    }
}

/**
 * @brief check if the scan is ignored
 * 
 * @param angle     angle of scan
 * @return true     
 * @return false 
 */
bool ObstacleDetector::is_ignore_scan(double angle)
{
    for(int i = 0; i < ignore_angle_range_list_.size(); i += 2)
    {
        if(ignore_angle_range_list_[i] < angle && angle < ignore_angle_range_list_[i + 1])
        {
            return true;
        }
    }
    return false;
}


