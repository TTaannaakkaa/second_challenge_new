#include "obstacle_detector/obstacle_detector.h"

ObstacleDetector::ObstacleDetector()
    : private_nh_("~")
{
    private_nh_.param("hz", hz_);
    private_nh_.param("lasar_step", lasar_step_);
    private_nh_.param("robot_frame", robot_frame_);
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
        const double angle = lasar_scan_.angle_min + lasar_scan_.angle_increment * i;
        const double range = lasar_scan_.ranges[i];

        if(is_ignore_scan(angle))
        {
            continue;
        }

        if(range < ignore_distance_)
        {
            geometry_msgs::Pose obs_pose;
            obs_pose.position.x = range * cos(angle);
            obs_pose.position.y = range * sin(angle);
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


