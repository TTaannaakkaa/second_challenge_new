#include "obstacle_detector/obstacle_detector.h"

ObstacleDetector::ObstacleDetector():private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("lasar_step", lasar_step_);
    private_nh_.getParam("robot_frame", robot_frame_);
    private_nh_.getParam("ignore_distance", ignore_distance_);
    private_nh_.getParam("ignore_angle_range_list", ignore_angle_range_list_);

    lasar_scan_sub_ = nh_.subscribe("/scan", 1, &ObstacleDetector::lasar_scan_callback, this);
    obstacle_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/obstacle_pose", 1);

    obstacle_pose_array_.header.frame_id = "base_link";
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
    // std::cout << "lasar size : " << lasar_scan_.ranges.size() << std::endl;
    std::cout << "lasar step : " << lasar_step_ << std::endl;
    for(int i=0;i<lasar_scan_.ranges.size();i+=lasar_step_)
    {
        // std::cout << "i : " << i << std::endl;
        const double angle = lasar_scan_.angle_min + lasar_scan_.angle_increment * i;
        const double range = lasar_scan_.ranges[i];

        // if(is_ignore_scan(angle))
        // {
        //     continue;
        // }

        // if(range < ignore_distance_)
        // {
        //     continue;
        // }

        geometry_msgs::Pose obs_pose;
        obs_pose.position.x = range * cos(angle);
        obs_pose.position.y = range * sin(angle);
        obstacle_pose_array_.poses.push_back(obs_pose);

    }
    // std::cout << "obstacle size : " << obstacle_pose_array_.poses.size() << std::endl;
    obstacle_pose_pub_.publish(obstacle_pose_array_);
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
    angle = abs(angle);
    const int size = ignore_angle_range_list_.size();

    for(int i=0; i<size/2; i++)
    {
        if(ignore_angle_range_list_[i*2] < angle and angle < ignore_angle_range_list_[i*2 + 1])
            return true;
    }

    if(size%2 == 1)
    {
        if(ignore_angle_range_list_[size-1] < angle)
            return true;
    }

    return false;
}