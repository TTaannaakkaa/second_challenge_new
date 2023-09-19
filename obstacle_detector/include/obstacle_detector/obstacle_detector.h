/**
 * @file obstacle_detector.h
 * @author Takuma Tanaka
 * @brief detection obstacles from laser scan
 * @version 0.1
 * @date 2023-08-30 
 */

#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

class ObstacleDetector
{
    public:
        ObstacleDetector();
        void process();
    
    private:
        void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void scan_obstacle();
        bool is_ignore_scan(double angle);

        int hz_;
        int laser_step_;
        double ignore_dist_;
        std::string robot_frame_;
        std::vector<double> ignore_angle_range_list_;

        bool flag_laser_scan_ = false;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber laser_scan_sub_;

        ros::Publisher obstacle_pose_pub_;

        geometry_msgs::PoseArray obstacle_pose_array_;
        sensor_msgs::LaserScan laser_scan_;
};

#endif // OBSTACLE_DETECTOR_H