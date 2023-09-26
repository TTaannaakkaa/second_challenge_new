#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

class LocalGoalCreator
{
public:
    LocalGoalCreator();
    void process();

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void publishGoal();
    double getDistance();

    int hz_;
    int index_step_;
    int goal_index_;
    double taeget_distance_;
    bool is_path_ = false;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher local_goal_pub_;

    nav_msgs::Path path_;
    geometry_msgs::PointStamped goal_;
    geometry_msgs::PoseStamped pose_;
};

#endif // LOCAL_GOAL_CREATOR_H