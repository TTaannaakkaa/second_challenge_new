#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator() : private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("index_step", index_step_);
    private_nh_.getParam("target_distance", taeget_distance_);
    private_nh_.getParam("goal_index", goal_index_);

    goal_.header.frame_id = "map";

    path_sub_ = nh_.subscribe("/global_path", 1, &LocalGoalCreator::pathCallback, this);
    pose_sub_ = nh_.subscribe("/estimate_pose", 1, &LocalGoalCreator::poseCallback, this);
    local_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/local_goal", 1);
}

void LocalGoalCreator::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose_ = *msg;
}

void LocalGoalCreator::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    path_ = *msg;
    is_path_ = true;
}

void LocalGoalCreator::process()
{
    ros::Rate LoopRate(hz_);

    while(ros::ok())
    {
        if(is_path_)
        {
            publishGoal();
        }
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void LocalGoalCreator::publishGoal()
{
    double distance = getDistance();
    if(distance < taeget_distance_)
    {
        goal_index_ += index_step_;
        if(goal_index_ >= path_.poses.size())
        {
            goal_index_ = path_.poses.size() - 1;
        }
    }
    goal_.header.stamp = ros::Time::now();
    goal_.point = path_.poses[goal_index_].pose.position;
    local_goal_pub_.publish(goal_);
}

double LocalGoalCreator::getDistance()
{
    double dx = path_.poses[goal_index_].pose.position.x - pose_.pose.position.x;
    double dy = path_.poses[goal_index_].pose.position.y - pose_.pose.position.y;
    return hypot(dx, dy);
}

