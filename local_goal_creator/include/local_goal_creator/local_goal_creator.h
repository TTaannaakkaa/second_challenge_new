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
    void goalCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void publishGoal();

    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher goal_pub_;

    nav_msgs::Path path_;
    geometry_msgs::PointStamped goal_;
};

#endif // LOCAL_GOAL_CREATOR_H