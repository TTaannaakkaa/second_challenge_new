#include "local_path_planner/local_path_planner.h"

DWAPlanner::DWAPlanner():private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("dt", dt_);
    private_nh_.getParam("goal_tolerance", goal_tolerance_);
    private_nh_.getParam("max_vel", max_vel_);
    private_nh_.getParam("min_vel", min_vel_);
    private_nh_.getParam("max_yawrate", max_yawrate_);
    private_nh_.getParam("min_yawrate", min_yawrate_);
    private_nh_.getParam("max_accel", max_accel_);
    private_nh_.getParam("max_dyawrate", max_dyawrate_);
    private_nh_.getParam("v_reso", v_reso_);
    private_nh_.getParam("y_reso", y_reso_);
    private_nh_.getParam("predict_time", predict_time_);
    private_nh_.getParam("heading_cost_gain", heading_cost_gain_);
    private_nh_.getParam("velocity_cost_gain", velocity_cost_gain_);
    private_nh_.getParam("distance_cost_gain", distance_cost_gain_);
    private_nh_.getParam("robot_radius", robot_radius_);
    private_nh_.getParam("radius_margin", radius_margin_);
    private_nh_.getParam("search_range", search_range_);


    local_goal_sub_ = nh_.subscribe("/local_goal", 1, &DWAPlanner::local_goal_callback, this);
    obs_pose_sub_ = nh_.subscribe("/obstacle_pose", 1, &DWAPlanner::obs_pose_callback, this);

    cmd_speed_pub_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
    predict_path_pub_ = nh_.advertise<nav_msgs::Path>("/predict_path", 1);
    optimal_path_pub_ = nh_.advertise<nav_msgs::Path>("/optimal_path", 1);
}

void DWAPlanner::local_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer_.lookupTransform("base_link", "map", ros::Time(0));
        flag_local_goal_ = true;
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        flag_local_goal_ = false;
        return;
    }
    tf2::doTransform(*msg, local_goal_, transformStamped);
}

void DWAPlanner::obs_pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    obs_pose_ = *msg;
    flag_obs_pose_ = true;
}

bool DWAPlanner::is_goal_reached()
{
    if(not(flag_local_goal_ or flag_obs_pose_))
    {
        return false;
    }

    double dx = local_goal_.point.x;
    double dy = local_goal_.point.y;
    double dist = hypot(dx, dy);

    if(dist < goal_tolerance_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void DWAPlanner::process()
{
    ROS_WARN_STREAM("hogehoge");
    ros::Rate loop_rate(hz_);
    tf2_ros::TransformListener tfListener(tfBuffer_);

    while(ros::ok())
    {
        if(not(is_goal_reached()))
        {
            const std::vector<double> input = calc_input();
            roomba_ctl(input[0], input[1]);
        }
        else
        {
            roomba_ctl(0.0, 0.0);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void DWAPlanner::roomba_ctl(double vel, double yawrate)
{
    roomba_ctl_msg_.mode = 11;
    roomba_ctl_msg_.cntl.linear.x = vel;
    roomba_ctl_msg_.cntl.angular.z = yawrate;

    cmd_speed_pub_.publish(roomba_ctl_msg_);
}

std::vector<double> DWAPlanner::calc_input()
{
    std::vector<double> input;
    std::vector<std::vector<State>> trajectory_list;
    double max_score = -1.0;
    int max_score_index = 0;

    calc_dynamic_window();

    int i = 0;
    for(double velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=v_reso_)
    {
        for(double yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate)
        {
            const std::vector<State> trajectory = calc_trajectory(velocity, yawrate);
            double score = calc_eval(trajectory);
            trajectory_list.push_back(trajectory);

            if(score > max_score)
            {
                max_score = score;
                max_score_index = i;
                input[0] = velocity;
                input[1] = yawrate;
            }
            i++;
        }
    }

    roomba_.vel = input[0];
    roomba_.yawrate = input[1];

    ros::Time now = ros::Time::now();
    for(int i=0; i<trajectory_list.size(); i++)
    {
        if(i == max_score_index)
        {
            visualize_trajectory(trajectory_list[i], optimal_path_pub_, now);
        }
        else
        {
            visualize_trajectory(trajectory_list[i], predict_path_pub_, now);
        }
    }

    return input;
}

void DWAPlanner::calc_dynamic_window()
{
    double Vs[] = {min_vel_, max_vel_, min_yawrate_, max_yawrate_};

    double Vd[] = {roomba_.vel - max_accel_*dt_,
                   roomba_.vel + max_accel_*dt_,
                   roomba_.yawrate - max_dyawrate_*dt_,
                   roomba_.yawrate + max_dyawrate_*dt_};

    dw_.min_vel = std::max(Vs[0], Vd[0]);
    dw_.max_vel = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

std::vector<State> DWAPlanner::calc_trajectory(double vel, double yawrate)
{
    std::vector<State> trajectory;
    State state = {0.0, 0.0, 0.0, 0.0, 0.0};

    for(int t=0; t<=predict_time_; t+=dt_)
    {
        move_robot_image(state, vel, yawrate);
        trajectory.push_back(state);
    }

    return trajectory;
}

void DWAPlanner::move_robot_image(State& state, double vel, double yawrate)
{
    state.x += vel * cos(state.yaw) * dt_;
    state.y += vel * sin(state.yaw) * dt_;
    state.yaw += yawrate * dt_;
    state.yaw = nomalize_angle(state.yaw);
    state.vel = vel;
    state.yawrate = yawrate;
}

double DWAPlanner::nomalize_angle(double angle)
{
    if(angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    else if(angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }

    return angle;
}

double DWAPlanner::calc_eval(const std::vector<State>& trajectory)
{
    double heading =  calc_heading_eval(trajectory);
    double velocity = calc_velocity_eval(trajectory);
    double distance = calc_distance_eval(trajectory);

    return heading_cost_gain_ * heading + velocity_cost_gain_ * velocity + distance_cost_gain_ * distance;
}

double DWAPlanner::calc_heading_eval(const std::vector<State>& trajectory)
{
    double dx = local_goal_.point.x - trajectory.back().x;
    double dy = local_goal_.point.y - trajectory.back().y;
    double goal_yaw = atan2(dy, dx);
    double yaw = trajectory.back().yaw;
    double heading = 0.0;
    if(goal_yaw > yaw)
    {
        heading = goal_yaw - yaw;
    }
    else
    {
        heading = yaw - goal_yaw;
    }

    return (M_PI - abs(nomalize_angle(heading - M_PI))) / M_PI;
}

double DWAPlanner::calc_velocity_eval(const std::vector<State>& trajectory)
{
    if(0.0 < trajectory.back().vel and trajectory.back().vel < max_vel_)
    {
        return trajectory.back().vel / max_vel_;
    }
    else
    {
        return 0.0;
    }
}

double DWAPlanner::calc_distance_eval(const std::vector<State>& trajectory)
{
    double min_dist = search_range_;

    for(const auto& state : trajectory)
    {
        for(const auto& obs : obs_pose_.poses)
        {
            double dx = state.x - obs.position.x;
            double dy = state.y - obs.position.y;
            double dist = hypot(dx, dy);

            if(dist < min_dist)
            {
                min_dist = dist;
            }
        }
    }

    return min_dist / search_range_;
}

void DWAPlanner::visualize_trajectory(const std::vector<State>& trajectory, const ros::Publisher& local_path_pub, const ros::Time now)
{
    nav_msgs::Path local_path;
    local_path.header.stamp = now;
    local_path.header.frame_id = "base_link";

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "base_link";

    for(const auto& state : trajectory)
    {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        local_path.poses.push_back(pose);
    }

    local_path_pub.publish(local_path);
}