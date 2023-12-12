// #include "local_path_planner/local_path_planner.h"

// DWAPlanner::DWAPlanner():private_nh_("~")
// {
//     private_nh_.getParam("hz", hz_);
//     private_nh_.getParam("dt", dt_);
//     private_nh_.getParam("goal_tolerance", goal_tolerance_);
//     private_nh_.getParam("max_vel", max_vel_);
//     private_nh_.getParam("min_vel", min_vel_);
//     private_nh_.getParam("max_yawrate", max_yawrate_);
//     private_nh_.getParam("min_yawrate", min_yawrate_);
//     private_nh_.getParam("max_accel", max_accel_);
//     private_nh_.getParam("max_dyawrate", max_dyawrate_);
//     private_nh_.getParam("v_reso", v_reso_);
//     private_nh_.getParam("y_reso", y_reso_);
//     private_nh_.getParam("predict_time", predict_time_);
//     private_nh_.getParam("heading_cost_gain", heading_cost_gain_);
//     private_nh_.getParam("velocity_cost_gain", velocity_cost_gain_);
//     private_nh_.getParam("distance_cost_gain", distance_cost_gain_);
//     private_nh_.getParam("robot_radius", robot_radius_);
//     private_nh_.getParam("radius_margin", radius_margin_);
//     private_nh_.getParam("search_range", search_range_);


//     local_goal_sub_ = nh_.subscribe("/local_goal", 1, &DWAPlanner::local_goal_callback, this);
//     obs_pose_sub_ = nh_.subscribe("/obstacle_pose", 1, &DWAPlanner::obs_pose_callback, this);

//     cmd_speed_pub_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
//     predict_path_pub_ = nh_.advertise<nav_msgs::Path>("/predict_path", 1);
//     optimal_path_pub_ = nh_.advertise<nav_msgs::Path>("/optimal_path", 1);
// }

// void DWAPlanner::local_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
// {
//     geometry_msgs::TransformStamped transformStamped;
//     try
//     {
//         transformStamped = tfBuffer_.lookupTransform("base_link", "map", ros::Time(0));
//         flag_local_goal_ = true;
//     }
//     catch (tf2::TransformException &ex) {
//         ROS_WARN("%s",ex.what());
//         flag_local_goal_ = false;
//         return;
//     }
//     tf2::doTransform(*msg, local_goal_, transformStamped);
// }

// void DWAPlanner::obs_pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
// {
//     obs_pose_ = *msg;
//     flag_obs_pose_ = true;
// }

// bool DWAPlanner::is_goal_reached()
// {
//     if(not(flag_local_goal_ or flag_obs_pose_))
//     {
//         return false;
//     }

//     double dx = local_goal_.point.x;
//     double dy = local_goal_.point.y;
//     double dist = hypot(dx, dy);

//     if(dist > goal_tolerance_)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// void DWAPlanner::process()
// {
//     ros::Rate loop_rate(hz_);
//     tf2_ros::TransformListener tfListener(tfBuffer_);

//     while(ros::ok())
//     {
//         if(is_goal_reached())
//         {
//             const std::vector<double> input = calc_input();
//             ROS_WARN_STREAM("input[0]: " << input[0]);
//             ROS_WARN_STREAM("input[1]: " << input[1]);
//             roomba_ctl(input[0], input[1]);
//         }
//         else
//         {
//             roomba_ctl(0.0, 0.0);
//         }

//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

// void DWAPlanner::roomba_ctl(double vel, double yawrate)
// {
//     roomba_ctl_msg_.mode = 11;
//     roomba_ctl_msg_.cntl.linear.x = vel;
//     roomba_ctl_msg_.cntl.angular.z = yawrate;

//     cmd_speed_pub_.publish(roomba_ctl_msg_);
// }

// std::vector<double> DWAPlanner::calc_input()
// {
//     // ROS_WARN_STREAM("calc_input");
//     std::vector<double> input;
//     std::vector<std::vector<State>> trajectory_list;
//     double max_score = -1.0;
//     int max_score_index = 0;

//     calc_dynamic_window();

//     int i = 0;
//     for(double velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=v_reso_)
//     {
//         for(double yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate)
//         {
//             const std::vector<State> trajectory = calc_trajectory(velocity, yawrate);
//             double score = calc_eval(trajectory);
//             trajectory_list.push_back(trajectory);

//             if(score > max_score)
//             {
//                 max_score = score;
//                 max_score_index = i;
//                 input[0] = velocity;
//                 input[1] = yawrate;
//             }
//             // ROS_WARN_STREAM("i: " << i);
//             i++;
//         }
//     }

//     roomba_.vel = input[0];
//     roomba_.yawrate = input[1];

//     ros::Time now = ros::Time::now();
//     for(int i=0; i<trajectory_list.size(); i++)
//     {
//         if(i == max_score_index)
//         {
//             visualize_trajectory(trajectory_list[i], optimal_path_pub_, now);
//         }
//         else
//         {
//             visualize_trajectory(trajectory_list[i], predict_path_pub_, now);
//         }
//     }

//     return input;
// }

// void DWAPlanner::calc_dynamic_window()
// {
//     ROS_WARN_STREAM("calc_dynamic_window");
//     double Vs[] = {min_vel_, max_vel_, min_yawrate_, max_yawrate_};

//     double Vd[] = {roomba_.vel - max_accel_*dt_,
//                    roomba_.vel + max_accel_*dt_,
//                    roomba_.yawrate - max_dyawrate_*dt_,
//                    roomba_.yawrate + max_dyawrate_*dt_};

//     dw_.min_vel = std::max(Vs[0], Vd[0]);
//     dw_.max_vel = std::min(Vs[1], Vd[1]);
//     dw_.min_yawrate = std::max(Vs[2], Vd[2]);
//     dw_.max_yawrate = std::min(Vs[3], Vd[3]);
//     // ROS_WARN_STREAM("dw_.min_vel: " << dw_.min_vel);
//     // ROS_WARN_STREAM("dw_.max_vel: " << dw_.max_vel);
//     // ROS_WARN_STREAM("dw_.min_yawrate: " << dw_.min_yawrate);
//     // ROS_WARN_STREAM("dw_.max_yawrate: " << dw_.max_yawrate);
// }

// std::vector<State> DWAPlanner::calc_trajectory(double vel, double yawrate)
// {
//     std::vector<State> trajectory;
//     State state = {0.0, 0.0, 0.0, 0.0, 0.0};

//     for(double t=0.0; t<=predict_time_; t+=dt_)
//     {
//         move_robot_image(state, vel, yawrate);
//         trajectory.push_back(state);
//     }

//     // ROS_WARN_STREAM("trajectory.back().x: " << trajectory.back().x);
//     // ROS_WARN_STREAM("trajectory.back().y: " << trajectory.back().y);
//     return trajectory;
// }

// void DWAPlanner::move_robot_image(State& state, double vel, double yawrate)
// {
//     ROS_WARN_STREAM("vel: " << vel);
//     ROS_WARN_STREAM("yawrate: " << yawrate);
//     ROS_WARN_STREAM("dt_: " << dt_);
//     state.x += vel * cos(state.yaw) * dt_;
//     state.y += vel * sin(state.yaw) * dt_;
//     state.yaw += yawrate * dt_;
//     state.yaw = nomalize_angle(state.yaw);
//     state.vel = vel;
//     state.yawrate = yawrate;
// }

// double DWAPlanner::nomalize_angle(double angle)
// {
//     if(angle > M_PI)
//     {
//         angle -= 2.0 * M_PI;
//     }
//     else if(angle < -M_PI)
//     {
//         angle += 2.0 * M_PI;
//     }

//     return angle;
// }

// double DWAPlanner::calc_eval(const std::vector<State>& trajectory)
// {
//     double heading =  calc_heading_eval(trajectory);
//     double velocity = calc_velocity_eval(trajectory);
//     double distance = calc_distance_eval(trajectory);

//     return heading_cost_gain_ * heading + velocity_cost_gain_ * velocity + distance_cost_gain_ * distance;
// }

// double DWAPlanner::calc_heading_eval(const std::vector<State>& trajectory)
// {
//     double dx = local_goal_.point.x - trajectory.back().x;
//     double dy = local_goal_.point.y - trajectory.back().y;
//     double goal_yaw = atan2(dy, dx);
//     double yaw = trajectory.back().yaw;
//     double heading = 0.0;
//     if(goal_yaw > yaw)
//     {
//         heading = goal_yaw - yaw;
//     }
//     else
//     {
//         heading = yaw - goal_yaw;
//     }

//     return (M_PI - abs(nomalize_angle(heading - M_PI))) / M_PI;
// }

// double DWAPlanner::calc_velocity_eval(const std::vector<State>& trajectory)
// {
//     if(0.0 < trajectory.back().vel and trajectory.back().vel < max_vel_)
//     {
//         return trajectory.back().vel / max_vel_;
//     }
//     else
//     {
//         return 0.0;
//     }
// }

// double DWAPlanner::calc_distance_eval(const std::vector<State>& trajectory)
// {
//     double min_dist = search_range_;

//     for(const auto& state : trajectory)
//     {
//         for(const auto& obs : obs_pose_.poses)
//         {
//             double dx = state.x - obs.position.x;
//             double dy = state.y - obs.position.y;
//             double dist = hypot(dx, dy);

//             if(dist < min_dist)
//             {
//                 min_dist = dist;
//             }
//         }
//     }

//     return min_dist / search_range_;
// }

// void DWAPlanner::visualize_trajectory(const std::vector<State>& trajectory, const ros::Publisher& local_path_pub, const ros::Time now)
// {
//     nav_msgs::Path local_path;
//     local_path.header.stamp = now;
//     local_path.header.frame_id = "base_link";

//     geometry_msgs::PoseStamped pose;
//     pose.header.stamp = now;
//     pose.header.frame_id = "base_link";

//     for(const auto& state : trajectory)
//     {
//         pose.pose.position.x = state.x;
//         pose.pose.position.y = state.y;
//         std::cout << "x: " << state.x << std::endl;
//         std::cout << "y: " << state.y << std::endl;
//         local_path.poses.push_back(pose);
//     }

//     local_path_pub.publish(local_path);
// }

// void DWAPlanner::debager()
// {
//     ROS_ERROR_STREAM("hogehoge");
// }

#include "local_path_planner/local_path_planner.h"
DWA::DWA():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("goal_tolerance", goal_tolerance_, {0.1});
    private_nh_.param("max_vel", max_vel_, {0.4});
    private_nh_.param("min_vel", min_vel_, {0.0});
    private_nh_.param("max_yawrate", max_yawrate_, {1.0});
    private_nh_.param("max_accel", max_accel_, {1000.0});
    private_nh_.param("max_dyawrate", max_dyawrate_, {1000.0});
    private_nh_.param("dt",dt_,{0.1});
    private_nh_.param("predict_time", predict_time_, {3.0});
    private_nh_.param("weight_heading", weight_heading_, {0.9});
    private_nh_.param("weight_distance", weight_distance_, {1.5});
    private_nh_.param("weight_velocity", weight_velocity_, {0.5});
    private_nh_.param("search_range", search_range_, {1.0});
    private_nh_.param("roomba_radius", roomba_radius_, {0.2});
    private_nh_.param("radius_margin", radius_margin_, {0.2});
    private_nh_.param("vel_reso", vel_reso_, {0.05});
    private_nh_.param("yawrate_reso", yawrate_reso_, {0.02});
    private_nh_.param("is_visible", is_visible_, {true});


    //Subscriber

    sub_local_goal_ = nh_.subscribe("/local_goal", 1, &DWA::local_goal_callback, this);
    sub_ob_poses_   = nh_.subscribe("/obstacle_pose", 1, &DWA::obstacle_poses_callback, this);

    //Publisher
    pub_cmd_vel_    = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
    pub_predict_path_ = nh_.advertise<nav_msgs::Path>("/predict_local_paths", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
    //pub_local_path_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);
}

void DWA::local_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform("base_link", "map", ros::Time(0));
        flag_local_goal_ = true;
    }
    catch(tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        flag_local_goal_ = false;
        return;
    }
    tf2::doTransform(*msg, local_goal_, transform);
}

void DWA::obstacle_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    ob_poses_ = *msg;
    // ROS_INFO("ob:OK");
    flag_ob_poses_ = true;
}


bool DWA::can_move()
{
    if(!(flag_local_goal_ && flag_ob_poses_)) return false;

    // ROS_INFO("l_goal:%f",local_goal_.point.x);
    const double dx = local_goal_.point.x - roomba_.x;
    const double dy = local_goal_.point.y - roomba_.y;
    const double dist_to_goal = hypot(dx, dy);

    if(dist_to_goal > goal_tolerance_)
        return true;
    else
        return false;
}

//Dynamic Window計算
void DWA::calc_dynamic_window()
{
    double Vs[] = {min_vel_,max_vel_,-max_yawrate_,max_yawrate_};

    double Vd[] = {roomba_.velocity - max_accel_*dt_,
        roomba_.velocity + max_accel_*dt_,
        roomba_.yawrate  - max_dyawrate_*dt_,
        roomba_.yawrate  + max_dyawrate_*dt_};

    dw_.min_vel = std::max(Vs[0], Vd[0]);
    dw_.max_vel = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}

double DWA::calc_evaluation(const std::vector<State>& traj)
{
    const double heading_score  = calc_heading_score(traj);
    const double heading_cost  = weight_heading_ * heading_score;
    const double distance_score = calc_dist_score(traj);
    const double distance_cost = weight_distance_ * distance_score;
    const double velocity_score = calc_vel_score(traj);
    const double velocity_cost = weight_velocity_ * velocity_score;

    // ROS_INFO("heading:%f distance:%f velocity:%f",heading_score,distance_score,velocity_score);
    const double total_cost= heading_cost + distance_cost + velocity_cost;
    return total_cost;
}

double DWA::calc_heading_score(const std::vector<State>& traj)
{
    const double theta = traj.back().yaw;

    const double goal_theta = atan2(local_goal_.point.y - traj.back().y, local_goal_.point.x - traj.back().x);
    // ROS_INFO("lg.y:%f traj.y:%f lg.x:%f traj.x:%f",local_goal_.point.y,traj.back().y,local_goal_.point.x,traj.back().x);
    double target_theta = 0.0;

    if(goal_theta > theta)
        target_theta = goal_theta - theta;
    else
        target_theta = theta - goal_theta;

    const double heading_eval = (M_PI - abs(regulate_angle(target_theta)))/M_PI;

    return heading_eval;
}

double DWA::calc_dist_score(const std::vector<State>& traj)
{
    double min_dist = search_range_;
    for(const auto& state : traj)
    {
        for(const auto& ob_pose : ob_poses_.poses)
        {
            const double dx = ob_pose.position.x - state.x;
            const double dy = ob_pose.position.y - state.y;
            const double dist = hypot(dx, dy);

            if(dist <= roomba_radius_+radius_margin_)
                return -1e6;

            if(dist < min_dist)
                min_dist = dist;
        }
    }
    return min_dist/search_range_;
}

//verocity評価関数
double DWA::calc_vel_score(const std::vector<State>& traj)
{
    if(0.0 < traj.back().velocity) // 前進
        return traj.back().velocity/max_vel_; // 正規化

    else
        return 0.0;
}

double DWA::regulate_angle(double angle)
{
    if(angle > M_PI)
        angle -= 2.0 * M_PI;
    if(angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

std::vector<double> DWA::calc_input()
{
    std::vector<double> input{0.0, 0.0};
    std::vector<std::vector<State>> trajectories;

    calc_dynamic_window();

    double max_score = -1e6;
    int max_score_index = 0;


    int i=0;
    for(double velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=vel_reso_)
    {
        // if(velocity==0.0) continue;
        for(double yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate_reso_)
        {
            if(velocity<vel_reso_*0.5 and abs(yawrate)<yawrate_reso_*2.0)
                continue;
            const std::vector<State> trajectory = calc_trajectory(velocity, yawrate);
            double score = calc_evaluation(trajectory);
            trajectories.push_back(trajectory);

            if(max_score < score)
            {
                max_score = score;
                input[0] = velocity;
                input[1] = yawrate;
                max_score_index = i;
            }
            // ROS_INFO("i:%d vel:%f yaw:%F score:%d",i,velocity,yawrate,score);
            i++;
        }
    }

    roomba_.velocity = input[0];
    roomba_.yawrate  = input[1];

    if(is_visible_)
    {
        ros::Time now = ros::Time::now();
        for(i=0; i<trajectories.size(); i++)
        {
            if(i == max_score_index)
                visualize_traj(trajectories[i], pub_optimal_path_, now);

            else
                // ROS_INFO_STREAM("hogehoge");
                // ROS_INFO_STREAM("x:" << trajectories[i].back().x << " y:" << trajectories[i].back().y);
                visualize_traj(trajectories[i], pub_predict_path_, now);

        }
    }
    ROS_INFO("input:%f %f  max_score_index:%d",input[0],input[1],max_score_index);
    return input;
}

std::vector<State> DWA::calc_trajectory(const double velocity, const double yawrate)
{
    std::vector<State> trajectory;
    State state = {0.0, 0.0, 0.0, 0.0, 0.0};

    for(double t=0.0; t<=predict_time_; t+=dt_)
    {
        virtual_rb(state, velocity, yawrate);
        trajectory.push_back(state);
    }

    return trajectory;
}

void DWA::virtual_rb(State& state, const double velocity, const double yawrate)
{
    state.yaw += yawrate * dt_;
    state.yaw = regulate_angle(state.yaw);
    state.x += velocity * cos(state.yaw) * dt_;
    state.y += velocity * sin(state.yaw) * dt_;
    state.velocity = velocity;

    state.yawrate = yawrate;
}

void DWA::roomba_control(double velocity, double yawrate)
{
    cmd_velocity_.mode = 11;  //mode11:速度と角速度を設定
    cmd_velocity_.cntl.linear.x = velocity;
    cmd_velocity_.cntl.angular.z = yawrate;
    pub_cmd_vel_.publish(cmd_velocity_);
}

void DWA::visualize_traj(const std::vector<State>& traj, const ros::Publisher& pub_local_path, ros::Time now)
{
    nav_msgs::Path local_path;
    local_path.header.stamp = now;
    local_path.header.frame_id = "base_link";


    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "base_link";


    for(const auto& state : traj)
    {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        local_path.poses.push_back(pose);
    }
    pub_local_path.publish(local_path);
}

void DWA::process()
{
    ros::Rate loop_rate(hz_);
    tf2_ros::TransformListener tf_listener(tf_buffer_);

    while(ros::ok())
    {
        if(can_move())
        {
            // ROS_INFO("ugokeru");
            const std::vector<double> input = calc_input();
            roomba_control(input[0], input[1]);
        }
        else
        {
            // ROS_INFO("ugokenn");
            roomba_control(0.0, 0.0);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
