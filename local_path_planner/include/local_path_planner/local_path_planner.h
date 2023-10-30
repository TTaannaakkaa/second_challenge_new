#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include "/home/amsl/catkin_ws/devel/.private/roomba_500driver_meiji/include/roomba_500driver_meiji/RoombaCtrl.h"

struct State
{
    double x;
    double y;
    double yaw;
    double vel;
    double yawrate;
};

struct Dynamic_Window
{
    double max_vel;
    double min_vel;
    double max_yawrate;
    double min_yawrate;
};

class DWAPlanner
{
    public:
        DWAPlanner();
        void process();
        void debager();

    private:
        void local_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
        void obs_pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg);

        void roomba_ctl(double vel, double yawrate);
        void move_robot_image(State& state, double vel, double yawrate);
        void visualize_trajectory(const std::vector<State>& trajectory, const ros::Publisher& local_path_pub, const ros::Time now);
        double nomalize_angle(double angle);
        double calc_eval(const std::vector<State>& trajectory);
        double calc_heading_eval(const std::vector<State>& trajectory);
        double calc_velocity_eval(const std::vector<State>& trajectory);
        double calc_distance_eval(const std::vector<State>& trajectory);
        std::vector<State> calc_trajectory(double vel, double yawrate);

        bool is_goal_reached();
        void calc_dynamic_window();
        std::vector<double> calc_input();

        int hz_;
        double dt_;
        double goal_tolerance_;
        double max_vel_;
        double min_vel_;
        double max_yawrate_;
        double min_yawrate_;
        double max_accel_;
        double max_dyawrate_;
        double v_reso_;
        double y_reso_;
        double predict_time_;
        double heading_cost_gain_;
        double velocity_cost_gain_;
        double distance_cost_gain_;
        double robot_radius_;
        double radius_margin_;
        double search_range_;


        bool flag_local_goal_ = false;
        bool flag_obs_pose_ = false;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        State roomba_;
        Dynamic_Window dw_;

        ros::Subscriber local_goal_sub_;
        ros::Subscriber obs_pose_sub_;

        ros::Publisher cmd_speed_pub_;
        ros::Publisher predict_path_pub_;
        ros::Publisher optimal_path_pub_;

        geometry_msgs::PointStamped local_goal_;
        geometry_msgs::PoseArray obs_pose_;

        tf2_ros::Buffer tfBuffer_;

        roomba_500driver_meiji::RoombaCtrl roomba_ctl_msg_;

};

#endif // LOCAL_PATH_PLANNER_H