/***********************************************************
 *
 * @file: pid_planner.cpp
 * @breif: Contains the Proportional–Integral–Derivative (PID) controller local planner class
 * @author: Yang Haodong, Guo Zhanyu, Wu Maojia
 * @update: 2023-10-1
 * @version: 1.1
 *
 * Copyright (c) 2023，Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <pluginlib/class_list_macros.h>

#include "pid_planner.h"

PLUGINLIB_EXPORT_CLASS(pid_planner::PIDPlanner, nav_core::BaseLocalPlanner)

namespace pid_planner
{
/**
 * @brief Construct a new PIDPlanner object
 */
PIDPlanner::PIDPlanner()
  : initialized_(false), tf_(nullptr), costmap_ros_(nullptr), goal_reached_(false), plan_index_(0)
{
}

/**
 * @brief Construct a new PIDPlanner object
 */
PIDPlanner::PIDPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : PIDPlanner()
{
  initialize(name, tf, costmap_ros);
}

/**
 * @brief Destroy the PIDPlanner object
 */
PIDPlanner::~PIDPlanner()
{
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void PIDPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh = ros::NodeHandle("~/" + name);

    nh.param("p_window", p_window_, 0.5);

    nh.param("p_precision", p_precision_, 0.2);
    nh.param("o_precision", o_precision_, 0.5);

    nh.param("max_v", max_v_, 0.5);
    nh.param("min_v", min_v_, 0.0);
    nh.param("max_v_inc", max_v_inc_, 0.5);

    nh.param("max_w", max_w_, 1.57);
    nh.param("min_w", min_w_, 0.0);
    nh.param("max_w_inc", max_w_inc_, 1.57);

    nh.param("k_v_p", k_v_p_, 1.00);
    nh.param("k_v_i", k_v_i_, 0.01);
    nh.param("k_v_d", k_v_d_, 0.10);

    nh.param("k_w_p", k_w_p_, 1.00);
    nh.param("k_w_i", k_w_i_, 0.01);
    nh.param("k_w_d", k_w_d_, 0.10);

    nh.param("k_theta", k_theta_, 0.5);

    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency, 10.0);
    d_t_ = 1 / controller_freqency;

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;

    odom_helper_ = new base_local_planner::OdometryHelperRos("/odom");
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
    current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);

    ROS_INFO("PID planner initialized!");
  }
  else
    ROS_WARN("PID planner has already been initialized.");
}

/**
 * @brief  Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return  true if the plan was updated successfully, else false
 */
bool PIDPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  ROS_INFO("Got new plan");

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  plan_index_ = std::min(1, (int)global_plan_.size() - 1);

  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_rpy_ = getEulerAngles(global_plan_.back());
    goal_reached_ = false;

    e_v_ = i_v_ = 0.0;
    e_w_ = i_w_ = 0.0;
  }

  return true;
}

/**
 * @brief  Check if the goal pose has been achieved
 * @return True if achieved, false otherwise
 */
bool PIDPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("PID planner has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

/**
 * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
 * @param cmd_vel will be filled with the velocity command to be passed to the robot base
 * @return  true if a valid trajectory was found, else false
 */
bool PIDPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR("PID planner has not been initialized");
    return false;
  }

  // current pose
  geometry_msgs::PoseStamped current_ps_odom;
  costmap_ros_->getRobotPose(current_ps_odom);

  // transform into map
  tf_->transform(current_ps_odom, current_ps_, map_frame_);

  // current angle
  double theta = tf2::getYaw(current_ps_.pose.orientation);  // [-pi, pi]

  double theta_d, theta_dir, theta_trj;
  double b_x_d, b_y_d;  // desired x, y in base frame
  double e_theta;

  while (plan_index_ < global_plan_.size())
  {
    target_ps_ = global_plan_[plan_index_];
    double x_d = target_ps_.pose.position.x;
    double y_d = target_ps_.pose.position.y;

    // from robot to plan point
    theta_dir = atan2((y_d - current_ps_.pose.position.y), (x_d - current_ps_.pose.position.x));

    int next_plan_index = plan_index_ + 1;
    if (next_plan_index < global_plan_.size())
    {
      // theta on the trajectory
      theta_trj = atan2((global_plan_[next_plan_index].pose.position.y - y_d),
                        (global_plan_[next_plan_index].pose.position.x - x_d));
    }

    // if the difference is greater than PI, it will get a wrong result
    if (fabs(theta_trj - theta_dir) > M_PI)
    {
      // add 2*PI to the smaller one
      if (theta_trj > theta_dir)
        theta_dir += 2 * M_PI;
      else
        theta_trj += 2 * M_PI;
    }

    // weighting between two angle
    theta_d = (1 - k_theta_) * theta_trj + k_theta_ * theta_dir;
    regularizeAngle(theta_d);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_d);
    tf2::convert(q, target_ps_.pose.orientation);

    // transform from map into base_frame
    geometry_msgs::PoseStamped dst;
    target_ps_.header.stamp = ros::Time(0);
    tf_->transform(target_ps_, dst, base_frame_);
    b_x_d = dst.pose.position.x;
    b_y_d = dst.pose.position.y;

    e_theta = theta_d - theta;
    regularizeAngle(e_theta);

    if (std::hypot(b_x_d, b_y_d) > p_window_)
      break;

    ++plan_index_;
  }

  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // position reached
  if (dist(Eigen::Vector2d(global_plan_.back().pose.position.x, global_plan_.back().pose.position.y),
           Eigen::Vector2d(current_ps_.pose.position.x, current_ps_.pose.position.y)) < p_precision_)
  {
    e_theta = goal_rpy_[2] - theta;
    regularizeAngle(e_theta);

    // orientation reached
    if (std::fabs(e_theta) < o_precision_)
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;

      goal_reached_ = true;
    }
    // orientation not reached
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = AngularPIDController(base_odom, e_theta);
    }
  }
  // large angle, turn first
  else if (std::fabs(e_theta) > M_PI_2)
  {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = AngularPIDController(base_odom, e_theta);
  }
  // posistion not reached
  else
  {
    cmd_vel.linear.x = LinearPIDController(base_odom, b_x_d, b_y_d);
    cmd_vel.angular.z = AngularPIDController(base_odom, e_theta);
  }

  // publish next target_ps_ pose
  // target_ps_.header.frame_id = "map";
  // target_ps_.header.stamp = ros::Time::now();
  target_pose_pub_.publish(target_ps_);

  // publish robot pose
  // current_ps_.header.frame_id = "map";
  // current_ps_.header.stamp = ros::Time::now();
  current_pose_pub_.publish(current_ps_);

  return true;
}

/**
 * @brief PID controller in linear
 * @param base_odometry odometry of the robot, to get velocity
 * @param b_x_d         desired x in body frame
 * @param b_y_d         desired y in body frame
 * @return  linear velocity
 */
double PIDPlanner::LinearPIDController(nav_msgs::Odometry& base_odometry, double b_x_d, double b_y_d)
{
  double v = std::hypot(base_odometry.twist.twist.linear.x, base_odometry.twist.twist.linear.y);
  double v_d = std::hypot(b_x_d, b_y_d) / d_t_;
  if (std::fabs(v_d) > max_v_)
    v_d = std::copysign(max_v_, v_d);

  double e_v = v_d - v;
  i_v_ += e_v * d_t_;
  double d_v = (e_v - e_v_) / d_t_;
  e_v_ = e_v;

  double v_inc = k_v_p_ * e_v + k_v_i_ * i_v_ + k_v_d_ * d_v;

  if (std::fabs(v_inc) > max_v_inc_)
    v_inc = std::copysign(max_v_inc_, v_inc);

  double v_cmd = v + v_inc;
  if (std::fabs(v_cmd) > max_v_)
    v_cmd = std::copysign(max_v_, v_cmd);
  else if (std::fabs(v_cmd) < min_v_)
    v_cmd = std::copysign(min_v_, v_cmd);

  return v_cmd;
}

/**
 * @brief PID controller in angular
 * @param base_odometry odometry of the robot, to get velocity
 * @param e_theta       the error between the current and desired theta
 * @return  angular velocity
 */
double PIDPlanner::AngularPIDController(nav_msgs::Odometry& base_odometry, double e_theta)
{
  regularizeAngle(e_theta);

  double w_d = e_theta / d_t_;
  if (std::fabs(w_d) > max_w_)
    w_d = std::copysign(max_w_, w_d);

  double w = base_odometry.twist.twist.angular.z;
  double e_w = w_d - w;
  i_w_ += e_w * d_t_;
  double d_w = (e_w - e_w_) / d_t_;
  e_w_ = e_w;

  double w_inc = k_w_p_ * e_w + k_w_i_ * i_w_ + k_w_d_ * d_w;

  if (std::fabs(w_inc) > max_w_inc_)
    w_inc = std::copysign(max_w_inc_, w_inc);

  double w_cmd = w + w_inc;
  if (std::fabs(w_cmd) > max_w_)
    w_cmd = std::copysign(max_w_, w_cmd);
  else if (std::fabs(w_cmd) < min_w_)
    w_cmd = std::copysign(min_w_, w_cmd);

  return w_cmd;
}

}  // namespace pid_planner