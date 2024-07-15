/***********************************************************
 *
 * @file: pid_planner.h
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

#ifndef PID_PLANNER_H_
#define PID_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <tf2/utils.h>
#include <Eigen/Dense>

#include "local_planner.h"

namespace pid_planner
{
/**
 * @brief A class implementing a local planner using the PID
 */
class PIDPlanner : public nav_core::BaseLocalPlanner, local_planner::LocalPlanner
{
public:
  /**
   * @brief Construct a new PIDPlanner object
   */
  PIDPlanner();

  /**
   * @brief Construct a new PIDPlanner object
   */
  PIDPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the PIDPlanner object
   */
  ~PIDPlanner();

  /**
   * @brief Initialization of the local planner
   * @param name        the name to give this instance of the trajectory planner
   * @param tf          a pointer to a transform listener
   * @param costmap_ros the cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return  true if the plan was updated successfully, else false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  /**
   * @brief  Check if the goal pose has been achieved
   * @return True if achieved, false otherwise
   */
  bool isGoalReached();

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
   * @param cmd_vel will be filled with the velocity command to be passed to the robot base
   * @return  true if a valid trajectory was found, else false
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief PID controller in linear
   * @param base_odometry odometry of the robot, to get velocity
   * @param b_x_d         desired x in body frame
   * @param b_y_d         desired y in body frame
   * @return  linear velocity
   */
  double LinearPIDController(nav_msgs::Odometry& base_odometry, double b_x_d, double b_y_d);

  /**
   * @brief PID controller in angular
   * @param base_odometry odometry of the robot, to get velocity
   * @param e_theta       the error between the current and desired theta
   * @return  angular velocity
   */
  double AngularPIDController(nav_msgs::Odometry& base_odometry, double e_theta);

private:
  bool initialized_, goal_reached_;
  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;

  int plan_index_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  geometry_msgs::PoseStamped target_ps_, current_ps_;

  double p_window_;                   // next point distance
  double p_precision_, o_precision_;  // goal reached tolerance
  double d_t_;                        // control time step

  double max_v_, min_v_, max_v_inc_;  // linear velocity
  double max_w_, min_w_, max_w_inc_;  // angular velocity

  double k_v_p_, k_v_i_, k_v_d_;  // pid controller params
  double k_w_p_, k_w_i_, k_w_d_;  // pid controller params
  double k_theta_;                // pid controller params

  double e_v_, e_w_;
  double i_v_, i_w_;

  base_local_planner::OdometryHelperRos* odom_helper_;
  ros::Publisher target_pose_pub_, current_pose_pub_;

  double goal_x_, goal_y_;
  Eigen::Vector3d goal_rpy_;
};
};  // namespace pid_planner

#endif