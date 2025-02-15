/***********************************************************
 *
 * @file: graph_planner.h
 * @breif: Contains the graph planner ROS wrapper class
 * @author: Yang Haodong
 * @update: 2022-10-26
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef GRAPH_PLANNER_H
#define GRAPH_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
// #include <angles/angles.h>
// #include <base_local_planner/world_model.h>
// #include <base_local_planner/costmap_model.h>

// #include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
// #include <geometry_msgs/Point.h>

#include "global_planner.h"

namespace graph_planner
{
class GraphPlanner : public nav_core::BaseGlobalPlanner
{
public:
  /**
   * @brief Construct a new Graph Planner object
   */
  GraphPlanner();

  /**
   * @brief Construct a new Graph Planner object
   * @param name      planner name
   * @param costmap_ros   the cost map to use for assigning costs to trajectories
   */
  GraphPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the Graph Planner object
   */
  ~GraphPlanner();

  /**
   * @brief Planner initialization
   * @param name        planner name
   * @param costmapRos  costmap ROS wrapper
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos);

  /**
   * @brief Planner initialization
   * @param name      planner name
   */
  void initialize(std::string name);

  /**
   * @brief Plan a path given start and goal in world map
   * @param start start in world map
   * @param goal  goal in world map
   * @param plan  plan
   * @return true if find a path successfully, else false
   */
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Plan a path given start and goal in world map
   * @param start     start in world map
   * @param goal      goal in world map
   * @param plan      plan
   * @param tolerance error tolerance
   * @return true if find a path successfully, else false
   */
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Publish planning path
   * @param path  planning path
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Regeister planning service
   * @param req   request from client
   * @param resp  response from server
   */
  bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

protected:
  /**
   * @brief publish expand zone
   * @param expand  set of expand nodes
   */
  void _publishExpand(std::vector<global_planner::Node>& expand);

  /**
   * @brief Calculate plan from planning path
   * @param path  path generated by global planner
   * @param plan  plan transfromed from path, i.e. [start, ..., goal]
   * @return  bool true if successful, else false
   */
  bool _getPlanFromPath(std::vector<global_planner::Node>& path, std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Tranform from costmap(x, y) to world map(x, y)
   * @param mx  costmap x
   * @param my  costmap y
   * @param wx  world map x
   * @param wy  world map y
   */
  void _mapToWorld(double mx, double my, double& wx, double& wy);

  /**
   * @brief Tranform from world map(x, y) to costmap(x, y)
   * @param mx  costmap x
   * @param my  costmap y
   * @param wx  world map x
   * @param wy  world map y
   * @return true if successfull, else false
   */
  bool _worldToMap(double wx, double wy, double& mx, double& my);

protected:
  bool initialized_;                          // initialization flag
  std::string planner_name_;                  // planner name
  costmap_2d::Costmap2D* costmap_;            // costmap
  costmap_2d::Costmap2DROS* costmap_ros_;     // costmap(ROS wrapper)
  global_planner::GlobalPlanner* g_planner_;  // global graph planner
  std::string frame_id_;                      // costmap frame ID
  unsigned int nx_, ny_;                      // costmap size
  double origin_x_, origin_y_;                // costmap origin
  double resolution_;                         // costmap resolution
  ros::Publisher plan_pub_;                   // path planning publisher
  ros::Publisher expand_pub_;                 // nodes explorer publisher
  ros::ServiceServer make_plan_srv_;          // planning service

private:
  boost::mutex mutex_;     // thread mutex
  double convert_offset_;  // offset of transform from world(x,y) to grid map(x,y)
  double tolerance_;       // tolerance
  double factor_;          // obstacle inflation factor
  bool is_outline_;        // whether outline the boudary of map
  bool is_expand_;         // whether publish expand map or not
};
}  // namespace graph_planner
#endif