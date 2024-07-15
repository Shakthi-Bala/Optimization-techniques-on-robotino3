
#include "evolutionary_planner.h"
#include <pluginlib/class_list_macros.h>

#include "aco.h"

PLUGINLIB_EXPORT_CLASS(evolutionary_planner::EvolutionaryPlanner, nav_core::BaseGlobalPlanner)

namespace evolutionary_planner
{
/**
 * @brief Construct a new Graph Planner object
 */
EvolutionaryPlanner::EvolutionaryPlanner() : initialized_(false), costmap_(nullptr), g_planner_(nullptr)
{
}

/**
 * @brief Construct a new Graph Planner object
 * @param name      planner name
 * @param costmap_ros   the cost map to use for assigning costs to trajectories
 */
EvolutionaryPlanner::EvolutionaryPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : EvolutionaryPlanner()
{
  initialize(name, costmap_ros);
}

/**
 * @brief Destroy the Graph Planner object
 */
EvolutionaryPlanner::~EvolutionaryPlanner()
{
  if (g_planner_)
  {
    delete g_planner_;
    g_planner_ = NULL;
  }
}

/**
 * @brief  Planner initialization
 * @param  name         planner name
 * @param  costmapRos   costmap ROS wrapper
 */
void EvolutionaryPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos)
{
  initialize(name, costmapRos->getCostmap(), costmapRos->getGlobalFrameID());
}

/**
 * @brief Planner initialization
 * @param name      planner name
 * @param costmap   costmap pointer
 * @param frame_id  costmap frame ID
 */
void EvolutionaryPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
{
  if (!initialized_)
  {
    initialized_ = true;

    // initialize ROS node
    ros::NodeHandle private_nh("~/" + name);

    // initialize costmap
    costmap_ = costmap;

    // costmap frame ID
    frame_id_ = frame_id;

    // get costmap properties
    nx_ = costmap->getSizeInCellsX(), ny_ = costmap->getSizeInCellsY();
    origin_x_ = costmap_->getOriginX(), origin_y_ = costmap_->getOriginY();
    resolution_ = costmap->getResolution();
    // ROS_WARN("nx: %d, origin_x: %f, res: %lf", nx_, origin_x_, resolution_);

    private_nh.param("convert_offset", convert_offset_, 0.0);  // offset of transform from world(x,y) to grid map(x,y)
    private_nh.param("default_tolerance", tolerance_, 0.0);    // error tolerance
    private_nh.param("outline_map", is_outline_, false);       // whether outline the map or not
    private_nh.param("obstacle_factor", factor_, 0.5);         // obstacle factor, NOTE: no use...

    // planner name
    std::string planner_name;
    private_nh.param("planner_name", planner_name, (std::string) "aco");
    if (planner_name == "aco")
    {
      int n_ants, max_iter;
      double alpha, beta, rho, Q;
      private_nh.param("n_ants", n_ants, 50);       // number of ants
      private_nh.param("alpha", alpha, 1.0);        // pheromone weight coefficient
      private_nh.param("beta", beta, 5.0);          // heuristic factor weight coefficient
      private_nh.param("rho", rho, 0.1);            // evaporation coefficient
      private_nh.param("Q", Q, 1.0);                // pheromone gain
      private_nh.param("max_iter", max_iter, 200);  // maximum iterations

      g_planner_ = new global_planner::ACO(nx_, ny_, resolution_, n_ants, alpha, beta, rho, Q, max_iter);
    }

    ROS_INFO("Using global graph planner: %s", planner_name.c_str());

    // register planning publisher
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

    // register planning service
    make_plan_srv_ = private_nh.advertiseService("make_plan", &EvolutionaryPlanner::makePlanService, this);
  }
  else
  {
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }
}

/**
 * @brief plan a path given start and goal in world map
 * @param start start in world map
 * @param goal  goal in world map
 * @param plan  plan
 * @return true if find a path successfully, else false
 */
bool EvolutionaryPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan)
{
  return makePlan(start, goal, tolerance_, plan);
}

/**
 * @brief Plan a path given start and goal in world map
 * @param start     start in world map
 * @param goal      goal in world map
 * @param plan      plan
 * @param tolerance error tolerance
 * @return true if find a path successfully, else false
 */
bool EvolutionaryPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                   double tolerance, std::vector<geometry_msgs::PoseStamped>& plan)
{
  // start thread mutex
  boost::mutex::scoped_lock lock(mutex_);
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  // clear existing plan
  plan.clear();

  // judege whether goal and start node in costmap frame or not
  if (goal.header.frame_id != frame_id_)
  {
    ROS_ERROR("The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
              frame_id_.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  if (start.header.frame_id != frame_id_)
  {
    ROS_ERROR("The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
              frame_id_.c_str(), start.header.frame_id.c_str());
    return false;
  }

  // get goal and strat node coordinate tranform from world to costmap
  double wx = start.pose.position.x, wy = start.pose.position.y;
  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if (!_worldToMap(wx, wy, m_start_x, m_start_y))
  {
    ROS_WARN(
        "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has "
        "been properly localized?");
    return false;
  }
  wx = goal.pose.position.x, wy = goal.pose.position.y;
  if (!_worldToMap(wx, wy, m_goal_x, m_goal_y))
  {
    ROS_WARN_THROTTLE(1.0,
                      "The goal sent to the global planner is off the global costmap. Planning will always fail to "
                      "this goal.");
    return false;
  }

  // tranform from costmap to grid map
  int g_start_x, g_start_y, g_goal_x, g_goal_y;
  g_planner_->map2Grid(m_start_x, m_start_y, g_start_x, g_start_y);
  g_planner_->map2Grid(m_goal_x, m_goal_y, g_goal_x, g_goal_y);

  // NOTE: how to init start and goal?
  global_planner::Node start_node(g_start_x, g_start_y, 0, 0, g_planner_->grid2Index(g_start_x, g_start_y), 0);
  global_planner::Node goal_node(g_goal_x, g_goal_y, 0, 0, g_planner_->grid2Index(g_goal_x, g_goal_y), 0);

  // clear the cost of robot location
  costmap_->setCost(g_start_x, g_start_y, costmap_2d::FREE_SPACE);

  // outline the map
  if (is_outline_)
    g_planner_->outlineMap(costmap_->getCharMap());

  // calculate path
  std::vector<global_planner::Node> path;
  std::vector<global_planner::Node> expand;
  bool path_found = g_planner_->plan(costmap_->getCharMap(), start_node, goal_node, path, expand);

  if (path_found)
  {
    if (_getPlanFromPath(path, plan))
    {
      geometry_msgs::PoseStamped goal_copy = goal;
      goal_copy.header.stamp = ros::Time::now();
      plan.push_back(goal_copy);
    }
    else
    {
      ROS_ERROR("Failed to get a plan from path when a legal path was found. This shouldn't happen.");
    }
  }
  else
  {
    ROS_ERROR("Failed to get a path.");
  }

  // publish visulization plan
  publishPlan(plan);

  return !plan.empty();
}

/**
 * @brief publish planning path
 * @param path  planning path
 */
void EvolutionaryPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  // create visulized path plan
  nav_msgs::Path gui_plan;
  gui_plan.poses.resize(plan.size());
  gui_plan.header.frame_id = frame_id_;
  gui_plan.header.stamp = ros::Time::now();
  for (unsigned int i = 0; i < plan.size(); i++)
    gui_plan.poses[i] = plan[i];

  // publish plan to rviz
  plan_pub_.publish(gui_plan);
}

/**
 * @brief Regeister planning service
 * @param req   request from client
 * @param resp  response from server
 * @return true
 */
bool EvolutionaryPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;

  return true;
}

/**
 * @brief Calculate plan from planning path
 * @param path  path generated by global planner
 * @param plan  plan transfromed from path, i.e. [start, ..., goal]
 * @return  bool true if successful, else false
 */
bool EvolutionaryPlanner::_getPlanFromPath(std::vector<global_planner::Node>& path, std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  std::string globalFrame = frame_id_;
  ros::Time planTime = ros::Time::now();
  plan.clear();

  for (int i = path.size() - 1; i >= 0; i--)
  {
    double wx, wy;
    _mapToWorld((double)path[i].x_, (double)path[i].y_, wx, wy);

    // coding as message type
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  return !plan.empty();
}

/**
 * @brief Tranform from costmap(x, y) to world map(x, y)
 * @param mx  costmap x
 * @param my  costmap y
 * @param wx  world map x
 * @param wy  world map y
 */
void EvolutionaryPlanner::_mapToWorld(double mx, double my, double& wx, double& wy)
{
  wx = origin_x_ + (mx + convert_offset_) * resolution_;
  wy = origin_y_ + (my + convert_offset_) * resolution_;
}

/**
 * @brief Tranform from world map(x, y) to costmap(x, y)
 * @param mx  costmap x
 * @param my  costmap y
 * @param wx  world map x
 * @param wy  world map y
 * @return true if successfull, else false
 */
bool EvolutionaryPlanner::_worldToMap(double wx, double wy, double& mx, double& my)
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (wx - origin_x_) / resolution_ - convert_offset_;
  my = (wy - origin_y_) / resolution_ - convert_offset_;
  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
    return true;

  return false;
}

}  // namespace evolutionary_planner
