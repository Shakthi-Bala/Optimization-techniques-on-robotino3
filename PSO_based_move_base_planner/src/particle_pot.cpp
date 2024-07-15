/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include "particle_potential.h"
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include "global_planner/astar.h"
#include "global_planner/grid_path.h"
#include "global_planner/dijkstra.h"
#include <nav_msgs/Path>
#include <nav_msgs/OccupancyGrid>
#include <geometry_msgs/PointStamped>
#include <visualization_msgs/Marker>

static int ct = 0;

// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(Pso_pot_planner::PsoPotPlannerROS, nav_core::BaseGlobalPlanner)

namespace Pso_pot_planner
{
    void PsoPotPlannerROS::outlineMap(unsigned char *costarr, int nx, int ny, unsigned char value)
    {
        unsigned char *pc = costarr;
        for (int i = 0; i < nx; i++)
            *pc++ = value;
        pc = costarr + (ny - 1) * nx;
        for (int i = 0; i < nx; i++)
            *pc++ = value;
        pc = costarr;
        for (int i = 0; i < ny; i++, pc += nx)
            *pc = value;
        pc = costarr + nx - 1;
        for (int i = 0; i < ny; i++, pc += nx)
            *pc = value;
    }

    PsoPotPlannerROS::PsoPotPlannerROS() : costmap_(NULL), initialized_(false), allow_unknown_(true)
    {
    }

    PsoPotPlannerROS::PsoPotPlannerROS(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id) : costmap_(NULL), initialized_(false), allow_unknown_(true)
    {
        // Initialize the planner
        initialize(name, costmap, frame_id);
    }

    PsoPotPlannerROS::~PsoPotPlannerROS()
    {
        if (p_calc_)
            delete p_calc_;
        if (planner_)
            delete planner_;
        if (path_maker_)
            delete path_maker_;
        if (dsrv_)
            delete dsrv_;
    }

    void PsoPotPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
    }

    void PsoPotPlannerROS::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id)
    {
        if (!initialized_)
        {
            ros::NodeHandle private_nh("~/" + name);
            costmap_ = costmap;
            frame_id_ = frame_id;

            unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();
            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();

            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();

            convert_offset_ = 0.5;
            p_calc_ = new PotentialCalculator(cx, cy);

            planner_ = new AStarExpansion(p_calc_, cx, cy);
            path_maker_ = new GridPath(p_calc_);

            orientation_filter_ = new OrientationFilter();

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);
            marker_pub1 = private_nh.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
            marker_pub2 = private_nh.advertise<visualization_msgs::Marker>("visualization_marker2", 10);

            private_nh.param("allow_unknown", allow_unknown_, true);
            planner_->setHasUnknown(allow_unknown_);
            private_nh.param("publish_scale", publish_scale_, 100);

            make_plan_srv_ = private_nh.advertiseService("make_plan", &PsoPotPlannerROS::makePlanService, this);

            dsrv_ = new dynamic_reconfigure::Server<Pso_pot_planner::PsoPotPlannerROSConfig>(ros::NodeHandle("~/" + name));
            dynamic_reconfigure::Server<Pso_pot_planner::PsoPotPlannerROSConfig>::CallbackType cb = boost::bind(
                &PsoPotPlannerROS::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);

            ROS_INFO("PSO Pot Planner initialized successfully");
            initialized_ = true;
        }
        else
            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }

    void PsoPotPlannerROS::reconfigureCB(Pso_pot_planner::PsoPotPlannerROSConfig &config, uint32_t level)
    {
        planner_->setLethalCost(config.lethal_cost);
        path_maker_->setLethalCost(config.lethal_cost);
        planner_->setNeutralCost(config.neutral_cost);
        planner_->setFactor(config.cost_factor);
        publish_potential_ = config.publish_potential;
    }

    bool PsoPotPlannerROS::makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
    {
        return makePlan(req.start, req.goal, resp.plan.poses);
    }

    bool PsoPotPlannerROS::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                    std::vector<geometry_msgs::PoseStamped> &plan)
    {
        // Clear the plan
        plan.clear();

        if (!initialized_)
        {
            ROS_ERROR("Global planner is not initialized");
            return false;
        }

        // Transform the start and goal
        geometry_msgs::PoseStamped start_transformed, goal_transformed;
        if (!transformPose(start, start_transformed) || !transformPose(goal, goal_transformed))
        {
            ROS_ERROR("Could not transform poses");
            return false;
        }

        // Call the planner
        std::vector<geometry_msgs::PoseStamped> path;
        if (!planner_->getPath(start_transformed, goal_transformed, path))
        {
            ROS_INFO("PSO Pot Planner failed to find a path");
            return false;
        }

        // Publish the plan for visualization purposes
        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = frame_id_;

        for (unsigned int i = 0; i < path.size(); i++)
            resp.plan.poses.push_back(path[i]);

        plan = path;

        return true;
    }

    bool PsoPotPlannerROS::transformPose(const geometry_msgs::PoseStamped &pose_in, geometry_msgs::PoseStamped &pose_out)
    {
        // Transform a pose from the planner frame to the frame of the costmap
        try
        {
            tf_->transform(pose_in, pose_out, frame_id_);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("Failed to transform the goal pose from %s to %s", pose_in.header.frame_id.c_str(),
                      frame_id_.c_str());
            return false;
        }
        return true;
    }

    void PsoPotPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before using this planner");
            return;
        }

        // Publish the plan for visualization purposes
        if (plan_pub_.getNumSubscribers() > 0)
        {
            nav_msgs::Path gui_path;
            gui_path.poses.resize(path.size());
            if (!path.empty())
            {
                gui_path.header.frame_id = path[0].header.frame_id;
                gui_path.header.stamp = path[0].header.stamp;
            }
            for (unsigned int i = 0; i < path.size(); i++)
                gui_path.poses[i] = path[i];

            plan_pub_.publish(gui_path);
        }
    }

    void PsoPotPlannerROS::publishPotential(float *potential)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before using this planner");
            return;
        }

        // Publish the potential array
        if (potential_pub_.getNumSubscribers() > 0)
        {
            nav_msgs::OccupancyGrid grid;
            grid.header.stamp = ros::Time::now();
            grid.header.frame_id = frame_id_;

            grid.info.width = width;
            grid.info.height = height;
            grid.info.resolution = resolution;

            float *true_potential = p_calc_->getPotArray();

            if (!true_potential)
                return;

            grid.info.origin.position.x = originX;
            grid.info.origin.position.y = originY;
            grid.info.origin.orientation.w = 1.0;

            grid.data.resize(grid.info.width * grid.info.height);

            // Subtract the offset to the original values
            float max = -1.0;
            for (unsigned int i = 0; i < width * height; i++)
            {
                true_potential[i] = potential[i];
                if (true_potential[i] > max)
                    max = true_potential[i];
            }

            if (max == 0.0)
                max = 1.0;

            // Finally, we'll normalize the values
            for (unsigned int i = 0; i < width * height; i++)
            {
                // We're going to threshold the potential values so that
                // we don't see points which are less than 90% of the max
                // We'll also artificially inflate the values along the path
                // so that they're visible
                if (true_potential[i] > max * 0.1)
                    grid.data[i] = true_potential[i] * publish_scale_;
                else
                    grid.data[i] = 0;
            }

            potential_pub_.publish(grid);
        }
    }
} // end namespace
