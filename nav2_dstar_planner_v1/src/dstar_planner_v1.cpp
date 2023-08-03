#include "nav2_dstar_planner_v1/dstar_planner_v1.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_dstar_planner_v1/d_star.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace nav2_dstar_planner_v1{
    DStarPlannerV1::DStarPlannerV1() : tf_(nullptr), costmap_(nullptr){

    }

    DStarPlannerV1::~DStarPlannerV1(){
        RCLCPP_INFO(logger_, "Destroying plugin %s of type DStarPlannerV1", name_.c_str());
    }

    void DStarPlannerV1::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
        tf_ = tf;
        name_ = name;
        costmap_ = costmap_ros->getCostmap();

        frame_id_ = costmap_ros->getGlobalFrameID();

        node_ = parent;
        auto node = parent.lock();
        clock_ = node->get_clock();
        logger_ = node->get_logger();

        RCLCPP_INFO(logger_, "Configuring plugin %s of type DStarPlannerV1-logs", name_.c_str());

        allow_unknown_ = true;

        planner_ = std::make_unique<DStar>(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    }

    void DStarPlannerV1::activate(){
        RCLCPP_INFO(logger_, "Activating plugin %s of type DStarPlannerV1", name_.c_str());
    }

    void DStarPlannerV1::deactivate(){
        RCLCPP_INFO(logger_, "Cleaning up plugin %s of type DStarPlannerV1", name_.c_str());
    }

    void DStarPlannerV1::cleanup(){
        RCLCPP_INFO(logger_, "Cleaning up plugin %s of type DStarPlannerV1", name_.c_str());
        planner_.reset();
    }

    nav_msgs::msg::Path DStarPlannerV1::createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal){
        unsigned int mx_start, my_start, mx_goal, my_goal;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)){
            RCLCPP_ERROR(logger_, "Start Coordinates was outside bounds");
        }
        if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
            RCLCPP_ERROR(logger_, "Goal Coordinates was outside bounds");
        }

        if(isPlannerOutOfDate()){
            //planner_->reset();
            planner_->setNavArr(
                costmap_->getSizeInCellsX(),
                costmap_->getSizeInCellsY()
            );
        }

        nav_msgs::msg::Path path;

        if (start.pose.position.x == goal.pose.position.x && start.pose.position.y == goal.pose.position.y){
            path.header.stamp = clock_->now();
            path.header.frame_id = frame_id_;
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.z = 0.0;

            pose.pose = start.pose;
            if (start.pose.orientation != goal.pose.orientation) {
                pose.pose.orientation = goal.pose.orientation;
            }

            path.poses.push_back(pose);
            return path;
        }

        if(!makePlan(start.pose, goal.pose, path)){
            RCLCPP_ERROR(logger_, "Goal Coordinates was in lethal cost");
        }

        return path;
    }

    bool DStarPlannerV1::isPlannerOutOfDate(){
        if (!planner_.get() ||
        planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) || 
        planner_->ny != static_cast<int>(costmap_->getSizeInCellsY())){
            return true;
        }
        return false;
    }

    bool DStarPlannerV1::makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    nav_msgs::msg::Path & plan){
        plan.poses.clear();

        plan.header.stamp = clock_->now();
        plan.header.frame_id = frame_id_;

        double wx = start.position.x;
        double wy = start.position.y;

        unsigned int mx, my;
        worldToMap(wx, wy, mx, my);

        clearRobotCell(mx, my);

        /*std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

        planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
        
        lock.unlock();*/

        int map_start[2];
        map_start[0] = mx;
        map_start[1] = my;

        wx = goal.position.x;
        wy = goal.position.y;

        worldToMap(wx, wy, mx, my);

        RCLCPP_INFO(logger_, "%d, %d DStar", mx, my);
        int map_goal[2];
        map_goal[0] = mx;
        map_goal[1] = my;

        int g_start_x, g_start_y, g_goal_x, g_goal_y;
        planner_->map2Grid(map_start[0], map_start[1], g_start_x, g_start_y);
        planner_->map2Grid(map_goal[0], map_goal[1], g_goal_x, g_goal_y);

        //RCLCPP_INFO(logger_, "%d, %d DStar", g_goal_x, g_goal_y);

        nav2_dstar_planner_v1::Node start_node(g_start_x, g_start_y, 0, 0, planner_->grid2Index(g_start_x, g_start_y), 0);
        nav2_dstar_planner_v1::Node goal_node(g_goal_x, g_goal_y, 0, 0, planner_->grid2Index(g_goal_x, g_goal_y), 0);

        std::vector<nav2_dstar_planner_v1::Node> path;
        std::vector<nav2_dstar_planner_v1::Node> expand;

        bool path_found = false;

        path_found = planner_->plan(costmap_->getCharMap(), start_node, goal_node, path, expand);

        if(path_found){
            if(_getPlanFromPath(path, plan)){
                geometry_msgs::msg::PoseStamped goalCopy;
                goalCopy.pose = goal;
                plan.poses.push_back(goalCopy);
            }
        }

        return !plan.poses.empty();
    }

    bool DStarPlannerV1::_getPlanFromPath(std::vector<nav2_dstar_planner_v1::Node>& path, nav_msgs::msg::Path & plan){
        plan.poses.clear();
        for(int i = path.size() - 1; i >= 0; i--){
            double wx, wy;
            mapToWorld((double)path[i].x_, (double)path[i].y_, wx, wy);
            //RCLCPP_INFO(logger_, "%f, %f DStar", wx, wy);
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.poses.push_back(pose);
        }

        return !plan.poses.empty();
    }

    void DStarPlannerV1::clearRobotCell(unsigned int mx, unsigned int my){
        costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
    }

    void DStarPlannerV1::mapToWorld(double mx, double my, double & wx, double & wy){
        wx = costmap_->getOriginX() + mx * costmap_->getResolution();
        wy = costmap_->getOriginY() + my * costmap_->getResolution();
    }

    bool DStarPlannerV1::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my){
        if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
            return false;
        }

        mx = static_cast<int>(std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
        my = static_cast<int>(std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

        if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
            return true;
        }

        RCLCPP_ERROR(logger_, "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my, costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

        return false;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_dstar_planner_v1::DStarPlannerV1, nav2_core::GlobalPlanner)