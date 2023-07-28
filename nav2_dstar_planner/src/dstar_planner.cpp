#include "nav2_dstar_planner/dstar_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_dstar_planner/dstar.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_dstar_planner{
    DstarPlanner::DstarPlanner() : tf_(nullptr), costmap_(nullptr){

    }

    DstarPlanner::~DstarPlanner(){
        RCLCPP_INFO(logger_, "Destroying plugin %s of type DstarPlanner", name_.c_str());
    }

    void DstarPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
        tf_ = tf;
        name_ = name;
        costmap_ = costmap_ros->getCostmap();

        global_frame_ = costmap_ros->getGlobalFrameID();

        node_ = parent;
        auto node = parent.lock();
        clock_ = node->get_clock();
        logger_ = node->get_logger();

        RCLCPP_INFO(logger_, "Configuring plugin %s of type DstarPlanner", name_.c_str());

        declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
        node->get_parameter(name + ".tolerance", tolerance_);
        declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
        node->get_parameter(name + ".allow_unknown", allow_unknown_);
        declare_parameter_if_not_declared(node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
        node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);

        planner_ = std::make_unique<Dstar>(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    }

    void DstarPlanner::activate(){
        RCLCPP_INFO(logger_, "Activating plugin %s of type DstarPlanner", name_.c_str());
        auto node = node_.lock();
        dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&DstarPlanner::dynamicParametersCallback, this, _1));
    }

    void DstarPlanner::deactivate(){
        RCLCPP_INFO(logger_, "Cleaning up plugin %s of type DstarPlanner", name_.c_str());
        dyn_params_handler_.reset();
    }

    void DstarPlanner::cleanup(){
        RCLCPP_INFO(logger_, "Cleaning up plugin %s of type DstarPlanner", name_.c_str());
        planner_.reset();
    }

    nav_msgs::msg::Path DstarPlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, 
    const geometry_msgs::msg::PoseStamped & goal){
        unsigned int mx_start, my_start, mx_goal, my_goal;
        if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
            RCLCPP_ERROR(logger_, "Start Coordinates was outside bounds");   
        }
        if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
            RCLCPP_ERROR(logger_, "Goal Coordinates was outside bounds");
        }
        if (costmap_->getCost(mx_start, my_start) == nav2_costmap_2d::LETHAL_OBSTACLE) {
            RCLCPP_ERROR(logger_, "Start Coordinates was in lethal cost");
        }
        if (tolerance_ == 0 && costmap_->getCost(mx_goal, my_goal) == nav2_costmap_2d::LETHAL_OBSTACLE) {
            RCLCPP_ERROR(logger_, "Goal Coordinates was in lethal cost");
        }

        if (isPlannerOutOfDate()) {
            planner_->setNavArr(
                costmap_->getSizeInCellsX(),
                costmap_->getSizeInCellsY()
            );
        }

        nav_msgs::msg::Path path;

        if (start.pose.position.x == goal.pose.position.x && start.pose.position.y == goal.pose.position.y){
            path.header.stamp = clock_->now();
            path.header.frame_id = global_frame_;
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.z = 0.0;

            pose.pose = start.pose;

            if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
                pose.pose.orientation = goal.pose.orientation;
            }

            path.poses.push_back(pose);
            return path;
        }

        if (!makePlan(start.pose, goal.pose, tolerance_, path)) {
            RCLCPP_ERROR(logger_, "Goal Coordinates was in lethal cost");
        }

        return path;
    }

    bool DstarPlanner::isPlannerOutOfDate(){
        if (!planner_.get() || 
        planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) || 
        planner_->ny != static_cast<int>(costmap_->getSizeInCellsY())){
            return true;
        }
        return false;
    }

    bool DstarPlanner::makePlan(const geometry_msgs::msg::Pose & start, 
    const geometry_msgs::msg::Pose & goal, 
    double tolerance, 
    nav_msgs::msg::Path & plan){
        plan.poses.clear();

        plan.header.stamp = clock_->now();
        plan.header.frame_id = global_frame_;

        double wx = start.position.x;
        double wy = start.position.y;

        RCLCPP_DEBUG(logger_, "Making plan from (%.2f,%.2f) to (%.2f,%.2f)", start.position.x, start.position.y, goal.position.x, goal.position.y);

        unsigned int mx, my;
        worldToMap(wx, wy, mx, my);

        clearRobotCell(mx, my);

        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

        planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

        planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

        lock.unlock();

        int map_start[2];
        map_start[0] = mx;
        map_start[1] = my;

        wx = goal.position.x;
        wy = goal.position.y;

        worldToMap(wx, wy, mx, my);
        int map_goal[2];
        map_goal[0] = mx;
        map_goal[1] = my;

        planner_->setStart(map_goal);
        planner_->setGoal(map_start);

        planner_->calcDstar();

        double resolution = costmap_->getResolution();
        geometry_msgs::msg::Pose p, best_pose;

        bool found_legal = false;

        p = goal;
        double potential = getPointPotential(p.position);
        if (potential < POT_HIGH) {
            best_pose = p;
            found_legal = true;
        } else {
            double best_sdist = std::numeric_limits<double>::max();
            p.position.y = goal.position.y - tolerance;
            while (p.position.y <= goal.position.y + tolerance) {
                p.position.x = goal.position.x - tolerance;
                while (p.position.x <= goal.position.x + tolerance) {
                    potential = getPointPotential(p.position);
                    double sdist = squared_distance(p, goal);
                    if (potential < POT_HIGH && sdist < best_sdist) {
                        best_sdist = sdist;
                        best_pose = p;
                        found_legal = true;
                    }
                    p.position.x += resolution;
                }
                p.position.y += resolution;
            }
        }

        if (found_legal) {
            if (getPlanFromPotential(best_pose, plan)) {
                smoothApproachToGoal(best_pose, plan);

                if (use_final_approach_orientation_) {
                    size_t plan_size = plan.poses.size();

                    if (plan_size == 1) {
                        plan.poses.back().pose.orientation = start.orientation;
                    } else if (plan_size > 1) {
                        double dx, dy, theta;
                        auto last_pose = plan.poses.back().pose.position;
                        auto approach_pose = plan.poses[plan_size - 2].pose.position;
                        if (std::abs(last_pose.x - approach_pose.x) < 0.0001 && std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2){
                            approach_pose = plan.poses[plan_size - 3].pose.position;
                        }
                        dx = last_pose.x - approach_pose.x;
                        dy = last_pose.y - approach_pose.y;
                        theta = atan2(dy, dx);
                        plan.poses.back().pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
                    }
                }
            } else {
                RCLCPP_ERROR(logger_, "Failed to create a plan from potential when a legal" " potential was found. This shouldn't happen.");
            }
        }

        return !plan.poses.empty();
    }

    void DstarPlanner::smoothApproachToGoal(const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan){
        if (plan.poses.size() >= 2) {
            auto second_to_last_pose = plan.poses.end()[-2];
            auto last_pose = plan.poses.back();
            if (squared_distance(last_pose.pose, second_to_last_pose.pose) > squared_distance(goal, second_to_last_pose.pose)){
                plan.poses.back().pose = goal;
                return;
            }
        }
        geometry_msgs::msg::PoseStamped goal_copy;
        goal_copy.pose = goal;
        plan.poses.push_back(goal_copy);
    }

    bool DstarPlanner::getPlanFromPotential(const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan){
        plan.poses.clear();

        double wx = goal.position.x;
        double wy = goal.position.y;

        unsigned int mx, my;
        worldToMap(wx, wy, mx, my);

        int map_goal[2];
        map_goal[0] = mx;
        map_goal[1] = my;

        planner_->setStart(map_goal);

        const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ? 
        (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

        int path_len = planner_->calcPath(max_cycles);
        if (path_len == 0) {
            return false;
        }

        auto cost = planner_->getLastPathCost();
        RCLCPP_DEBUG(logger_, "Path found, %d steps, %f cost\n", path_len, cost);

        float * x = planner_->getPathX();
        float * y = planner_->getPathY();

        int len = planner_->getPathLen();

        for (int i = len - 1; i >= 0; --i) {
            double world_x, world_y;
            mapToWorld(x[i], y[i], world_x, world_y);

            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.poses.push_back(pose);
        }

        return !plan.poses.empty();
    }

    double DstarPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point){
        unsigned int mx, my;
        if (!worldToMap(world_point.x, world_point.y, mx, my)) {
            return std::numeric_limits<double>::max();
        }

        unsigned int index = my * planner_->nx + mx;
        return planner_->potarr[index];
    }

    bool DstarPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my){
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

    void DstarPlanner::mapToWorld(double mx, double my, double & wx, double & wy){
        wx = costmap_->getOriginX() + mx * costmap_->getResolution();
        wy = costmap_->getOriginY() + my * costmap_->getResolution();
    }

    void DstarPlanner::clearRobotCell(unsigned int mx, unsigned int my){
        costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
    }

    rcl_interfaces::msg::SetParametersResult DstarPlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters){
        rcl_interfaces::msg::SetParametersResult result;
        for (auto parameter : parameters) {
            const auto & type = parameter.get_type();
            const auto & name = parameter.get_name();

            if (type == ParameterType::PARAMETER_DOUBLE) {
                if (name == name_ + ".tolerance") {
                    tolerance_ = parameter.as_double();
                } 
            } else if (type == ParameterType::PARAMETER_BOOL) {
                if (name == name_ + ".allow_unknown") {
                    allow_unknown_ = parameter.as_bool();
                } else if (name == name_ + ".use_final_approach_orientation") {
                    use_final_approach_orientation_ = parameter.as_bool();
                }
            }
        }

        result.successful = true;
        return result;
    }
} // namespace nav2_dstar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_dstar_planner::DstarPlanner, nav2_core::GlobalPlanner)