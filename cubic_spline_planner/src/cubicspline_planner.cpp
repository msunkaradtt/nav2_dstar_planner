#include <cmath>
#include <string>
#include <memory>
#include <iterator> 

#include "nav2_util/node_utils.hpp"

#include "cubic_spline_planner/cubicspline_planner.hpp"

namespace cubic_spline_planner{
    void CubicSplinePlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        count = 150;

        startendAdd = false;

        // Parameter initialization
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    }

    void CubicSplinePlanner::cleanup()
    {
        RCLCPP_INFO(
            node_->get_logger(), "CleaningUp plugin %s of type CubicSplinePlanner",
            name_.c_str());
    }

    void CubicSplinePlanner::activate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Activating plugin %s of type CubicSplinePlanner",
            name_.c_str());
    }

    void CubicSplinePlanner::deactivate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Deactivating plugin %s of type CubicSplinePlanner",
            name_.c_str());
    }

    nav_msgs::msg::Path CubicSplinePlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
    {
        nav_msgs::msg::Path path, mainPath;

        //RCLCPP_INFO(node_->get_logger(), "[CubicSplinePlanner-Log] %f, %f", start.pose.position.x, start.pose.position.y);
        //RCLCPP_INFO(node_->get_logger(), "[CubicSplinePlanner-Log] %f, %f", goal.pose.position.x, goal.pose.position.y);

        if(!startendAdd){
            xs.insert(xs.begin(), start.pose.position.x);
            xs.push_back(goal.pose.position.x);

            //tangentsIn.insert(tangentsIn.begin(), {0.0, 0.0});
            //tangentsIn.push_back({0.0, 0.0});

            //tangentsOut.insert(tangentsOut.begin(), {0.0, 0.0});
            //tangentsOut.push_back({0.0, 0.0});

            ys.insert(ys.begin(), start.pose.position.y);
            ys.push_back(goal.pose.position.y);

            startendAdd = true;
        }

        path.poses.clear();
        path.header.stamp = node_->now();
        path.header.frame_id = global_frame_;

        std::pair<std::vector<double>, std::vector<double>> interpolatedPoints = cubic_spline_planner::Spline::InterpolateXY(xs, ys, count); //tangentsIn, tangentsOut,
        for (size_t i = 0; i <= interpolatedPoints.first.size(); i++) {
            //RCLCPP_INFO(node_->get_logger(), "[CubicSplinePlanner-Log] %f, %f", interpolatedPoints.first[i], interpolatedPoints.second[i]);
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = interpolatedPoints.first[i];
            pose.pose.position.y = interpolatedPoints.second[i];
            pose.pose.position.z = 0.0;
            
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;

            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            path.poses.push_back(pose);
        }

        return path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cubic_spline_planner::CubicSplinePlanner, nav2_core::GlobalPlanner)