#include <cmath>
#include <string>
#include <memory>
#include <iterator> 

#include "nav2_util/node_utils.hpp"

#include "cubic_spline_planner/cubic_spline_interpolator.hpp"
#include "cubic_spline_planner/cubicspline_planner.hpp"

namespace cubic_spline_planner{
    void CubicSplinePlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        pointsPerUnity = 5;
        skipPoints = 1;
        useEndConditions = true;
        useMiddleConditions = false;

        cubic_spline_planner::UserPoses a;
        a.x = 22.0;
        a.y = 7.0;
        a.yaw = 135.0;
        poseList.push_back(a);
        a.x = 20.5;
        a.y = 5.0;
        a.yaw = -180.0;
        poseList.push_back(a);
        a.x = 18.0;
        a.y = 5.0;
        a.yaw = -180.0;
        poseList.push_back(a);
        a.x = 18.0;
        a.y = 5.0;
        a.yaw = -180.0;
        poseList.push_back(a);
        a.x = -18.0;
        a.y = 5.0;
        a.yaw = -180.0;
        poseList.push_back(a);
        a.x = -20.5;
        a.y = 5.0;
        a.yaw = -180.0;
        poseList.push_back(a);
        a.x = -22.0;
        a.y = 7.0;
        a.yaw = 45.0;
        poseList.push_back(a);

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

    auto CubicSplinePlanner::createQuaternionMsgFromYaw(double yaw){
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }

    nav_msgs::msg::Path CubicSplinePlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
    {
        nav_msgs::msg::Path path, smoothedPath;

        path.poses.clear();
        path.header.stamp = node_->now();
        path.header.frame_id = global_frame_;

        smoothedPath.header.stamp = node_->now();
        smoothedPath.header.frame_id = global_frame_;

        geometry_msgs::msg::PoseStamped pose_;

        cubic_spline_planner::CubicSplineInterpolator csi(pointsPerUnity, skipPoints, useEndConditions, useMiddleConditions);

        for(UserPoses po : poseList){
            pose_.header.stamp = node_->now();
            pose_.header.frame_id = global_frame_;

            pose_.pose.position.x = static_cast<double>(po.x);
            pose_.pose.position.y = static_cast<double>(po.y);
            pose_.pose.position.z = 0.0;
            pose_.pose.orientation = createQuaternionMsgFromYaw(po.yaw);
            path.poses.push_back(pose_);
        }

        csi.interpolatePath(path, smoothedPath);


        return smoothedPath;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cubic_spline_planner::CubicSplinePlanner, nav2_core::GlobalPlanner)