#ifndef CUBIC_SPLINE_PLANNER__CUBICSPLINE_PLANNER_HPP_
#define CUBIC_SPLINE_PLANNER__CUBICSPLINE_PLANNER_HPP_

#include <string>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "cubic_spline_planner/spline.hpp"


namespace cubic_spline_planner{
    class CubicSplinePlanner : public nav2_core::GlobalPlanner{
        public:
        CubicSplinePlanner() = default;
        ~CubicSplinePlanner() = default;

        virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

        virtual void cleanup();
        virtual void activate();
        virtual void deactivate();

        virtual nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal);

        private:
        std::shared_ptr<tf2_ros::Buffer> tf_;
        nav2_util::LifecycleNode::SharedPtr node_;

        nav2_costmap_2d::Costmap2D * costmap_;
        std::string global_frame_, name_;

        double interpolation_resolution_;

        std::vector<double> xs = {22.0, 20.0, 18.0, 16.0, 14.0};
        std::vector<double> ys = {7.0, 7.0, 9.0, 9.0, 7.0};

        //std::vector<std::pair<double, double>> tangentsIn = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
        //std::vector<std::pair<double, double>> tangentsOut = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};

        //std::vector<double> xs = {-2.0, -1.5, -1.0, -0.5, 0.0};
        //std::vector<double> ys = {-0.44, -0.44, -0.40, -0.40, -0.44};

        int count;
        bool startendAdd;
    };
}


#endif //CUBIC_SPLINE_PLANNER__CUBICSPLINE_PLANNER_HPP_