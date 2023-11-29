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

#include <iostream>
#include <fstream>
#include <sstream>

#include <filesystem>

namespace fs = std::filesystem;


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
        void scaleposx(double scaleVal);

        private:
        std::shared_ptr<tf2_ros::Buffer> tf_;
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

        rclcpp::Logger logger_{rclcpp::get_logger("CubicSplinePlanner")};

        nav2_costmap_2d::Costmap2D * costmap_;
        std::string global_frame_, name_;

        std::string csv_file_;
        double x_scale_;

        /*std::vector<double> xs = {0.0, 14.4464263916016, 16.6706962585449, 19.515531539917, 21.2098083496094, 30};
        std::vector<double> ys = {0.0, 0.0, 1.52817690372467, 1.52817690372467, 0.0, 0.0};

        std::vector<std::pair<double, double>> tangentsIn = {{-1.0, 0.0}, {-1.0, 0.0}, {-0.910575151443481, -0.0321578122675419}, {-1.0, 0.0}, {-1.0, 0.0}, {-1.0, 0.0}};
        std::vector<std::pair<double, double>> tangentsOut = {{1.0, 0.0}, {0.620000004768372, 0.0}, {1.0, 0.0}, {0.846666872501373, 0.0}, {1.0, 0.0}, {1.0, 0.0}};*/

        std::vector<double> xs_t = {};
        std::vector<double> ys_t = {};

        std::vector<double> xs_prime = {};
        std::vector<double> ys_prime = {};

        std::vector<std::pair<double, double>> tangentsIn_t = {};
        std::vector<std::pair<double, double>> tangentsOut_t = {};

        double startPointX;
        double startPointY;

        double goalPointX = 0.0;
        double goalPointY = 0.0;

        int count;
        bool startendAdd;
        bool diffState;

        int ticker;

        std::vector<double> diff;

        protected:
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
        rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
    };
}


#endif //CUBIC_SPLINE_PLANNER__CUBICSPLINE_PLANNER_HPP_