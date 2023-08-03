#ifndef NAV2_DSTAR_PLANNER_V1__DSTAR_PLANNER_V1_HPP_
#define NAV2_DSTAR_PLANNER_V1__DSTAR_PLANNER_V1_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_core/exceptions.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav2_dstar_planner_v1/d_star.hpp"

namespace nav2_dstar_planner_v1{
    class DStarPlannerV1 : public nav2_core::GlobalPlanner{
        public:
        DStarPlannerV1();
        ~DStarPlannerV1();

        void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void cleanup() override;
        void activate() override;
        void deactivate() override;

        nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

        bool isPlannerOutOfDate();

        protected:
        bool makePlan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan);
        bool _getPlanFromPath(std::vector<nav2_dstar_planner_v1::Node>& path, nav_msgs::msg::Path & plan);

        void mapToWorld(double mx, double my, double & wx, double & wy);
        bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
        void clearRobotCell(unsigned int mx, unsigned int my);

        protected:

        std::string name_;

        nav2_costmap_2d::Costmap2D * costmap_;
        nav2_costmap_2d::Costmap2DROS* costmap_ros_;

        std::string frame_id_;

        unsigned int nx_, ny_;
        double origin_x_, origin_y_;
        double resolution_;

        public:
        std::unique_ptr<DStar> planner_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Logger logger_{rclcpp::get_logger("DStarPlannerV1")};

        bool allow_unknown_;

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

        private:
    }; 
}

#endif //NAV2_DSTAR_PLANNER_V1__DSTAR_PLANNER_V1_HPP_