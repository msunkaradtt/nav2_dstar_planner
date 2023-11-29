#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"

#include "cubic_spline_planner/cubicspline_planner.hpp"

#include "builtin_interfaces/msg/duration.hpp"

#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using namespace std::chrono;  // NOLINT
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace cubic_spline_planner{
    void CubicSplinePlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
        node_ = parent;
        auto node = parent.lock();
        logger_ = node->get_logger();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        // Parameter initialization
        declare_parameter_if_not_declared(node, name_ + ".csv_file", rclcpp::ParameterValue(""));
        node->get_parameter(name_ + ".csv_file", csv_file_);
        declare_parameter_if_not_declared(node, name_ + ".x_scale", rclcpp::ParameterValue(0.0));
        node->get_parameter(name_ + ".x_scale", x_scale_);

        count = 200;

        startendAdd = false;
        diffState = false;
        ticker = 0;
        diff = {};

        startPointX = 0.0;
        startPointY = 0.0;

        fs::path csvPath(fs::current_path());
        csvPath /= "src";
        csvPath /= "cubic_spline_planner";
        csvPath /= "data";
        csvPath /= csv_file_;

        RCLCPP_INFO(logger_, "User-Log: %s", csvPath.c_str());

        std::ifstream file(csvPath.c_str());

        std::vector<std::vector<std::string>> rows;
        std::string line;

        while(std::getline(file, line)){
            std::vector<std::string> row;
            std::stringstream ss(line);
            std::string cell;

            while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
            }

            rows.push_back(row);
        }

        file.close();

        std::string lookf = "spline points\n";
        int splinePoints = 0;
        std::string lookf1 = "TangentIn.x";
        int startRow = 0;

        for(int i=0; i < rows.size(); i++){
            for(int j = 0; j < rows[i].size(); j++){
                if(std::strcmp(rows[i][j].c_str(), lookf.c_str()) == 3 ){
                    splinePoints = std::stoi(rows[i+1][0]);
                }

                if(lookf1.compare(rows[i][j]) == 0){
                    startRow = i+1;
                }
            }
        }

        int endRow = startRow + splinePoints;

        for(int k=startRow; k<endRow; k++){
            /*
            TagentIn
            */
           std::vector<std::string> tanIn = {rows[k].begin(), rows[k].begin() + 2};
           double tanInX = std::stod(tanIn[0]);
           double tanInY = std::stod(tanIn[1]);
           std::pair<double, double> TanIn = {tanInX, tanInY};
           tangentsIn_t.push_back(TanIn);

           /*
           TagentOut
           */ 
          std::vector<std::string> tanOut = {rows[k].begin() + 3, rows[k].begin() + 5};
          double tanOutX = std::stod(tanOut[0]);
          double tanOutY = std::stod(tanOut[1]);
          std::pair<double, double> TanOut = {tanOutX, tanOutY};
          tangentsOut_t.push_back(TanOut);

          /*
          Position_XY
          */
         std::vector<std::string> posXY = {rows[k].begin() + 6, rows[k].end()};
         double posX = std::stod(posXY[0]);
         double posY = std::stod(posXY[1]);
         xs_t.push_back(posX);
         ys_t.push_back(posY);
        }

        scaleposx(x_scale_);
    }

    void CubicSplinePlanner::scaleposx(double scaleVal){
        for(int i = 1; i < xs_t.size(); i++){
            xs_t[i] -= scaleVal; 
        }
    }

    void CubicSplinePlanner::cleanup()
    {
        RCLCPP_INFO(
            logger_, "CleaningUp plugin %s of type CubicSplinePlanner",
            name_.c_str());
    }

    void CubicSplinePlanner::activate()
    {
        RCLCPP_INFO(
            logger_, "Activating plugin %s of type CubicSplinePlanner",
            name_.c_str());
        
        auto node = node_.lock();
        dyn_params_handler_ = node->add_on_set_parameters_callback(std::bind(&CubicSplinePlanner::dynamicParametersCallback, this, _1));
    }

    void CubicSplinePlanner::deactivate()
    {
        RCLCPP_INFO(
            logger_, "Deactivating plugin %s of type CubicSplinePlanner",
            name_.c_str());

        dyn_params_handler_.reset();
    }

    nav_msgs::msg::Path CubicSplinePlanner::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
    {
        
        nav_msgs::msg::Path path, mainPath;

        auto node = node_.lock();

        if((goalPointX != goal.pose.position.x) && (goalPointY != goal.pose.position.y)){
            //RCLCPP_INFO(node_->get_logger(), "User-Log: %s", "Here_11111!");
            goalPointX = goal.pose.position.x;
            goalPointY = goal.pose.position.y;
            diffState = false;
        }

        path.poses.clear();
        path.header.stamp = node->now();
        path.header.frame_id = global_frame_;

        std::vector<cubic_spline_planner::InterpolatedPoint> interpolatedPoints = cubic_spline_planner::Spline::InterpolateXYWithYaw(xs_t, ys_t, tangentsIn_t, tangentsOut_t, count);
        
        
        if(!diffState){
            //RCLCPP_INFO(node_->get_logger(), "User-Log: %s", "Here!");
            ticker = 0;
            startPointX = start.pose.position.x;
            startPointY = start.pose.position.y;
            diff.clear();
            diff.push_back(0.0);
            double startx = interpolatedPoints[0].x;
            for(int i = 1; i < interpolatedPoints.size(); i++){
                double d = interpolatedPoints[i].x - startx;
                diff.push_back(d);
            }

            diffState = true;
        }

        for (int j = 0; j < interpolatedPoints.size(); j++) {

            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = startPointX - diff[j]; 
            pose.pose.position.y = interpolatedPoints[j].y + startPointY; 
            pose.pose.position.z = 0.0;

            tf2::Quaternion _quater;
            _quater.setRPY(0, 0, interpolatedPoints[j].yaw);
            _quater = _quater.normalize();

            pose.pose.orientation.x = _quater.getX();
            pose.pose.orientation.y = _quater.getY();
            pose.pose.orientation.z = _quater.getZ();
            pose.pose.orientation.w = _quater.getW();

            pose.header.stamp = node->now();
            pose.header.frame_id = global_frame_;
            path.poses.push_back(pose);
        }

        //RCLCPP_INFO(node_->get_logger(), "User-Log: %f", goal.pose.position.x);
        

        if(((start.pose.position.x - path.poses[path.poses.size() - 1].pose.position.x) <= 2.0f)){
            //RCLCPP_INFO(node_->get_logger(), "User-Log: %s", "triggred!");
            ticker += 1;
        }

        double app_goal_x = path.poses[path.poses.size() - 1].pose.position.x;
        double app_goal_y = path.poses[path.poses.size() - 1].pose.position.y;

        if((ticker >= 2)){
            //RCLCPP_INFO(node_->get_logger(), "User-Log: %s", "Final approch!");   
            
            int total_number_of_loop = std::hypot(app_goal_x - start.pose.position.x, app_goal_y - start.pose.position.y) / 0.1f;
            //RCLCPP_INFO(node_->get_logger(), "User-Log: %d", total_number_of_loop);
            double x_increment = (app_goal_x - start.pose.position.x) / total_number_of_loop;
            double y_increment = (app_goal_y - start.pose.position.y) / total_number_of_loop;
            for (int i = 0; i < total_number_of_loop; ++i) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = start.pose.position.x + x_increment * i;
                pose.pose.position.y = start.pose.position.y + y_increment * i;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                pose.header.stamp = node->now();
                pose.header.frame_id = global_frame_;
                path.poses.push_back(pose);
            }

            if(total_number_of_loop <= 4){
                path.poses.clear();
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = app_goal_x;
                pose.pose.position.y = app_goal_y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;
                pose.pose.orientation.w = 1.0;
                pose.header.stamp = node->now();
                pose.header.frame_id = global_frame_;
                path.poses.push_back(pose);

                //startendAdd = false;
            }
        }

        //RCLCPP_INFO(node_->get_logger(), "User-Log: %s", "####################E###################");
        return path;
    }

    rcl_interfaces::msg::SetParametersResult CubicSplinePlanner::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters){
        rcl_interfaces::msg::SetParametersResult result;
        for (auto parameter : parameters) {
            const auto & type = parameter.get_type();
            const auto & name = parameter.get_name();

            if (type == ParameterType::PARAMETER_STRING) {
                if (name == name_ + ".csv_file") {
                    csv_file_ = parameter.as_string();
                }   
            } else if (type == ParameterType::PARAMETER_DOUBLE){
                if (name == name_ + ".x_scale") {
                    x_scale_ = parameter.as_double();
                }
            }
        }

        result.successful = true;
        return result;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cubic_spline_planner::CubicSplinePlanner, nav2_core::GlobalPlanner)