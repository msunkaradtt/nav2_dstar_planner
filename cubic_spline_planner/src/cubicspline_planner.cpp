#include <cmath>
#include <string>
#include <memory>
#include <iterator> 

#include "nav2_util/node_utils.hpp"

#include "cubic_spline_planner/cubicspline_planner.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace cubic_spline_planner{
    void CubicSplinePlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name, std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        // Parameter initialization
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".csv_file", rclcpp::ParameterValue(""));
        node_->get_parameter(name_ + ".csv_file", csv_file_);
        nav2_util::declare_parameter_if_not_declared(node_, name_ + ".x_scale", rclcpp::ParameterValue(0.0));
        node_->get_parameter(name_ + ".x_scale", x_scale_);

        count = 500;

        startendAdd = false;
        diffState = false;
        diff = {};

        startPointX = 0.0;
        startPointY = 0.0;

        fs::path csvPath(fs::current_path());
        csvPath /= "src";
        csvPath /= "cubic_spline_planner";
        csvPath /= "data";
        csvPath /= csv_file_;

        RCLCPP_INFO(node_->get_logger(), "User-Log: %s", csvPath.c_str());

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
        RCLCPP_INFO(node_->get_logger(), "User-Log: %s", "triggred!");
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

        if(!startendAdd){
            startPointX = start.pose.position.x;
            startPointY = start.pose.position.y;
            startendAdd = true;
        }

        //RCLCPP_INFO(node_->get_logger(), "User-Log: %f, %f", start.pose.position.x, start.pose.position.y);
        //RCLCPP_INFO(node_->get_logger(), "[CubicSplinePlanner-Log] %f, %f", goal.pose.position.x, goal.pose.position.y);

        /*if(!startendAdd){
            xs.insert(xs.begin(), start.pose.position.x);
            xs.push_back(goal.pose.position.x);

            //tangentsIn.insert(tangentsIn.begin(), {0.0, 0.0});
            //tangentsIn.push_back({0.0, 0.0});

            //tangentsOut.insert(tangentsOut.begin(), {0.0, 0.0});
            //tangentsOut.push_back({0.0, 0.0});

            ys.insert(ys.begin(), start.pose.position.y);
            ys.push_back(goal.pose.position.y);

            startendAdd = true;
        }*/

        path.poses.clear();
        path.header.stamp = node_->now();
        path.header.frame_id = global_frame_;

        std::vector<cubic_spline_planner::InterpolatedPoint> interpolatedPoints = cubic_spline_planner::Spline::InterpolateXYWithYaw(xs_t, ys_t, tangentsIn_t, tangentsOut_t, count); //xs_prime, ys_prime //tangentsIn_t, tangentsOut_t
        
        
        if(!diffState){
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

            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            path.poses.push_back(pose);
        }
        
        return path;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cubic_spline_planner::CubicSplinePlanner, nav2_core::GlobalPlanner)