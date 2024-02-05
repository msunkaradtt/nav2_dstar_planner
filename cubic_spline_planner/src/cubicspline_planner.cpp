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
        declare_parameter_if_not_declared(node, name_ + ".spline_points", rclcpp::ParameterValue(0));
        node->get_parameter(name_ + ".spline_points", spline_points_);
        declare_parameter_if_not_declared(node, name_ + ".spline_range", rclcpp::ParameterValue(0));
        node->get_parameter(name_ + ".spline_range", spline_range_);

        count = spline_points_;

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

         Point p = {posX, posY, tanInX, tanInY, tanOutX, tanOutY};
         points.push_back(p);
        }

        scaleposx(x_scale_);
    }

    Point CubicSplinePlanner::CubicHermiteSpline(const Point& p0, const Point& p1, double t) {
        double t2 = t * t;
        double t3 = t2 * t;

        double h00 = 2 * t3 - 3 * t2 + 1;
        double h10 = t3 - 2 * t2 + t;
        double h01 = -2 * t3 + 3 * t2;
        double h11 = t3 - t2;

        Point result;
        result.x = h00 * p0.x + h10 * p0.dx_out + h01 * p1.x + h11 * p1.dx_in;
        result.y = h00 * p0.y + h10 * p0.dy_out + h01 * p1.y + h11 * p1.dy_in;

        return result;
    }

    YawAngle CubicSplinePlanner::CalculateYawAngle(const Point& p0, const Point& p1) {
        YawAngle yaw;
        yaw.angle = atan2(p1.y - p0.y, p1.x - p0.x);
        return yaw;
    }

    void CubicSplinePlanner::SmoothSpline(std::vector<Point>& points, int smoothingWindow){
        std::vector<Point> smoothedPoints = points;
        int halfWindow = smoothingWindow / 2;

        for (int i = halfWindow; i < points.size() - halfWindow; ++i) {
            double sumX = 0.0, sumY = 0.0;
            for (int j = -halfWindow; j <= halfWindow; ++j) {
                sumX += points[i + j].x;
                sumY += points[i + j].y;
            }
            smoothedPoints[i].x = sumX / smoothingWindow;
            smoothedPoints[i].y = sumY / smoothingWindow;
        }

        for (int i = halfWindow; i < points.size() - halfWindow; ++i) {
            points[i] = smoothedPoints[i];
        }
    }

    std::vector<Point> CubicSplinePlanner::GenerateCubicHermiteSpline(const std::vector<Point>& points, 
    int numSegments, std::vector<YawAngle>& yawAngles){
        std::vector<Point> splinePoints;

        for (size_t i = 0; i < points.size() - 1; i++) {
            for (int j = 0; j < numSegments; j++) {
                double t = static_cast<double>(j) / static_cast<double>(numSegments - 1);
                Point p = CubicHermiteSpline(points[i], points[i + 1], t);
                splinePoints.push_back(p);
            }
        }

        SmoothSpline(splinePoints, spline_range_);

        // Calculate yaw angles for the generated spline points
        for (size_t i = 0; i < splinePoints.size() - 1; i++) {
            yawAngles.push_back(CalculateYawAngle(splinePoints[i], splinePoints[i + 1]));
        }

        if(splinePoints.size() > 1){
            yawAngles.push_back(CalculateYawAngle(splinePoints[splinePoints.size() - 2], splinePoints.back()));
        }

        return splinePoints;
    }

    void CubicSplinePlanner::scaleposx(double scaleVal){
        for(int i = 1; i < xs_t.size(); i++){
            points[i].x -= scaleVal; 
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

        std::vector<Point> splinePoints = GenerateCubicHermiteSpline(points, count, yawAngles);

        //std::vector<cubic_spline_planner::InterpolatedPoint> interpolatedPoints = cubic_spline_planner::Spline::InterpolateXYWithYawC2(xs_t, ys_t, tangentsIn_t, tangentsOut_t, count); //tangentsIn_t, tangentsOut_t
        
        
        if(!diffState){
            //RCLCPP_INFO(node_->get_logger(), "User-Log: %s", "Here!");
            ticker = 0;
            startPointX = start.pose.position.x;
            startPointY = start.pose.position.y;
            diff.clear();
            diff.push_back(0.0);
            double startx = splinePoints[0].x;
            for(int i = 1; i < splinePoints.size(); i++){
                double d = splinePoints[i].x - startx;
                diff.push_back(d);
            }

            diffState = true;
        }

        for (int j = 0; j < splinePoints.size(); j++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = startPointX - diff[j];
            pose.pose.position.y = splinePoints[j].y + startPointY;
            pose.pose.position.z = 0.0;

            tf2::Quaternion _quater;
            _quater.setRPY(0, 0, yawAngles[j].angle);
            _quater = _quater.normalize();

            pose.pose.orientation.x = _quater.getX();
            pose.pose.orientation.y = _quater.getY();
            pose.pose.orientation.z = _quater.getZ();
            pose.pose.orientation.w = _quater.getW();

            pose.header.stamp = node->now();
            pose.header.frame_id = global_frame_;
            path.poses.push_back(pose);
        }

        //RCLCPP_INFO(logger_, "User-Log: %s", "####################E###################");
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
            } else if (type == ParameterType::PARAMETER_INTEGER){
                if (name == name_ + ".spline_points") {
                    spline_points_ = parameter.as_int();
                } else if (name == name_ + ".spline_range"){
                    spline_range_ = parameter.as_int();
                }
            }
        }

        result.successful = true;
        return result;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cubic_spline_planner::CubicSplinePlanner, nav2_core::GlobalPlanner)