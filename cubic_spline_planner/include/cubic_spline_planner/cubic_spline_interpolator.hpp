#ifndef CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_INTERPOLATOR_HPP_
#define CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_INTERPOLATOR_HPP_

#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace cubic_spline_planner{
    class CubicSplineInterpolator{
        public:
        CubicSplineInterpolator(double pointsPerUnit = 5.0, unsigned int skipPoints = 0, bool useEndConditions = true, bool useMiddleConditions = false);
        CubicSplineInterpolator(std::string name);
        ~CubicSplineInterpolator();

        void interpolatePath(nav_msgs::msg::Path & path, nav_msgs::msg::Path & smoothedPath);
        void interpolatePath(const std::vector<geometry_msgs::msg::PoseStamped> & path, std::vector<geometry_msgs::msg::PoseStamped> & smoothedPath);

        void interpolatePoint(const std::vector<geometry_msgs::msg::PoseStamped>& path, const std::vector<double>& cummulativeDistances, geometry_msgs::msg::PoseStamped& point, double pointCummDist);
        void calcCummulativeDistances(const std::vector<geometry_msgs::msg::PoseStamped> path, std::vector<double>& cummulativeDistances);

        double calcTotalDistance(const std::vector<geometry_msgs::msg::PoseStamped>& path);
        double calcDistance(const std::vector<geometry_msgs::msg::PoseStamped>& path, unsigned int idx);

        double calcAlphaCoeff(const std::vector<geometry_msgs::msg::PoseStamped> path, const std::vector<double> cummulativeDistances, unsigned int idx, double input);
        double calcBetaCoeff(const std::vector<geometry_msgs::msg::PoseStamped> path, const std::vector<double> cummulativeDistances, unsigned int idx, double input);
        double calcGammaCoeff(const std::vector<geometry_msgs::msg::PoseStamped> path, const std::vector<double> cummulativeDistances, unsigned int idx, double input);
        double calcDeltaCoeff(const std::vector<geometry_msgs::msg::PoseStamped> path, const std::vector<double> cummulativeDistances, unsigned int idx, double input);

        double calcRelativeDistance(const std::vector<double>& cummulativeDistances, unsigned int idx, double input);
        void calcPointGradient(const std::vector<geometry_msgs::msg::PoseStamped>& path, const std::vector<double>& cummulativeDistances, unsigned int idx, std::vector<double>& gradient);

        unsigned int findGroup(const std::vector<double>& cummulativeDistances, double pointCummDist);

        double getPointsPerUnit() {return pointsPerUnit_;}
        unsigned int skipPoints() {return skipPoints_;}
        bool getUseEndConditions() {return useEndConditions_;}
        bool getUseMiddleConditions() {return useMiddleConditions_;}

        void setPointsPerUnit(double ppu) {pointsPerUnit_ = ppu;}
        void setSkipPoints(unsigned int sp) {skipPoints_ = sp;}
        void setUseEndConditions(bool uec) {useEndConditions_ = uec;}
        void setUseMiddleConditions(bool umc) {useMiddleConditions_ = umc;}

        public:
        auto createQuaternionMsgFromYaw(double yaw);

        private:
        double pointsPerUnit_;
        unsigned int skipPoints_;
        bool useEndConditions_;
        bool useMiddleConditions_;
    };
}

#endif //CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_INTERPOLATOR_HPP_