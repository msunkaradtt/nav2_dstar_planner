#ifndef CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_INTERPOLATOR_HPP_
#define CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_INTERPOLATOR_HPP_

#include <string>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace cubic_spline_planner{
    class CubicSplineInterpolator{
        public:
        CubicSplineInterpolator(double pointsPerUnit = 5.0, unsigned int skipPoints = 0, bool useEndConditions = true, bool useMiddleConditions = false);
        CubicSplineInterpolator(std::string name);
        ~CubicSplineInterpolator();

        void interpolatePath(nav_msgs::msg::Path & path, nav_msgs::msg::Path & smoothedPath);
        void interpolatePath(const std::vector<geometry_msgs::msg::PoseStamped> & path, std::vector<geometry_msgs::msg::PoseStamped> & smoothedPath);
        
    };
}

#endif //CUBIC_SPLINE_PLANNER__CUBIC_SPLINE_INTERPOLATOR_HPP_