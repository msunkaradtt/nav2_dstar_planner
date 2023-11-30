#ifndef CUBIC_SPLINE_PLANNER__SPLINE_HPP_
#define CUBIC_SPLINE_PLANNER__SPLINE_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace cubic_spline_planner{
    struct InterpolatedPoint
    {
        double x;
        double y;
        double yaw;
    };
    
    class Spline{
        public:
        Spline();
        ~Spline();
        static std::pair<std::vector<double>, std::vector<double>> InterpolateXY(const std::vector<double>& xs, const std::vector<double>& ys, int count);

        static std::vector<InterpolatedPoint> InterpolateXYWithYaw(const std::vector<double>& xs, const std::vector<double>& ys, int count);

        static std::vector<InterpolatedPoint> InterpolateXYWithYaw(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, int count);

        static std::pair<std::vector<double>, std::vector<double>> InterpolateXY(const std::vector<double>& xs, const std::vector<double>& ys, 
        const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, int count);

        static std::vector<InterpolatedPoint> InterpolateXYWithYawC2(const std::vector<double>& xs, const std::vector<double>& ys, int count);
        static std::vector<InterpolatedPoint> InterpolateXYWithYawC2(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, int count);

        private:
        static std::vector<double> Interpolate(const std::vector<double>& xOrig, const std::vector<double>& yOrig, const std::vector<double>& xInterp);

        static std::vector<InterpolatedPoint> InterpolateYaw(const std::vector<double>& xOrig, const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& xInterp);
        static std::vector<InterpolatedPoint> InterpolateYaw(const std::vector<double>& xOrig, const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, const std::vector<double>& xInterp);

        static std::vector<InterpolatedPoint> InterpolateYawC2(const std::vector<double>& xOrig, const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& xInterp);
        static std::vector<InterpolatedPoint> InterpolateYawC2(const std::vector<double>& xOrig, const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, const std::vector<double>& xInterp);
        
        static std::vector<double> Interpolate(const std::vector<double>& xOrig, const std::vector<double>& yOrig, 
        const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, const std::vector<double>& xInterp);
    };
}

#endif //CUBIC_SPLINE_PLANNER__SPLINE_HPP_