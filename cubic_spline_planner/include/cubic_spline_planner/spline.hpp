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
    class Spline{
        public:
        Spline(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& tangents_in, const std::vector<glm::vec3>& tangents_out);
        ~Spline();

        glm::vec3 Interplolate(float t);

        static std::pair<std::vector<double>, std::vector<double>> InterpolateXY(const std::vector<double>& xs, const std::vector<double>& ys, int count);


        private:
        void CalculateControlPoints();
        static std::vector<double> Interpolate(const std::vector<double>& xOrig, const std::vector<double>& yOrig, const std::vector<double>& xInterp); 

        std::vector<glm::vec3> points_;
        std::vector<glm::vec3> tagents_in_;
        std::vector<glm::vec3> tagents_out_;
        std::vector<glm::vec3> control_points_;
    };
}

#endif //CUBIC_SPLINE_PLANNER__SPLINE_HPP_