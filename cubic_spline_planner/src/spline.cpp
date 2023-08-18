#include "cubic_spline_planner/spline.hpp"

namespace cubic_spline_planner{
    Spline::Spline(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& tangents_in, const std::vector<glm::vec3>& tangents_out){
        if(points.size() != tangents_in.size() || points.size() != tangents_out.size()){
            //Errror Info!
        }

        points_ = points;
        tagents_in_ = tangents_in;
        tagents_out_ = tangents_out;

        CalculateControlPoints();
    }

    Spline::~Spline(){}

    glm::vec3 Spline::Interplolate(float t){
        int segment = int(t * (points_.size() - 1));
        float t_segment = t * (points_.size() - 1) - segment;

        glm::vec3 p0 = points_[segment];
        glm::vec3 p1 = control_points_[segment];
        glm::vec3 p2 = control_points_[segment + 1];
        glm::vec3 p3 = points_[segment + 1];

        float t2 = t_segment * t_segment;
        float t3 = t2 * t_segment;

        glm::vec3 interpolated_point = 0.5f * ((2.0f * p1) + (-p0 + p2) * t_segment + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 + (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);

        return interpolated_point;
    }

    void Spline::CalculateControlPoints(){
        control_points_.clear();

         for (size_t i = 0; i < points_.size(); ++i) {
            glm::vec3 p0 = points_[i];
            glm::vec3 p1 = p0 + (1.0f / 3.0f) * tagents_out_[i];
            glm::vec3 p2 = points_[(i + 1) % points_.size()] - (1.0f / 3.0f) * tagents_in_[i];
            glm::vec3 p3 = points_[(i + 1) % points_.size()];

            control_points_.push_back(p1);
            control_points_.push_back(p2);
         }
    }

    std::pair<std::vector<double>, std::vector<double>> Spline::InterpolateXY(const std::vector<double>& xs, const std::vector<double>& ys, int count){
        if (xs.size() != ys.size()) {
            //
        }

        int inputPointCount = xs.size();
        std::vector<double> inputDistances(inputPointCount, 0.0);
        for (int i = 1; i < inputPointCount; i++) {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            double distance = std::sqrt(dx * dx + dy * dy);
            inputDistances[i] = inputDistances[i - 1] + distance;
        }


        double meanDistance = inputDistances.back() / (count - 1);
        std::vector<double> evenDistances(count);
        for (int i = 0; i < count; i++) {
            evenDistances[i] = i * meanDistance;
        }

        std::vector<double> xsOut = Interpolate(inputDistances, xs, evenDistances);
        std::vector<double> ysOut = Interpolate(inputDistances, ys, evenDistances);

        return std::make_pair(xsOut, ysOut);
    }

    std::vector<double> Spline::Interpolate(const std::vector<double>& xOrig, const std::vector<double>& yOrig, const std::vector<double>& xInterp) {
        std::vector<double> yInterp(xInterp.size());

        for (size_t i = 0; i < xInterp.size(); i++) {
            size_t j = 0;
            while (j < xOrig.size() - 2 && xInterp[i] > xOrig[j + 1]) {
                j++;
            }

            double t = (xInterp[i] - xOrig[j]) / (xOrig[j + 1] - xOrig[j]);
            double x1 = xOrig[j];
            double y1 = yOrig[j];
            double x2 = xOrig[j + 1];
            double y2 = yOrig[j + 1];
            double cx1 = x1 + (x2 - x1) / 3.0;
            double cy1 = y1 + (y2 - y1) / 3.0;
            double cx2 = x1 + 2.0 * (x2 - x1) / 3.0;
            double cy2 = y1 + 2.0 * (y2 - y1) / 3.0;
            double bx = x1 + t * (x2 - x1);
            double by = y1 + t * (y2 - y1);
            double x = (1 - t) * ((1 - t) * x1 + t * cx1) + t * ((1 - t) * cx2 + t * bx);
            double y = (1 - t) * ((1 - t) * y1 + t * cy1) + t * ((1 - t) * cy2 + t * by);
            yInterp[i] = y;
        }

        return yInterp;
    } 
}