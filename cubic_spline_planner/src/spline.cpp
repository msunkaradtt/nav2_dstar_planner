#include "cubic_spline_planner/spline.hpp"

namespace cubic_spline_planner{
    Spline::Spline(){}

    Spline::~Spline(){}

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

    std::pair<std::vector<double>, std::vector<double>> Spline::InterpolateXY(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, int count){
        if (xs.size() != ys.size() || xs.size() != tangentsIn.size() || xs.size() != tangentsOut.size()) {
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

        std::vector<double> xsOut = Interpolate(inputDistances, xs, tangentsIn, tangentsOut, evenDistances);
        std::vector<double> ysOut = Interpolate(inputDistances, ys, tangentsIn, tangentsOut, evenDistances);

        return std::make_pair(xsOut, ysOut);
    } 

    std::vector<double> Spline::Interpolate(const std::vector<double>& xOrig, const std::vector<double>& yOrig, const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, const std::vector<double>& xInterp){
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
            double cx1 = x1 + tangentsOut[j].first;
            double cy1 = y1 + tangentsOut[j].second;
            double cx2 = x2 - tangentsIn[j + 1].first;
            double cy2 = y2 - tangentsIn[j + 1].second;
            double bx = x1 + t * (x2 - x1);
            double by = y1 + t * (y2 - y1);
            double x = (1 - t) * ((1 - t) * x1 + t * cx1) + t * ((1 - t) * cx2 + t * bx);
            double y = (1 - t) * ((1 - t) * y1 + t * cy1) + t * ((1 - t) * cy2 + t * by);
            yInterp[i] = y;
        }

        return yInterp;
    }

    std::vector<InterpolatedPoint> Spline::InterpolateXYWithYaw(const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    int count){
        if (xs.size() != ys.size()) {

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

        return InterpolateYaw(inputDistances, xs, ys, evenDistances);
    }

    std::vector<InterpolatedPoint> Spline::InterpolateYaw(const std::vector<double>& xOrig, 
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    const std::vector<double>& xInterp) {
        std::vector<InterpolatedPoint> interpolatedPoints(xInterp.size());
        for (size_t i = 0; i < xInterp.size(); i++) {
            size_t j = 0;
            while (j < xOrig.size() - 2 && xInterp[i] > xOrig[j + 1]) {
                j++;
            }

            double t = (xInterp[i] - xOrig[j]) / (xOrig[j + 1] - xOrig[j]);
            double x1 = xs[j];
            double y1 = ys[j];
            double x2 = xs[j + 1];
            double y2 = ys[j + 1];
            double cx1 = x1 + (x2 - x1) / 3.0;
            double cy1 = y1 + (y2 - y1) / 3.0;
            double cx2 = x1 + 2.0 * (x2 - x1) / 3.0;
            double cy2 = y1 + 2.0 * (y2 - y1) / 3.0;
            double bx = x1 + t * (x2 - x1);
            double by = y1 + t * (y2 - y1);
            double x = (1 - t) * ((1 - t) * x1 + t * cx1) + t * ((1 - t) * cx2 + t * bx);
            double y = (1 - t) * ((1 - t) * y1 + t * cy1) + t * ((1 - t) * cy2 + t * by);
            double tangentX1 = x2 - x1;
            double tangentY1 = y2 - y1;
            double tangentX2 = cx2 - cx1;
            double tangentY2 = cy2 - cy1;
            double dotProduct = tangentX1 * tangentX2 + tangentY1 * tangentY2;
            double crossProduct = tangentX1 * tangentY2 - tangentY1 * tangentX2;
            double yawAngle = std::atan2(crossProduct, dotProduct);

            interpolatedPoints[i] = {x, y, yawAngle};
        }

        return interpolatedPoints;
    }  

    std::vector<InterpolatedPoint> Spline::InterpolateXYWithYaw(const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    const std::vector<std::pair<double, double>>& tangentsIn, 
    const std::vector<std::pair<double, double>>& tangentsOut, 
    int count){
        if (xs.size() != ys.size() || xs.size() != tangentsIn.size() || xs.size() != tangentsOut.size()) {

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

        return InterpolateYaw(inputDistances, xs, ys, tangentsIn, tangentsOut, evenDistances);
    }

    std::vector<InterpolatedPoint> Spline::InterpolateYaw(const std::vector<double>& xOrig, 
    const std::vector<double>& xs, 
    const std::vector<double>& ys, 
    const std::vector<std::pair<double, double>>& tangentsIn, 
    const std::vector<std::pair<double, double>>& tangentsOut, 
    const std::vector<double>& xInterp){
        std::vector<InterpolatedPoint> interpolatedPoints(xInterp.size());
        for (size_t i = 0; i < xInterp.size(); i++) {
            size_t j = 0;
            while (j < xOrig.size() - 2 && xInterp[i] > xOrig[j + 1]) {
                j++;
            }

            double t = (xInterp[i] - xOrig[j]) / (xOrig[j + 1] - xOrig[j]);
            double x1 = xs[j];
            double y1 = ys[j];
            double x2 = xs[j + 1];
            double y2 = ys[j + 1];
            double cx1 = x1 + tangentsOut[j].first;
            double cy1 = y1 + tangentsOut[j].second;
            double cx2 = x2 - tangentsIn[j + 1].first;
            double cy2 = y2 - tangentsIn[j + 1].second;
            double bx = x1 + t * (x2 - x1);
            double by = y1 + t * (y2 - y1);
            double x = (1 - t) * ((1 - t) * x1 + t * cx1) + t * ((1 - t) * cx2 + t * bx);
            double y = (1 - t) * ((1 - t) * y1 + t * cy1) + t * ((1 - t) * cy2 + t * by);

            double tangentX1 = x2 - x1;
            double tangentY1 = y2 - y1;
            double tangentX2 = cx2 - cx1;
            double tangentY2 = cy2 - cy1;
            double dotProduct = tangentX1 * tangentX2 + tangentY1 * tangentY2;
            double crossProduct = tangentX1 * tangentY2 - tangentY1 * tangentX2;
            double yawAngle = std::atan2(crossProduct, dotProduct);

            interpolatedPoints[i] = {x, y, yawAngle};
        }

        return interpolatedPoints;
    }

    std::vector<InterpolatedPoint> Spline::InterpolateXYWithYawC2(const std::vector<double>& xs, const std::vector<double>& ys, int count){
        if (xs.size() != ys.size()) {

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

        return InterpolateYawC2(inputDistances, xs, ys, evenDistances);
    }

    std::vector<InterpolatedPoint> Spline::InterpolateYawC2(const std::vector<double>& xOrig, const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<double>& xInterp){
        std::vector<InterpolatedPoint> interpolatedPoints(xInterp.size());

        for (size_t i = 0; i < xInterp.size(); i++) {
            size_t j = 0;
            while (j < xOrig.size() - 2 && xInterp[i] > xOrig[j + 1]) {
                j++;
            }

            double t = (xInterp[i] - xOrig[j]) / (xOrig[j + 1] - xOrig[j]);
            double x1 = xs[j];
            double y1 = ys[j];
            double x2 = xs[j + 1];
            double y2 = ys[j + 1];
            double cx1 = x1 + (x2 - x1) / 3.0;
            double cy1 = y1 + (y2 - y1) / 3.0;
            double cx2 = x1 + 2.0 * (x2 - x1) / 3.0;
            double cy2 = y1 + 2.0 * (y2 - y1) / 3.0;
            double bx = x1 + t * (x2 - x1);
            double by = y1 + t * (y2 - y1);
            double x = (1 - t) * ((1 - t) * x1 + t * cx1) + t * ((1 - t) * cx2 + t * bx);
            double y = (1 - t) * ((1 - t) * y1 + t * cy1) + t * ((1 - t) * cy2 + t * by);

            double tangentX1 = x2 - x1;
            double tangentY1 = y2 - y1;
            double tangentX2 = cx2 - cx1;
            double tangentY2 = cy2 - cy1;
            double dotProduct = tangentX1 * tangentX2 + tangentY1 * tangentY2;
            double crossProduct = tangentX1 * tangentY2 - tangentY1 * tangentX2;
            double yawAngle = std::atan2(crossProduct, dotProduct);

            interpolatedPoints[i] = {x, y, yawAngle};
        }

        return interpolatedPoints;
    }

    std::vector<InterpolatedPoint> Spline::InterpolateXYWithYawC2(const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, int count){
        if (xs.size() != ys.size() || xs.size() != tangentsIn.size() || xs.size() != tangentsOut.size()) {

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

         return InterpolateYawC2(inputDistances, xs, ys, tangentsIn, tangentsOut, evenDistances);
    }

    std::vector<InterpolatedPoint> Spline::InterpolateYawC2(const std::vector<double>& xOrig, const std::vector<double>& xs, const std::vector<double>& ys, const std::vector<std::pair<double, double>>& tangentsIn, const std::vector<std::pair<double, double>>& tangentsOut, const std::vector<double>& xInterp){
        std::vector<InterpolatedPoint> interpolatedPoints(xInterp.size());

        for (size_t i = 0; i < xInterp.size(); i++) {
            size_t j = 0;

            while (j < xOrig.size() - 2 && xInterp[i] > xOrig[j + 1]) {
                j++;
            }

            double t = (xInterp[i] - xOrig[j]) / (xOrig[j + 1] - xOrig[j]);
            double x1 = xs[j];
            double y1 = ys[j];
            double x2 = xs[j + 1];
            double y2 = ys[j + 1];
            double cx1 = x1 + tangentsOut[j].first / 3.0;
            double cy1 = y1 + tangentsOut[j].second / 3.0;
            double cx2 = x2 - tangentsIn[j + 1].first / 3.0;
            double cy2 = y2 - tangentsIn[j + 1].second / 3.0;
            double bx = x1 + t * (x2 - x1);
            double by = y1 + t * (y2 - y1);
            double x = (1 - t) * ((1 - t) * x1 + t * cx1) + t * ((1 - t) * cx2 + t * bx);
            double y = (1 - t) * ((1 - t) * y1 + t * cy1) + t * ((1 - t) * cy2 + t * by);

            double tangentX1 = x2 - x1;
            double tangentY1 = y2 - y1;
            double tangentX2 = cx2 - cx1;
            double tangentY2 = cy2 - cy1;
            double dotProduct = tangentX1 * tangentX2 + tangentY1 * tangentY2;
            double crossProduct = tangentX1 * tangentY2 - tangentY1 * tangentX2;
            double yawAngle = std::atan2(crossProduct, dotProduct);

            interpolatedPoints[i] = {x, y, yawAngle};
        }

        return interpolatedPoints;
    }
}