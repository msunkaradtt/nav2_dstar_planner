#include "cubic_spline_planner/cubic_spline_interpolator.hpp"
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/utils.h"


namespace cubic_spline_planner{
    CubicSplineInterpolator::CubicSplineInterpolator(double pointsPerUnit, unsigned int skipPoints, bool useEndConditions, 
    bool useMiddleConditions) : pointsPerUnit_(pointsPerUnit), skipPoints_(skipPoints), useEndConditions_(useEndConditions), 
    useMiddleConditions_(useMiddleConditions){
    }

    CubicSplineInterpolator::CubicSplineInterpolator(std::string name){
        pointsPerUnit_ = 5.0;
        useEndConditions_ = false;
        useMiddleConditions_ = false;
        skipPoints_ = 0;
    }

    CubicSplineInterpolator::~CubicSplineInterpolator(){
    }

    void CubicSplineInterpolator::interpolatePath(nav_msgs::msg::Path & path, nav_msgs::msg::Path & smoothedPath){
        smoothedPath.header = path.header;
        interpolatePath(path.poses, smoothedPath.poses);
    }

    auto CubicSplineInterpolator::createQuaternionMsgFromYaw(double yaw){
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }

    void CubicSplineInterpolator::interpolatePath(const std::vector<geometry_msgs::msg::PoseStamped> & path, 
    std::vector<geometry_msgs::msg::PoseStamped> & smoothedPath){
        smoothedPath.clear();

        unsigned int oldSkipPoints = skipPoints_;
        skipPoints_ = std::min<int>(path.size() - 2, skipPoints_);

        std::vector<double> cummulativeDistances;
        calcCummulativeDistances(path, cummulativeDistances);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path[0].header;

        unsigned int numPoints = pointsPerUnit_ * calcTotalDistance(path);
        smoothedPath.resize(numPoints);

        for (unsigned int i = 0; i < numPoints; i++){
            double u = static_cast<double>(i) / (numPoints-1);
            interpolatePoint(path, cummulativeDistances, pose, u);

            if (std::isnan(pose.pose.position.x) || std::isnan(pose.pose.position.y)){
                pose.pose = smoothedPath[std::max(static_cast<int>(i)-1, 0)].pose;
            }

            smoothedPath[i] = pose;
        }

        smoothedPath.front().pose.orientation = path.front().pose.orientation;
        smoothedPath.back().pose.orientation = path.back().pose.orientation;

        for (unsigned int i = 1; i < smoothedPath.size()-1; i++){
            double dx = smoothedPath[i+1].pose.position.x - smoothedPath[i].pose.position.x;
            double dy = smoothedPath[i+1].pose.position.y - smoothedPath[i].pose.position.y;
            double th = atan2(dy, dx);
            smoothedPath[i].pose.orientation = createQuaternionMsgFromYaw(th);
        }

        skipPoints_ = oldSkipPoints;
    }

    void CubicSplineInterpolator::interpolatePoint(const std::vector<geometry_msgs::msg::PoseStamped>& path, 
    const std::vector<double>& cummulativeDistances, geometry_msgs::msg::PoseStamped& point, double pointCummDist){
        unsigned int group = findGroup(cummulativeDistances, pointCummDist);

        double a = calcAlphaCoeff(path, cummulativeDistances, group, pointCummDist);
        double b = calcBetaCoeff(path, cummulativeDistances, group, pointCummDist);
        double c = calcGammaCoeff(path, cummulativeDistances, group, pointCummDist);
        double d = calcDeltaCoeff(path, cummulativeDistances, group, pointCummDist);

        std::vector<double> grad, nextGrad;
        calcPointGradient(path, cummulativeDistances, group, grad);
        calcPointGradient(path, cummulativeDistances, group+1, nextGrad);

        point.pose.position.x = + a * path[group*(skipPoints_+1)].pose.position.x + b * path[(group+1)*(skipPoints_+1)].pose.position.x + c * grad[0] + d * nextGrad[0];
        point.pose.position.y = + a * path[group*(skipPoints_+1)].pose.position.y + b * path[(group+1)*(skipPoints_+1)].pose.position.y + c * grad[1] + d * nextGrad[1];
    }

    void CubicSplineInterpolator::calcCummulativeDistances(const std::vector<geometry_msgs::msg::PoseStamped> path, std::vector<double>& cummulativeDistances){
        cummulativeDistances.clear();
        cummulativeDistances.push_back(0);

        for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1){
            cummulativeDistances.push_back(cummulativeDistances.back() + calcDistance(path, i) / calcTotalDistance(path));
        }
    }

    double CubicSplineInterpolator::calcTotalDistance(const std::vector<geometry_msgs::msg::PoseStamped>& path){
        double totalDist = 0;
        for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1){
            totalDist += calcDistance(path, i);
        }

        return totalDist;
    }

    double CubicSplineInterpolator::calcDistance(const std::vector<geometry_msgs::msg::PoseStamped>& path, unsigned int idx){
        if (idx <= 0 || idx >=path.size()){
            return 0;
        }

        double dist = hypot(path[idx].pose.position.x - path[idx-skipPoints_-1].pose.position.x, path[idx].pose.position.y - path[idx-skipPoints_-1].pose.position.y);
        return dist;
    }

    double CubicSplineInterpolator::calcAlphaCoeff(const std::vector<geometry_msgs::msg::PoseStamped> path, const std::vector<double> cummulativeDistances, unsigned int idx, double input){
        double alpha = + 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3) - 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2) + 1;
        return alpha;
    }

    double CubicSplineInterpolator::calcBetaCoeff(const std::vector<geometry_msgs::msg::PoseStamped> path, const std::vector<double> cummulativeDistances, unsigned int idx, double input){
        double beta = - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3) + 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2);
        return beta;
    }

    double CubicSplineInterpolator::calcGammaCoeff(const std::vector<geometry_msgs::msg::PoseStamped> path, const std::vector<double> cummulativeDistances, unsigned int idx, double input){
        double gamma = (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3) - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2)) * (cummulativeDistances[idx+1] - cummulativeDistances[idx]) + input - cummulativeDistances[idx];
        return gamma; 
    }

    double CubicSplineInterpolator::calcDeltaCoeff(const std::vector<geometry_msgs::msg::PoseStamped> path, const std::vector<double> cummulativeDistances, unsigned int idx, double input){
        double delta = (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3) - pow(calcRelativeDistance(cummulativeDistances, idx, input), 2)) * (cummulativeDistances[idx+1] - cummulativeDistances[idx]);
        return delta;
    }

    double CubicSplineInterpolator::calcRelativeDistance(const std::vector<double>& cummulativeDistances, unsigned int idx, double input){
        double relDist = (input - cummulativeDistances[idx]) / (cummulativeDistances[idx+1] - cummulativeDistances[idx]);
        return relDist;
    }

    void CubicSplineInterpolator::calcPointGradient(const std::vector<geometry_msgs::msg::PoseStamped>& path, const std::vector<double>& cummulativeDistances, 
    unsigned int idx, std::vector<double>& gradient){
        double dx, dy, du;
        gradient.assign(2, 0);

        if ((useEndConditions_ && (idx == 0 || idx == cummulativeDistances.size()-1)) || useMiddleConditions_){
            double th = tf2::getYaw(path[idx*(skipPoints_+1)].pose.orientation);
            int sign = (fabs(th) < M_PI / 2) ? 1 : -1;

            gradient[0] = sign * calcTotalDistance(path) * sqrt(1 + pow(tan(th),2)) / (1 + pow(tan(th), 2));
            gradient[1] = tan(th) * gradient[0];
        } else {
            if (idx == 0 || idx == cummulativeDistances.size()-1){
                return;
            }

            dx = path[(idx)*(skipPoints_+1)].pose.position.x - path[(idx-1)*(skipPoints_+1)].pose.position.x;
            dy = path[(idx)*(skipPoints_+1)].pose.position.y - path[(idx-1)*(skipPoints_+1)].pose.position.y;
            du = cummulativeDistances[idx] - cummulativeDistances[idx-1];

            gradient[0] =  dx / du;
            gradient[1] =  dy / du;
        }
    }

    unsigned int CubicSplineInterpolator::findGroup(const std::vector<double>& cummulativeDistances, double pointCummDist){
        unsigned int i;
        for (i = 0; i < cummulativeDistances.size()-1; i++){
            if (pointCummDist <= cummulativeDistances[i+1]){
                return i;
            }
        }
        return i;
    }
}