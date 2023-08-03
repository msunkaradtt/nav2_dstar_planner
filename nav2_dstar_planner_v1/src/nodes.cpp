#include "nav2_dstar_planner_v1/nodes.hpp"

namespace nav2_dstar_planner_v1{
    Node::Node(int x, int y, double g, double h, int id, int pid) : x_(x), y_(y), g_(g), h_(h), id_(id), pid_(pid){

    }

    Node Node::operator+(const Node& n) const{
        Node result;
        result.x_ = x_ + n.x_;
        result.y_ = y_ + n.y_;
        result.g_ = g_ + n.g_;

        return result;
    }

    Node Node::operator-(const Node& n) const{
        Node result;
        result.x_ = x_ - n.x_;
        result.y_ = y_ - n.y_;
        
        return result;
    }

    bool Node::operator==(const Node& n) const{
        return x_ == n.x_ && y_ == n.y_;
    }

    bool Node::operator!=(const Node& n) const{
        return !operator==(n);
    }

    size_t NodeIdAsHash::operator()(const Node& n) const{
        return n.id_;
    }

    bool compare_cost::operator()(const Node& n1, const Node& n2) const{
        return (n1.g_ + n1.h_ > n2.g_ + n2.h_) || ((n1.g_ + n1.h_ == n2.g_ + n2.h_) && (n1.h_ > n2.h_));
    }

    bool compare_coordinates::operator()(const Node& n1, const Node& n2) const{
        return (n1.x_ == n2.x_) && (n1.y_ == n2.y_);
    }
}