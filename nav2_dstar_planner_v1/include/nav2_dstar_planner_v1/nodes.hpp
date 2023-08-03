#ifndef NAV2_DSTAR_PLANNER_V1__NODES_HPP_
#define NAV2_DSTAR_PLANNER_V1__NODES_HPP_

#define INF 10000

#include <cmath>
#include <array>
#include <vector>
#include <map>

namespace nav2_dstar_planner_v1{
    class Node{
        public:
        Node(int x = 0, int y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0);
        Node operator+(const Node& n) const;
        Node operator-(const Node& n) const;
        bool operator==(const Node& n) const;
        bool operator!=(const Node& n) const;

        public:
        int x_, y_;
        double g_, h_;
        int id_, pid_;
    };

    class NodeIdAsHash{
        public:
        size_t operator()(const Node& n) const;
    };

    struct compare_cost{
        bool operator()(const Node& n1, const Node& n2) const;
    };

    struct compare_coordinates{
        bool operator()(const Node& n1, const Node& n2) const;
    };
    
    struct pair_hash
    {
        template <class T1, class T2>
        std::size_t operator()(const std::pair<T1, T2>& pair) const{
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    class DNode : public Node{
        public:
        enum Tag{
            NEW = 0,
            OPEN = 1,
            CLOSED = 2
        };

        DNode(const int x = 0, const int y = 0, const double cost = INF, const double h_cost = INF, const int id = 0,
        const int pid = -1, const int t = NEW, const double k = INF)
        : Node(x, y, cost, h_cost, id, pid), t_(t), k_(k){}

        public:
        int t_;
        double k_;
    };
    
}

#endif //NAV2_DSTAR_PLANNER_V1__NODES_HPP_