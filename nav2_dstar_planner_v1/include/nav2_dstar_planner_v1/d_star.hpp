#ifndef NAV2_DSTAR_PLANNER_V1__D_STAR_HPP_
#define NAV2_DSTAR_PLANNER_V1__D_STAR_HPP_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "nav2_dstar_planner_v1/nodes.hpp"

namespace nav2_dstar_planner_v1{
    #define COST_UNKNOWN_ROS 255
    #define COST_OBS 254
    #define COST_OBS_ROS 253

    #define COST_NEUTRAL 50
    #define COST_FACTOR 0.8

    #ifndef COSTTYPE
    #define COSTTYPE unsigned char
    #endif

    #define WINDOW_SIZE 70

    typedef DNode* DNodePtr;

    class DStar {
        public:
        DStar(int xs, int ys);
        ~DStar();

        void setNavArr(int xs, int ys);
        int nx, ny, ns;

        void insert(DNodePtr node_ptr, double h_new);
        double processState();
        void getNeighbours(DNodePtr node_ptr, std::vector<DNodePtr>& neighbours);
        double getCost(DNodePtr n1, DNodePtr n2);
        bool isCollision(DNodePtr n1, DNodePtr n2);
        void extractPath(const Node& start, const Node& goal);
        Node getState(const Node& current);
        void modify(DNodePtr x);
        bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);


        public:
        unsigned char* curr_global_costmap_;
        unsigned char* last_global_costmap_; 
        Node goal_;
        std::multimap<double, DNodePtr> open_list_;
        std::vector<Node> path_;
        std::vector<Node> expand_;

        void initMap();

        public:
        DNodePtr** map_;

        unsigned char lethal_cost_;
        double factor_;

        int grid2Index(int x, int y);
        void index2Grid(int i, int& x, int& y);
        void map2Grid(double mx, double my, int& gx, int& gy);
        void reset();
    };
}

#endif //NAV2_DSTAR_PLANNER_V1__D_STAR_HPP_