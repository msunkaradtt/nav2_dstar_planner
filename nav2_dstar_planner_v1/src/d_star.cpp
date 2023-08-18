#include "nav2_dstar_planner_v1/d_star.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_dstar_planner_v1{

    DStar::DStar(int xs, int ys){
        setNavArr(xs, ys);
        factor_ = OBSTACLE_FACTOR;
        lethal_cost_ = COST_OBS;
    }

    DStar::~DStar(){
        reset();
    }

    void DStar::setNavArr(int xs, int ys){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[DStar] Array is %d x %d\n", xs, ys);
        nx = xs;
        ny = ys;
        ns = nx * ny;

        curr_global_costmap_ = new unsigned char[ns];
        last_global_costmap_ = new unsigned char[ns];

        goal_.x_ = goal_.y_ = INF;

        initMap();
    }

    void DStar::initMap(){
        map_ = new DNodePtr*[nx];
        for(int i = 0; i < nx; i++){
            map_[i] = new DNodePtr[ny];
            for(int j = 0; j < ny; j++){
                map_[i][j] = new DNode(i, j, INF, INF, grid2Index(i, j), -1, DNode::NEW, INF);
            }
        }
    }

    void DStar::reset(){
        open_list_.clear();
        
        for (int i = 0; i < nx; i++){
            for (int j = 0; j < ny; j++){
                delete map_[i][j];
            }
        }

        for (int i = 0; i < nx; i++){
            delete[] map_[i];
        }

        delete[] map_;

        initMap();
    }

    Node DStar::getState(const Node& current){
        Node state(path_[0].x_, path_[0].y_);

        double dis_min = std::hypot(state.x_ - current.x_, state.y_ - current.y_);
        int idx_min = 0;
        for (int i = 1; i < path_.size(); i++){
            double dis = std::hypot(path_[i].x_ - current.x_, path_[i].y_ - current.y_);
            if (dis < dis_min){
                dis_min = dis;
                idx_min = i;
            }
        }
        state.x_ = path_[idx_min].x_;
        state.y_ = path_[idx_min].y_;
        
        return state;
    }

    void DStar::modify(DNodePtr x){
        if (x->t_ == DNode::CLOSED){
            insert(x, x->g_);
        }
    }

    void DStar::insert(DNodePtr node_ptr, double h_new){
        if (node_ptr->t_ == DNode::NEW){
            node_ptr->k_ = h_new;
        } else if (node_ptr->t_ == DNode::OPEN){
            node_ptr->k_ = std::min(node_ptr->k_, h_new);
        } else if (node_ptr->t_ == DNode::CLOSED){
            node_ptr->k_ = std::min(node_ptr->g_, h_new);
        }
        
        node_ptr->g_ = h_new;
        node_ptr->t_ = DNode::OPEN;
        open_list_.insert(std::make_pair(node_ptr->k_, node_ptr));
    }

    bool DStar::isCollision(DNodePtr n1, DNodePtr n2){
        
        return curr_global_costmap_[n1->id_] > lethal_cost_ * factor_ || curr_global_costmap_[n2->id_] > lethal_cost_ * factor_;
    }

    void DStar::getNeighbours(DNodePtr node_ptr, std::vector<DNodePtr>& neighbours){
        int x = node_ptr->x_, y = node_ptr->y_;
        for (int i = -1; i <= 1; i++){
            for (int j = -1; j <= 1; j++){
                if (i == 0 && j == 0){
                    continue;   
                }
                
                int x_n = x + i, y_n = y + j;
                if (x_n < 0 || x_n > nx - 1 || y_n < 0 || y_n > ny - 1){
                    continue;
                }
                
                DNodePtr neigbour_ptr = map_[x_n][y_n];
                neighbours.push_back(neigbour_ptr);
            }
        }
    }

    double DStar::getCost(DNodePtr n1, DNodePtr n2){
        if (isCollision(n1, n2)){
            return INF;
        }

        return std::hypot(n1->x_ - n2->x_, n1->y_ - n2->y_);
    }

    void DStar::extractPath(const Node& start, const Node& goal){
        DNodePtr node_ptr = map_[start.x_][start.y_];
        while (node_ptr->x_ != goal.x_ || node_ptr->y_ != goal.y_){
            path_.push_back(*node_ptr);
            
            int x, y;
            index2Grid(node_ptr->pid_, x, y);
            node_ptr = map_[x][y];
        }
        std::reverse(path_.begin(), path_.end());
    }

    double DStar::processState(){
        if (open_list_.empty()){
            return -1;
        }

        double k_old = open_list_.begin()->first;
        DNodePtr x = open_list_.begin()->second;
        open_list_.erase(open_list_.begin());
        x->t_ = DNode::CLOSED;
        expand_.push_back(*x);

        std::vector<DNodePtr> neigbours;
        getNeighbours(x, neigbours);
        
        if (k_old < x->g_){
            for (DNodePtr y : neigbours){
                if (y->t_ != DNode::NEW && y->g_ <= k_old && x->g_ > y->g_ + getCost(y, x)){
                    x->pid_ = y->id_;
                    x->g_ = y->g_ + getCost(y, x);
                }
            }
        }
        
        if (k_old == x->g_){
            for (DNodePtr y : neigbours){
                if (y->t_ == DNode::NEW || ((y->pid_ == x->id_) && (y->g_ != x->g_ + getCost(x, y))) || ((y->pid_ != x->id_) && (y->g_ > x->g_ + getCost(x, y)))){
                    y->pid_ = x->id_;
                    insert(y, x->g_ + getCost(x, y));
                }
            }
        } else {
            for (DNodePtr y : neigbours){
                
                if (y->t_ == DNode::NEW || ((y->pid_ == x->id_) && (y->g_ != x->g_ + getCost(x, y)))){
                    y->pid_ = x->id_;
                    insert(y, x->g_ + getCost(x, y));
                } else if (y->pid_ != x->id_ && (y->g_ > x->g_ + getCost(x, y))) {
                    insert(x, x->g_);
                } else if (y->pid_ != x->id_ && (x->g_ > y->g_ + getCost(y, x)) && y->t_ == DNode::CLOSED && (y->g_ > k_old)){
                    insert(y, y->g_);
                }
            }
        }

        return open_list_.begin()->first;
    }

    bool DStar::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand){
        memcpy(last_global_costmap_, curr_global_costmap_, ns);
        memcpy(curr_global_costmap_, global_costmap, ns);

        expand.clear();

        if(goal_.x_ != goal.x_ || goal_.y_ != goal.y_){
            
            reset();
            goal_ = goal;

            DNodePtr start_ptr = map_[start.x_][start.y_];
            DNodePtr goal_ptr = map_[goal.x_][goal.y_];

            goal_ptr->g_ = 0;
            insert(goal_ptr, 0);
            while (1){
                double k_min = processState();
                if (k_min == -1 || start_ptr->t_ == DNode::CLOSED){
                    break;
                }
            }

            path_.clear();
            extractPath(start, goal);

            expand = expand_;
            path = path_;

            return true;
        } else {
            
            Node state = getState(start);
            
            for (int i = -WINDOW_SIZE / 2; i < WINDOW_SIZE / 2; i++){
                for (int j = -WINDOW_SIZE / 2; j < WINDOW_SIZE / 2; j++){

                    int x_n = state.x_ + i; 
                    int y_n = state.y_ + j;

                    if (x_n < 0 || x_n > nx - 1 || y_n < 0 || y_n > ny - 1){
                        continue;
                    }

                    DNodePtr x = map_[x_n][y_n];
                    std::vector<DNodePtr> neigbours;
                    getNeighbours(x, neigbours);

                    int idx = grid2Index(x_n, y_n);
                    if (curr_global_costmap_[idx] != last_global_costmap_[idx]){
                        modify(x);
                        for (DNodePtr y : neigbours){
                            modify(y);
                        }
                    }
                }
            }
            
            DNodePtr x = map_[state.x_][state.y_];
            while (1){
                double k_min = processState();
                if (k_min >= x->g_ || k_min == -1){
                    break;
                }
            }

            path_.clear();
            extractPath(state, goal);

            expand = expand_;
            path = path_;

            return true;
        }
    }

    int DStar::grid2Index(int x, int y){
        return x + nx * y;
    }

    void DStar::index2Grid(int i, int& x, int& y){
        x = i % nx;
        y = i / nx;
    }
}