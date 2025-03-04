#include "hybrid_astar_searcher/hybrid_astar.h"

using namespace std;

namespace planning
{
    HybridAstar::HybridAstar(const nav_msgs::OccupancyGridConstPtr& grid, ros::NodeHandle &nh) :
        grid_(grid){
        xy_resolution_ = grid_->info.resolution;
        theta_resolution_ = 0.1;
        x_min_ = grid_->info.origin.position.x;
        x_max_ = x_min_ + grid_->info.width * xy_resolution_;
        y_min_ = grid_->info.origin.position.y;
        y_max_ = y_min_ + grid_->info.height * xy_resolution_;
        map_size_x_ = grid_->info.width; 
        map_size_y_ = grid_->info.height;
        configurationSpace.updateGrid(grid);
        grid_search_ptr_ = make_unique<GridSearch>(grid_, configurationSpace);
        if (!nh.getParam("/simple_astar_test/Hastar/dubins_radius", dubins_radius_)) {
            nh.getParam("/hybrid_astar_dubins_opt/Hastar/dubins_radius", dubins_radius_);
        }
        ROS_INFO("dubins_radius_=%f", dubins_radius_);
        dubins_generator_ =  std::make_shared<DubinsStateSpace>(dubins_radius_, step_size_); //path_step_size = step_size_
        ROS_INFO("HybridAstar Init!");
    }

    HybridAstar::~HybridAstar() {}

    bool HybridAstar::plan(Vec3d start_pose, Vec3d goal_pose, HybridAstarResult &result) {
        auto dp_map_start = chrono::high_resolution_clock::now();

        // grid_search_ptr_->generateDpMap({goal_pose(0), goal_pose(1)}, {start_pose(0), start_pose(1)});
        grid_search_ptr_->InitLookUpTable(goal_pose, start_pose);
        auto dp_map_end = chrono::high_resolution_clock::now();
        chrono::duration<double> dp_map_use_time = dp_map_end - dp_map_start;
        cout << "Distance lookup table generation time: " << dp_map_use_time.count() * 1000 << "ms" << endl;

        Vec3i start_pose_map = getIndexFromPose(start_pose);
        Vec3i goal_pose_map = getIndexFromPose(goal_pose);
        start_node_.reset(new Node3d({start_pose(0)}, {start_pose(1)}, {start_pose(2)}));
        goal_node_.reset(new Node3d({goal_pose(0)}, {goal_pose(1)}, {goal_pose(2)}));

        if (!validityCheck(start_node_)) {
            cout << "Invalid start point" << endl;
            return false;
        }
        if (!validityCheck(goal_node_)) {
            cout << "Invalid goal point" << endl;
            return false;
        }
        
        double path_length = std::numeric_limits<double>::max();
        open_set_.clear();
        close_set_.clear();
        open_pq_ = decltype(open_pq_)();
        final_node_ = nullptr;
        auto hybrid_astar_start = chrono::high_resolution_clock::now();
        start_node_->setIndex(start_pose_map, map_size_x_, map_size_y_);
        open_set_.emplace(start_node_->getIndex(), start_node_);
        open_pq_.emplace(start_node_->getIndex(), start_node_->getCost());
        int explored_node_num = 0;
        vis.vis_nodes_clear();
        while (!open_pq_.empty()) {
            int cur_index = open_pq_.top().first;
            // cout << "cur_index:" << cur_index << endl;
            open_pq_.pop();
            shared_ptr<Node3d> cur_node = open_set_[cur_index];
            
            // Visualize the growing node
            vis.publishExploredNodes({cur_node->getX(), cur_node->getY(), cur_node->getTheta()});
            
            // if (explored_node_num > 2000) {
            //     explored_node_num=0;
            //     cout << "Search failed" << endl;
            //     return false;
            //     // continue;
            // }

            if (AnalyticExpansion(cur_node)) {
                cout << "Direct connection successful" << endl;
                break;
            }

            close_set_.emplace(cur_index, cur_node);

            for (int i = 0; i < next_node_num_; ++i) {
                shared_ptr<Node3d> next_node = nextNodeGenerator(cur_node, i);
                if (next_node == nullptr) {
                    continue;
                }            
                // cout << "node" << i << "is in map" << endl;

                Vec3i index111 = getIndexFromPose({next_node->getX(), next_node->getY(), next_node->getTheta()});
                next_node->setIndex(index111, map_size_x_, map_size_y_);
                // cout << "node" << i << "index :" << next_node->getIndex() << endl;
                if (close_set_.find(next_node->getIndex()) != close_set_.end()) {
                    // cout << "node " << i << "is in close set" << endl;
                    continue;
                }
                if (!validityCheck(next_node)) {
                    continue;
                }
                // cout << "node " << i << "is valid" << endl;

                if (open_set_.find(next_node->getIndex()) == open_set_.end()) {
                    explored_node_num++;
                    // cout << "Number of explored nodes: " << explored_node_num << endl;
                    
                    calculateNodeCost(cur_node, next_node);
                    
                    open_pq_.emplace(next_node->getIndex(), next_node->getCost());
                    open_set_.emplace(next_node->getIndex(), next_node);
                }
            }
        }

        if (final_node_ == nullptr) {
            cout << "Search failed" << endl;
            return false;
        }
        if (!getHybridAstarResult(result)) {
            cout << "No solution found" << endl;
        }
        auto hybrid_astar_end = chrono::high_resolution_clock::now();
        cout << "Number of explored nodes: " << explored_node_num << endl;
        cout << "Path length: " << result.total_length << "m" << endl;

        // Total planning time (without backend optimization)
        chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start;
        cout << "Planning time: " << hybrid_astar_use_time.count() * 1000 << "ms" << endl;

        return true;
    }

    bool HybridAstar::getHybridAstarResult(HybridAstarResult &result) {
        shared_ptr<Node3d> cur_node = final_node_;
        auto l1 = cur_node->getXs().size();
        vector<double> res_x;
        vector<double> res_y;
        vector<double> res_theta;
        while (cur_node->getParent() != nullptr) {
            vector<double> x = cur_node->getXs();
            vector<double> y = cur_node->getYs();
            vector<double> theta = cur_node->getThetas();
            reverse(x.begin(), x.end());
            reverse(y.begin(), y.end());
            reverse(theta.begin(), theta.end());

            x.pop_back();
            y.pop_back();
            theta.pop_back();

            res_x.insert(res_x.end(), x.begin(), x.end());
            res_y.insert(res_y.end(), y.begin(), y.end());
            res_theta.insert(res_theta.end(), theta.begin(), theta.end());
            cur_node = cur_node->getParent();
        }
        res_x.push_back(cur_node->getX());
        res_y.push_back(cur_node->getY());
        res_theta.push_back(cur_node->getTheta());
        reverse(res_x.begin(), res_x.end());
        reverse(res_y.begin(), res_y.end());
        reverse(res_theta.begin(), res_theta.end());

        result.x = res_x;
        result.y = res_y;
        result.theta = res_theta;

        auto l2 = (res_x.size() - l1) * step_size_;
        // Total length
        result.total_length = db_length + l2;
        return true;
    }

    shared_ptr<Node3d> HybridAstar::nextNodeGenerator(shared_ptr<Node3d> cur_node, int next_node_idx) {
        double steering = 0.0;
        double traveled_distance = 0.0;
        
        // double delta_steering = 2 * max_steer_angle_ / (next_node_num_ / 2 - 1);
        // double delta_steering = 2 * max_steer_angle_ / (next_node_num_ - 2);
        double delta_steering = 2 * max_steer_angle_ / (next_node_num_ - 1);
        // cout << "Expanding node" << endl;
        // cout << "next_node_idx :" << next_node_idx << endl;

        /*First, the direction of movement (forward and reverse) can be distinguished based on the comparison between next_node_index and next_node_num_.
        The first half of next_node_num_ is forward, and the second half is reverse.
        steering = initial offset + unit interval × index*/
        /****************************************************************************
        *      Steering is defined as left turn as "+", right turn as "-", as defined in carsim.
        *      (-max_steer_angle_) 4   < \     / >   3  (max_steer_angle_)
        *                                 \   /
        *     (Reverse direction)  5 <----- O -----> 2  (Forward direction)
        *                                 /   \
        *       (max_steer_angle_) 6   < /     \ >   1 (-max_steer_angle_)
        * **************************************************************************/
        steering = -max_steer_angle_ + delta_steering * next_node_idx;
        traveled_distance = step_size_;            
        double steering1 = steering / 180 * M_PI; 

        // double arc = sqrt(2) * xy_resolution_; 
        double arc = 2.0;
        vector<double> traversed_x;
        vector<double> traversed_y;
        vector<double> traversed_theta;
        double cur_x = cur_node->getX();
        double cur_y = cur_node->getY();
        double cur_theta = cur_node->getTheta();
        traversed_x.push_back(cur_x);
        traversed_y.push_back(cur_y);
        traversed_theta.push_back(cur_theta);

        for (int i = 0; i < arc / step_size_; ++i) {
            double next_x = cur_x + traveled_distance * cos(cur_theta);
            double next_y = cur_y + traveled_distance * sin(cur_theta);
            double next_theta = cur_theta + traveled_distance * tan(steering1) / wheel_base_;
            // cout << "turn radius:" << 1 / (tan(steering1) / wheel_base_) << endl;
            mod2Pi(next_theta);
            
            traversed_x.push_back(next_x);
            traversed_y.push_back(next_y);
            traversed_theta.push_back(next_theta);

            cur_x = next_x;
            cur_y = next_y;
            cur_theta = next_theta;
        }

        // Final node:
        if (traversed_x.back() < x_min_ || traversed_x.back() > x_max_ ||
            traversed_y.back() < y_min_ || traversed_y.back() > y_max_) {
            // cout << "Exceeds boundary" << endl;
            return nullptr;
        }
        
        shared_ptr<Node3d> next_node = make_shared<Node3d>(traversed_x, traversed_y, traversed_theta);
        next_node->setParent(cur_node);
        next_node->setDirection(traveled_distance > 0.0);
        next_node->setSteering(steering);
        return next_node;

    }


    // Calculate "g" in f = h + g: movement cost G
    double HybridAstar::TrajCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
        double piecewise_cost = 0.0;
        // based on penalty coefficients to select a better path during path search.
        if (next_node->getDirection()) {
            piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_forward_penalty_;
        } else {
            piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_back_penalty_;
        }

        if (cur_node->getDirection() != next_node->getDirection()) {
            // Penalty for gear switching
            piecewise_cost += traj_gear_switch_penalty_;
        }
        piecewise_cost += traj_steer_change_penalty_ * std::abs(
            next_node->getSteering() - cur_node->getSteering()
        );
        piecewise_cost += traj_steer_penalty_ * std::abs(next_node->getSteering());

        return piecewise_cost;
        // Various penalties to be added
    }

    // Calculate the path cost and heuristic cost separately
    void HybridAstar::calculateNodeCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
        // The cost G of the trajectory traveled in Hybrid A*
        next_node->setTrajCost(cur_node->getTrajCost() + TrajCost(cur_node, next_node));

        double holo_heu_cost = 0.0;
        // holo_heu_cost = grid_search_ptr_->lookupInDpMap({next_node->getX(), next_node->getY()});
        // cout << "holo heu cost:" << holo_heu_cost << endl;

        // Query the approximate A* distance value
        holo_heu_cost = grid_search_ptr_->CheckLookUpTable({next_node->getX(), next_node->getY()});

        double DB_heu_cost;
        try {
            // auto non_holo_heu_start = chrono::system_clock::now();
            ob::StateSpacePtr space(std::make_shared<ob::DubinsStateSpace>(dubins_radius_));
            ob::SE2StateSpace::StateType * dbStart = (ob::SE2StateSpace::StateType*)space->allocState();
            ob::SE2StateSpace::StateType* dbEnd = (ob::SE2StateSpace::StateType*)space->allocState();
            dbStart->setXY(next_node->getX(), next_node->getY());
            dbStart->setYaw(next_node->getTheta());
            dbEnd->setXY(goal_node_->getX(), goal_node_->getY());
            dbEnd->setYaw(goal_node_->getTheta());
            DB_heu_cost = space->distance(dbStart, dbEnd);

        } catch (const std::exception& e) {
            std::cerr << "An exception occurred: " << e.what() << std::endl;
            ROS_ERROR("An exception occurred.");
        }

        next_node->setHeuristicCost(max(DB_heu_cost, holo_heu_cost));
    }

    vector<Vec2d> HybridAstar::calculateCarBoundingBox(Vec3d pose) {
        vector<Vec2d> vertices;
        double shift_distance = length_ / 2 - back_edge_to_center_;
        Vec2d center = {pose(0) + shift_distance * std::cos(pose(2)),
                        pose(1) + shift_distance * std::sin(pose(2))};
        // Set an inflation value to enlarge the outline (conservative)
        const double dx1 = std::cos(pose(2)) * (length_ / 2 + inflation_value );
        const double dy1 = std::sin(pose(2)) * (length_ / 2 + inflation_value);
        const double dx2 = std::sin(pose(2)) * (width_ / 2 + inflation_value);
        const double dy2 = -std::cos(pose(2)) * (width_ / 2 + inflation_value);

        // Four vertices
        vertices.emplace_back(center(0) + dx1 + dx2, center(1) + dy1 + dy2);
        vertices.emplace_back(center(0) + dx1 - dx2, center(1) + dy1 - dy2);
        vertices.emplace_back(center(0) - dx1 - dx2, center(1) - dy1 - dy2);
        vertices.emplace_back(center(0) - dx1 + dx2, center(1) - dy1 + dy2); 
        return vertices;
    }

    // Check if a point on a line is in an obstacle (interpolation)
    bool HybridAstar::isLinecollision(double x0, double y0, double x1, double y1) {
        auto check_point_num = static_cast<int>(std::hypot(x1 - x0, y1 - y0) / xy_resolution_);

        double delta_x = (x1 - x0) / check_point_num;
        double delta_y = (y1 - y0) / check_point_num;

        double cur_x = x0;
        double cur_y = y0;

        auto mapObstacle = [&](double x, double y) {
            auto i = std::floor((x - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
            auto j = std::floor((y - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
            auto idx = i + j * grid_->info.width;
            return grid_->data[idx] == 100 || grid_->data[idx] == -1;
        };

        for (int i = 0; i < check_point_num; ++i) {
            if (!isInMap(cur_x, cur_y)) {
                return true;
            }

            if(mapObstacle(cur_x, cur_y))  return true;
            cur_x += delta_x;
            cur_y += delta_y;
        }
        return false;
    }

    // Verify if the node is feasible. It requires obtaining the car's outline and then using the above method to check if the four edges pass through obstacles.
    bool HybridAstar::validityCheck(std::shared_ptr<Node3d> node) {
        int node_step_size = node->getStepSize();
        const auto traversed_x = node->getXs();
        const auto traversed_y = node->getYs();
        const auto traversed_theta = node->getThetas();

        int start_index = 0;
        if (node_step_size > 1) {  
            start_index = 1;   
        }

        for (int i = start_index; i < node_step_size; ++i) {
            if (!isInMap(traversed_x[i], traversed_y[i])) {
                return false;
            }

            // Method 1: Use line collision detection at each waypoint
            // Generate the four vertices of the car
            // auto vertices = calculateCarBoundingBox({traversed_x[i], traversed_y[i], traversed_theta[i]});
            // // Check if the four edges collide
            // for (int i = 0; i < vertices.size(); ++i) {
            //     if (isLinecollision(vertices[i].x(), vertices[i].y(), 
            //                         vertices[(i+1) % 4].x(), vertices[(i+1) % 4].y())){
            //         return false;                    
            //     }
            // }

            // Method 2: Occupancy grid template
            if(!configurationSpace.isTraversable({traversed_x[i], traversed_y[i], traversed_theta[i]}))
                return false;
        }
        return true;
    }

    Vec3i HybridAstar::getIndexFromPose(Vec3d pose) {
        Vec3i index;
        index(0) = std::floor((pose(0) - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
        index(1) = std::floor((pose(1) - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
        mod2Pi(pose(2));
        index(2) = static_cast<int>((pose(2)) / theta_resolution_);
        return index;
    }

    // Directly connect to the goal using a Dubins (DB) curve. Attempt to connect the current point to the goal point using a DB curve.
    bool HybridAstar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
        std::shared_ptr<DubinsPath_> dubins_to_check = std::make_shared<DubinsPath_>();
        // Generate the shortest Dubins curve
        dubins_generator_->ShortestDBP(current_node, goal_node_, *dubins_to_check);

        // std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(reeds_shepp_to_check->x, reeds_shepp_to_check->y, reeds_shepp_to_check->phi));

        std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
        dubins_to_check->x, dubins_to_check->y, dubins_to_check->phi));

        if (!validityCheck(node)) {
            return false;
        }
        std::shared_ptr<Node3d> goal_node = std::shared_ptr<Node3d>(new Node3d(
            dubins_to_check->x, dubins_to_check->y, dubins_to_check->phi));
        db_length = dubins_to_check->total_length;
        goal_node->setParent(current_node);
        close_set_.emplace(goal_node->getIndex(), goal_node);
        final_node_ = goal_node;
        return true;
    }

    inline void HybridAstar::mod2Pi(double &angle) {
        if (angle >  M_PI)  angle -= 2 * M_PI;
        if (angle < -M_PI)  angle += 2 * M_PI;
    }

    inline bool HybridAstar::isInMap(double x, double y) {
        auto i = std::floor((x - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
        auto j = std::floor((y - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
        return (i >= 0 && j >= 0 && i < grid_->info.width && j < grid_->info.height);
    }

} // namespace planning

