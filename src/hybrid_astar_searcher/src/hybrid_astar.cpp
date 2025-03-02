#include "hybrid_astar_searcher/hybrid_astar.h"

using namespace std;

namespace planning
{
    // HybridAstar::HybridAstar(grid_map::GridMap map) {
    //     //     map_ = map;
    //     //     xy_resolution_ = map.getResolution();
    //     //     theta_resolution_ = 0.1;
    //     //     x_min_ = map.getStartIndex()(0) * xy_resolution_ - 25;
    //     //     x_max_ = x_min_ + map.getLength().x();
    //     //     y_min_ = map.getStartIndex()(1) * xy_resolution_ -25;
    //     //     y_max_ = y_min_ + map.getLength().y();
            
    //     //     map_size_x_ = map.getSize()(0); 
    //     //     map_size_y_ = map.getSize()(1);
            
    //     //     cout << "map_size_x_" << map_size_x_ << endl;
    //     //     cout << "getStartIndex()(0)" << map.getStartIndex()(0) << endl;
    //     //     grid_search_ptr_ = make_unique<GridSearch>(map_);
    //     //     // Smart pointer
    //     //     // reed_shepp_generator_ =  std::make_shared<ReedShepp>(0.3, 0.7);
    //     //     // RS (maximum curvature, step size)
    //     //     dubins_generator_ =  std::make_shared<DubinsStateSpace>(min_turn_radius_, path_step_size);
    //     //     // Dubins curve
    //     // }

    HybridAstar::HybridAstar(const nav_msgs::OccupancyGridConstPtr& grid) :
        grid_(grid){
        // grid_ = grid;
        xy_resolution_ = grid_->info.resolution;
        theta_resolution_ = 0.1;
        // Radian, angular resolution
        x_min_ = grid_->info.origin.position.x;
        // The value of the map origin (first grid point) in the frame_id coordinate system ("map")
        x_max_ = x_min_ + grid_->info.width * xy_resolution_;
        // grid.info.width: width: number of grids in the x direction
        y_min_ = grid_->info.origin.position.y;
        y_max_ = y_min_ + grid_->info.height * xy_resolution_;

        map_size_x_ = grid_->info.width; 
        map_size_y_ = grid_->info.height;

        // Grids used for collision detection with the coverage grid template
        configurationSpace.updateGrid(grid);
        grid_search_ptr_ = make_unique<GridSearch>(grid_, configurationSpace);
        // Smart pointer

        // reed_shepp_generator_ =  std::make_shared<ReedShepp>(0.3, 0.7);
        // RS (maximum curvature, step size)
        // This 2.2 (curvature radius) should be set reasonably
        dubins_generator_ =  std::make_shared<DubinsStateSpace>(dubins_radius_, path_step_size);
        // Dubins curve: considering the direction when reaching the target point
        // dubins_generator_ =  std::make_shared<DubinsStateSpace>(0.8, path_step_size);
        // Dubins curve

        ROS_INFO("HybridAstar Init!");
    }

    HybridAstar::~HybridAstar() {}

    /**
    * @brief Hybrid A* search
    * 
    * @param start_pose 
    * @param goal_pose 
    * @param result 
    * @return true 
    * @return false 
    */
    bool HybridAstar::plan(Vec3d start_pose, Vec3d goal_pose, HybridAstarResult &result) {

        // Generate Heuristic lookup table for estimated cost H
        auto dp_map_start = chrono::high_resolution_clock::now();

        // Use dynamic programming (DP) to calculate the heuristic cost from the goal point to a certain point (using the goal point as the starting point of DP)
        // While generating the graph, obtain the cost from the goal point to any point in the graph, and later only need to look up the table, trading space for time
        // grid_search_ptr_->generateDpMap({goal_pose(0), goal_pose(1)}, {start_pose(0), start_pose(1)});

        // Initialize the approximate A* distance values, passing the real coordinates of the start and goal points
        grid_search_ptr_->InitLookUpTable(goal_pose, start_pose);
        auto dp_map_end = chrono::high_resolution_clock::now();
        chrono::duration<double> dp_map_use_time = dp_map_end - dp_map_start;
        cout << "Distance lookup table generation time: " << dp_map_use_time.count() * 1000 << "ms" << endl;

        // Calculate grid index
        Vec3i start_pose_map = getIndexFromPose(start_pose);
        Vec3i goal_pose_map = getIndexFromPose(goal_pose);

        start_node_.reset(new Node3d({start_pose(0)}, {start_pose(1)}, {start_pose(2)}));
        goal_node_.reset(new Node3d({goal_pose(0)}, {goal_pose(1)}, {goal_pose(2)}));

        // Check if the start_node_ and goal_node_ collide
        if (!validityCheck(start_node_)) {
            cout << "Invalid start point" << endl;
            return false;
        }

        if (!validityCheck(goal_node_)) {
            cout << "Invalid goal point" << endl;
            return false;
        }
        
        double path_length = std::numeric_limits<double>::max();
        // auto hybrid_astar_start_ = chrono::high_resolution_clock::now(); // Initialization
        // auto hybrid_astar_end_ = chrono::high_resolution_clock::now();
        // HybridAstarResult result_ = {};
        // Or dynamically allocate the number of planning failures --------------
            // for(auto j=0; j<3; j++){
                // For each planning, clear the previous cached data
                open_set_.clear();
                close_set_.clear();
                open_pq_ = decltype(open_pq_)();
                // Clear variables
                final_node_ = nullptr;
    
                // Start timing
                auto hybrid_astar_start = chrono::high_resolution_clock::now();
    
                // ROS_INFO("1");
    
                // Add the start point start_node_ to the open_pq_ queue and open_set_ set
                start_node_->setIndex(start_pose_map, map_size_x_, map_size_y_);
                open_set_.emplace(start_node_->getIndex(), start_node_);
                // The open_pq_ queue is sorted by node cost from smallest to largest
                open_pq_.emplace(start_node_->getIndex(), start_node_->getCost());
    
                // ROS_INFO("2");
    
                int explored_node_num = 0;
                vis.vis_nodes_clear();
                while (!open_pq_.empty()) {
                    // cout << "Loop" << endl;
                    int cur_index = open_pq_.top().first;
                    // cout << "cur_index:" << cur_index << endl;
                    open_pq_.pop();
                    // Pop the first node current_node from the queue
                    shared_ptr<Node3d> cur_node = open_set_[cur_index];
                    
                    // Visualize the growing node
                    vis.publishExploredNodes({cur_node->getX(), cur_node->getY(), cur_node->getTheta()});
                    
                    
                    // if (explored_node_num > 2000) {
                    //     explored_node_num=0;
                    //     cout << "Search failed" << endl;
                    //     return false;
                    //     // continue;
                    // }
    
                    // Check if we can directly connect to the goal using a Dubins curve
            /*During the execution of Hybrid A*, intermittently trigger Dubins curves to generate a curve connecting the current pose and the goal pose, then check if it meets collision conditions. If it does, the problem is solved. If not, discard the result and continue with the normal search logic. It's like occasionally trying a shortcut; if it works, great, if not, no harm done, just keep going.*/
            /*First generate a kinematically feasible path, then verify if this path meets collision conditions.*/
            // Once the ReedShepp curve segment connects the current node and the goal node, and passes the safety check, the planning for this round ends.
            // However, this may cause curvature discontinuity at the connection point, which needs to be optimized later.
                    if (AnalyticExpansion(cur_node)) {
                        // It will eventually connect successfully
                        cout << "Direct connection successful" << endl;
                        // grid_search_ptr_->clear();
                        break;
                    }
                    // cout << "Direct connection failed" << endl;
            /*Check if the current node can connect to the goal node through an RS curve to generate a collision-free path AnalyticExpansion. If so, the complete path is found, break.
              Otherwise, add current_node to close_set_*/
                    close_set_.emplace(cur_index, cur_node);
    
                    for (int i = 0; i < next_node_num_; ++i) { // next_node_num_=6, 6 orientations
                        // cout << "expand " << i << "node" << endl;
                        // Generate the next node next_node reachable from current_node
                        // Starting from current_node, use different steering angles to move forward by arc (diagonal length)
                        // The last path point in a grid is called a node. There can be multiple path points in this grid, and the next_node of this node must be in an adjacent grid.
                        shared_ptr<Node3d> next_node = nextNodeGenerator(cur_node, i);
                        // Out of bounds
                        if (next_node == nullptr) {
                            continue;
                        }            
                        // cout << "node" << i << "is in map" << endl;
    
                        // Check if it is in the closelist
                        // Compare the index with nodes in closet_set_ to eliminate already traversed nodes
                        Vec3i index111 = getIndexFromPose({next_node->getX(), next_node->getY(), next_node->getTheta()});
                        next_node->setIndex(index111, map_size_x_, map_size_y_);
                        // cout << "node" << i << "index :" << next_node->getIndex() << endl;
                        // If in close list, skip
                        if (close_set_.find(next_node->getIndex()) != close_set_.end()) {
                            // cout << "node " << i << "is in close set" << endl;
                            continue;
                        }
                        // Collision check for next_node, eliminate nodes that would collide on the path
                        // If collision exists, skip
                        if (!validityCheck(next_node)) {
                            continue;
                        }
                        // cout << "node " << i << "is valid" << endl;
    
            /*If not explored yet, initialize 
              open_set_ is actually a combination of close_set_ and open_pq_
              The index of each grid is determined by x_grid_, y_grid_, and phi_grid_, not just x_grid_ and y_grid,
              but here phi_grid_ is discretized according to phi_grid_resolution, so there is still some overlap.*/
    
                /*Check if the node is already in the to-be-traversed set open_set_. If not, calculate the node cost and add it to open_set_ and open_pq_ queue.*/
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
                    // continue; // Enter the next loop
                }
                // Use backtracking to get the path, starting from final_node_
                if (!getHybridAstarResult(result)) {
                    cout << "No solution found" << endl;
                    // continue; // Enter the next loop
                }
    
    
                auto hybrid_astar_end = chrono::high_resolution_clock::now();
                cout << "Number of explored nodes: " << explored_node_num << endl;
                cout << "Path length: " << result.total_length << "m" << endl;
    
                // Total planning time (without backend optimization)
                chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start;
                cout << "Planning time: " << hybrid_astar_use_time.count() * 1000 << "ms" << endl;
    
                // if(path_length > result.total_length){
                //     path_length = result.total_length;
                //     // Save the timestamp of this iteration
                //     // hybrid_astar_start_ = hybrid_astar_start;
                //     // hybrid_astar_end_ = hybrid_astar_end;
                //     result_ = HybridAstarResult();
                //     result_ = result;
                //     // path2smooth.clear();
                //     // for (int i = 0; i < result.x.size(); ++i) {
                //     //     path2smooth.push_back({result.x[i], result.y[i], result.theta[i]}); // Load the path to be smoothed
                //     // }
                // }
    
        // }
        // result = HybridAstarResult();
        // result = result_;
        // Empty value
        // if(result.x.empty()) return false;
        return true;
    
    }


    // Polyline A* planning
    // void HybridAstar::Astarplan(Vec2d start_pose, Vec2d goal_pose) {
    //     // Original A*, only taking the start and end points of each straight segment, which does not affect visualization
    //     std::vector<Point2D> ret = grid_search_ptr_->calculateHeuByAstar(start_pose, goal_pose);
    //     // The straightened A* polyline path
    //     std::vector<Point2D> ret_ = grid_search_ptr_->PathShorten(ret);
    //     // Visualization
    //     // Before visualization, convert row and column indices to real coordinates
    //     std::vector<geometry_msgs::PoseStamped> mret;
    //     std::vector<geometry_msgs::PoseStamped> mret_;
    //     for(auto const& idx : ret){
    //         geometry_msgs::PoseStamped p;
    //         p.header.frame_id = "map";
    //         p.pose.position.x = (idx.x+0.5) * xy_resolution_ + x_min_;
    //         p.pose.position.y = (idx.y+0.5) * xy_resolution_ + y_min_;
    //         mret.push_back(p);
    //     }
    //     for(auto const& idx : ret_){
    //         geometry_msgs::PoseStamped p;
    //         p.header.frame_id = "map";
    //         p.pose.position.x = (idx.x+0.5) * xy_resolution_ + x_min_;
    //         p.pose.position.y = (idx.y+0.5) * xy_resolution_ + y_min_;
    //         mret_.push_back(p);
    //     }
    //     // vis.vis_clear();
    //     vis.publishPrimitiveAstar(mret);
    //     vis.publishSimpleAstar(mret_);
    // }


    // Backtracking to get the path, starting from final_node_
    bool HybridAstar::getHybridAstarResult(HybridAstarResult &result) {
        shared_ptr<Node3d> cur_node = final_node_;
        /*The last directly connected part*/

        auto l1 = cur_node->getXs().size();

        vector<double> res_x;
        vector<double> res_y;
        vector<double> res_theta;
        // Core while loop, the backtracking process
        while (cur_node->getParent() != nullptr) {
            vector<double> x = cur_node->getXs();
            vector<double> y = cur_node->getYs();
            vector<double> theta = cur_node->getThetas();
            // Reverse the traversed_x_, y_, phi_ to get the correct order of the subsequence
            reverse(x.begin(), x.end());
            reverse(y.begin(), y.end());
            reverse(theta.begin(), theta.end());

            x.pop_back();
            // Remove the last element from the x vector
            y.pop_back();
            theta.pop_back();

            res_x.insert(res_x.end(), x.begin(), x.end());
            res_y.insert(res_y.end(), y.begin(), y.end());
            res_theta.insert(res_theta.end(), theta.begin(), theta.end());
            cur_node = cur_node->getParent();
        }
        // At this point, cur_node is the start point
        res_x.push_back(cur_node->getX());
        res_y.push_back(cur_node->getY());
        res_theta.push_back(cur_node->getTheta());
        // The traversed_x_, y_, phi_ were reversed earlier, so reverse them back
        reverse(res_x.begin(), res_x.end());
        reverse(res_y.begin(), res_y.end());
        reverse(res_theta.begin(), res_theta.end());

        // Assign the goal point sequence
        // res_x.push_back(goal_node_->getX());
        // res_y.push_back(goal_node_->getY());
        // res_theta.push_back(goal_node_->getTheta());     

        // At this point, result only contains pose information x, y, phi
        result.x = res_x;
        result.y = res_y;
        result.theta = res_theta;

        // Calculate the length of the path:
        // auto l2 = (res_x.size()- l1 -1)*step_size_;
        auto l2 = (res_x.size() - l1) * step_size_;
        // ROS_INFO("4");
        // Total length
        result.total_length = db_length + l2;

        // cout << "db_length:" << db_length << endl;
        // cout << "l1:" << l1 << endl;
        // cout << "res size:" << res_x.size() << endl;
        // cout << "l2:" << l2 << endl;
        // cout << "Result obtained" << endl;
        return true;
    }


    // From current_node, move forward with a certain steering (length is the diagonal length of a grid)
    // The key point of expanding nodes is to incorporate the constraints of the vehicle kinematics model, 
    // and search adjacent grids based on the limited steering angle.
    // Expanding a node means expanding a grid, but it will generate multiple path points within the same grid.
    shared_ptr<Node3d> HybridAstar::nextNodeGenerator(shared_ptr<Node3d> cur_node, int next_node_idx) {
        double steering = 0.0;
        double traveled_distance = 0.0;
        
        // double delta_steering = 2 * max_steer_angle_ / (next_node_num_ / 2 - 1);
        // double delta_steering = 2 * max_steer_angle_ / (next_node_num_ - 2);
        double delta_steering = 2 * max_steer_angle_ / (next_node_num_ - 1);
        // cout << "Expanding node" << endl;
        // cout << "next_node_idx :" << next_node_idx << endl;

        // Get forward/reverse moving distance and steering angle
        /*First, the direction of movement (forward and reverse) can be distinguished based on the comparison between next_node_index and next_node_num_.
        The first half of next_node_num_ is forward, and the second half is reverse.
        steering = initial offset + unit interval × index*/
        /****************************************************************************
        *      Steering is defined as left turn as "+", right turn as "-", as defined in carsim.
        *      (-max_steer_angle_) 4   < \     / >   3  (max_steer_angle_)
        *                                 \   /
        *               (Reverse direction)  5 <----- O -----> 2  (Forward direction)
        *                                 /   \
        *       (max_steer_angle_) 6   < /     \ >   1 (-max_steer_angle_)
        * **************************************************************************/

        /*Why is the steering angle calculated this way?
        First, the direction of movement (forward and reverse) can be distinguished based on the comparison between next_node_index and next_node_num_.
        The if-else here is used to distinguish between forward and reverse directions.
        Secondly, the vehicle can turn both left and right in its current posture, so the steering angle
        actually ranges from [-max_steer_angle_, max_steer_angle_], and in either forward or reverse direction,
        it can take next_node_num_/2 valid values.
        That is, divide [-max_steer_angle_, max_steer_angle_] into (next_node_num_/2-1) parts.
        Therefore, steering = initial offset + unit interval × index
        The sign of the steering angle depends on the direction of the turn, not the direction of movement.*/
        // if (next_node_idx < next_node_num_ / 2) {
        //     // Forward steering calculation
        //     steering = -max_steer_angle_ + delta_steering * next_node_idx;
        //     traveled_distance = step_size_;
        // } else {
        //     // Reverse steering calculation
        //     steering = -max_steer_angle_ + delta_steering * (next_node_idx - next_node_num_ / 2);
        //     traveled_distance = -step_size_;
        // }

        // Only reverse
        // if (next_node_idx == next_node_num_-1) {
        //     steering = 0.;
        //     traveled_distance = -step_size_;
        // }
        // The remaining 5 directions are forward
        // else {
            steering = -max_steer_angle_ + delta_steering * next_node_idx;
            traveled_distance = step_size_;            
        // }
        // cout << "steer:" << steering << endl;
        double steering1 = steering / 180 * M_PI; 
        // Radian, front wheel steering angle
        // cout << "steer1:" << steering1 << endl;


        /* Process: Move with the steering angle through the bicycle kinematics model for the length of the arc, 
        and record the recursive process (i.e., the points along the way) traversed_x_, traversed_y_, traversed_phi_ */

        // Store a series of intermediate points of the node
        double arc = sqrt(2) * xy_resolution_; 
        // Can set the desired length, this is the diagonal length of the grid (converted to meters)
        arc = 2.0; // Can set the desired length in meters
        // cout << "111111" << endl;
        vector<double> traversed_x;
        vector<double> traversed_y;
        vector<double> traversed_theta;
        double cur_x = cur_node->getX();
        double cur_y = cur_node->getY();
        double cur_theta = cur_node->getTheta();
        // cout << "11222" << endl;
        // Add the previous node as the first member
        traversed_x.push_back(cur_x);
        traversed_y.push_back(cur_y);
        traversed_theta.push_back(cur_theta);
        // cout << "arc / step_size_:" << arc / step_size_ << endl;
        // cout << "112223333" << endl;

        // Move forward from current_node by arc (i.e., the length of the diagonal), this is just one direction
        // Move from the current grid to the next grid, there may be multiple path points within a grid = arc / step_size_, 
        // so one direction will have several small path points
        for (int i = 0; i < arc / step_size_; ++i) {
        // Bicycle model recursive x, y, phi
        // traveled_distance = v * dt
            // Simplified calculation of the position and orientation after the vehicle travels a certain distance
            double next_x = cur_x + traveled_distance * cos(cur_theta);
            double next_y = cur_y + traveled_distance * sin(cur_theta);
            double next_theta = cur_theta + traveled_distance * tan(steering1) / wheel_base_;
            // The formula is correct
            // cout << "turn radius:" << 1 / (tan(steering1) / wheel_base_) << endl;
            mod2Pi(next_theta);
            
            traversed_x.push_back(next_x);
            traversed_y.push_back(next_y);
            traversed_theta.push_back(next_theta);

            cur_x = next_x;
            cur_y = next_y;
            cur_theta = next_theta;
        }
        // cout << "xuanhuanwan" << endl;
        // cout << traversed_x.size() << endl;

        // Final node:
        // Return nullptr if it exceeds the boundary
        if (traversed_x.back() < x_min_ || traversed_x.back() > x_max_ ||
            traversed_y.back() < y_min_ || traversed_y.back() > y_max_) {
            // cout << "Exceeds boundary" << endl;
            return nullptr;
        }
        
        // If it does not exceed the boundary, treat it as the next_node
        shared_ptr<Node3d> next_node = make_shared<Node3d>(traversed_x, traversed_y, traversed_theta);
        next_node->setParent(cur_node);
        next_node->setDirection(traveled_distance > 0.0);
        // Forward or reverse
        next_node->setSteering(steering);
        // Front wheel steering angle
        // cout << "xyt:" << traversed_x.back() << " " << traversed_y.back() << " " << traversed_theta.back() << endl;
        return next_node;

    }


    // Calculate "g" in f = h + g: movement cost G
    // The node movement cost includes the cost of moving forward or backward between two nodes, 
    // the cost of the front wheel steering angle, and the cost of gear shifting if there is a gear change between two nodes:
    double HybridAstar::TrajCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
        double piecewise_cost = 0.0;

        // Cost of moving forward or backward
        // This code calculates the cost of the path based on the direction and distance of the next node, 
        // used to evaluate the quality of the path. Different directions and distances will adjust the path cost 
        // based on penalty coefficients to select a better path during path search.
        if (next_node->getDirection()) {
            // Forward
            // next_node->getStepSize() represents the number of steps from the current node to the next node, 
            // step_size_ represents the length of each step
            // (next_node->getStepSize() - 1) * step_size_ represents the distance from the current node to the next node
            // Note that the scale should match with g
            piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_forward_penalty_;
        } else {
            // Reverse
            piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_back_penalty_;
        }

        // Forward/reverse switching
        if (cur_node->getDirection() != next_node->getDirection()) {
            // Penalty for gear switching
            piecewise_cost += traj_gear_switch_penalty_;
        }
        // Cost of steering from the current node to the next node
        // This is done to encourage the path to maintain smooth steering, avoid frequent direction changes, 
        // and improve the stability and reliability of path planning.
        piecewise_cost += traj_steer_change_penalty_ * std::abs(
            next_node->getSteering() - cur_node->getSteering()
        );
        // Penalty for steering changes
        // To encourage the path to maintain smaller steering angles, i.e., reduce the turning amplitude, 
        // and improve the smoothness and comfort of path planning.
        piecewise_cost += traj_steer_penalty_ * std::abs(next_node->getSteering());

        return piecewise_cost;
        // Various penalties to be added
    }


    // Calculate the path cost and heuristic cost separately
    void HybridAstar::calculateNodeCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
        // Calculate f = h + g

        // The cost G of the trajectory traveled in Hybrid A*
        next_node->setTrajCost(cur_node->getTrajCost() + TrajCost(cur_node, next_node));

        // Holonomic heuristic with obstacles
        double holo_heu_cost = 0.0;
        // Calculate "h" in f = h + g, obtained through DP_Map, which is essentially the Dijkstra method with the goal as the start
        // The heuristic cost H from the current point to the goal in A* is calculated using dynamic programming DP (with the goal as the starting point of DP)
        // holo_heu_cost = grid_search_ptr_->lookupInDpMap({next_node->getX(), next_node->getY()});
        // cout << "holo heu cost:" << holo_heu_cost << endl;

        // Query the approximate A* distance value
        holo_heu_cost = grid_search_ptr_->CheckLookUpTable({next_node->getX(), next_node->getY()});

        // Real-time A* distance
        // holo_heu_cost = grid_search_ptr_->calculateHeuByAstar({next_node->getX(), next_node->getY()}, {goal_node_->getX(), goal_node_->getY()});

        // Non-holonomic without obstacles (collision-free)
        /* This code is used to calculate the heuristic cost in non-holonomic constrained steering path planning. The following is an explanation of the code:
        First, a 3D Reeds-Shepp state space is created using ob::ReedsSheppStateSpace(3), which is used to simulate non-holonomic constrained steering path planning. Then, two SE2 state space state objects, rsStart and rsEnd, are allocated using space->allocState().
        Next, the coordinates and heading angle of the starting node are set using rsStart->setXY(next_node->getX(), next_node->getY()) and rsStart->setYaw(next_node->getTheta()). The coordinates and heading angle of the goal node are set using rsEnd->setXY(goal_node_->getX(), goal_node_->getY()) and rsEnd->setYaw(goal_node_->getTheta()).
        Then, the Reeds-Shepp distance from the starting node to the goal node is calculated using space->distance(rsStart, rsEnd), and the result is stored in the RS_heu_cost variable.
        Finally, the maximum value between the Reeds-Shepp distance and the previously calculated holonomic steering heuristic cost (holo_heu_cost) is selected as the heuristic cost of the current node and set as the heuristic cost of next_node.
        After calculating the heuristic cost, the code also outputs the time used for non-holonomic constrained steering path planning and the calculated Reeds-Shepp heuristic cost. */
        // auto non_holo_heu_start = chrono::system_clock::now();

        // ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>(3));
        // ob::SE2StateSpace::StateType * rsStart = (ob::SE2StateSpace::StateType*)space->allocState();
        // ob::SE2StateSpace::StateType* rsEnd = (ob::SE2StateSpace::StateType*)space->allocState();
        // rsStart->setXY(next_node->getX(), next_node->getY());
        // rsStart->setYaw(next_node->getTheta());
        // rsEnd->setXY(goal_node_->getX(), goal_node_->getY());
        // rsEnd->setYaw(goal_node_->getTheta());
        // double RS_heu_cost = space->distance(rsStart, rsEnd);
        // auto non_holo_heu_end = chrono::system_clock::now();
        // chrono::duration<double> non_holo_use_time = non_holo_heu_end - non_holo_heu_start;
        // cout << "nonholo use time:" << non_holo_use_time.count() * 1000 << "ms" << endl;
        // cout << "RS heu cost: " << RS_heu_cost << endl;
        // next_node->setHeuristicCost(max(RS_heu_cost, holo_heu_cost));

        double DB_heu_cost;
        try {
            // auto non_holo_heu_start = chrono::system_clock::now();
            ob::StateSpacePtr space(std::make_shared<ob::DubinsStateSpace>(dubins_radius_)); // Curvature radius
            ob::SE2StateSpace::StateType * dbStart = (ob::SE2StateSpace::StateType*)space->allocState();
            ob::SE2StateSpace::StateType* dbEnd = (ob::SE2StateSpace::StateType*)space->allocState();
            dbStart->setXY(next_node->getX(), next_node->getY());
            dbStart->setYaw(next_node->getTheta());
            dbEnd->setXY(goal_node_->getX(), goal_node_->getY());
            dbEnd->setYaw(goal_node_->getTheta());
            DB_heu_cost = space->distance(dbStart, dbEnd);
            // auto non_holo_heu_end = chrono::system_clock::now();
            // chrono::duration<double> non_holo_use_time = non_holo_heu_end - non_holo_heu_start;
            // cout << "nonholo use time:" << non_holo_use_time.count() * 1000 << "ms" << endl;
            // cout << "DB heu cost: " << DB_heu_cost << endl;

        } catch (const std::exception& e) {
            // Code to handle exceptions
            std::cerr << "An exception occurred: " << e.what() << std::endl;
            ROS_ERROR("An exception occurred.");
        }

        // std::shared_ptr<DubinsPath_> dubins_to_check = std::make_shared<DubinsPath_>();
        // dubins_generator_->ShortestDBP_(cur_node, goal_node_, *dubins_to_check);
        // double DB_heu_cost = dubins_to_check->total_length;

        // Set the H value
        next_node->setHeuristicCost(max(DB_heu_cost, holo_heu_cost));
    }


    // Generate the four vertices of the car at the current waypoint
    vector<Vec2d> HybridAstar::calculateCarBoundingBox(Vec3d pose) {
        vector<Vec2d> vertices;
        double shift_distance = length_ / 2 - back_edge_to_center_;
        // Offset from the rear axle center to the car center
        // Car center
        Vec2d center = {pose(0) + shift_distance * std::cos(pose(2)),
                        // Pose point is at the rear axle center
                        pose(1) + shift_distance * std::sin(pose(2))};
        // Set an inflation value to enlarge the outline (conservative)
        const double dx1 = std::cos(pose(2)) * (length_ / 2 + inflation_value );
        const double dy1 = std::sin(pose(2)) * (length_ / 2 + inflation_value);
        const double dx2 = std::sin(pose(2)) * (width_ / 2 + inflation_value);
        const double dy2 = -std::cos(pose(2)) * (width_ / 2 + inflation_value);

        // const double dx1 = std::cos(pose(2)) * length_ / 2 ;
        // const double dy1 = std::sin(pose(2)) * length_ / 2 ;
        // const double dx2 = std::sin(pose(2)) * width_ / 2 ;
        // const double dy2 = -std::cos(pose(2)) * width_ / 2 ;

        // Four vertices
        vertices.emplace_back(center(0) + dx1 + dx2, center(1) + dy1 + dy2);
        vertices.emplace_back(center(0) + dx1 - dx2, center(1) + dy1 - dy2);
        vertices.emplace_back(center(0) - dx1 - dx2, center(1) - dy1 - dy2);
        vertices.emplace_back(center(0) - dx1 + dx2, center(1) - dy1 + dy2); 
        // Clockwise order
        return vertices;
    }


    /**
    * @brief Check if a point on a line is in an obstacle (interpolation)
    * 
    * @param x0 
    * @param y0 
    * @param x1 
    * @param y1 
    * @return true 
    * @return false 
    */
    bool HybridAstar::isLinecollision(double x0, double y0, double x1, double y1) {
        // int check_point_num = static_cast<int>(max(abs(x1 - x0),abs(y1 - y0)) / xy_resolution_) + 1;
        // int check_point_num = static_cast<int>(max(abs(x1 - x0),abs(y1 - y0)) / 1) + 1;
        
        auto check_point_num = static_cast<int>(std::hypot(x1 - x0, y1 - y0) / xy_resolution_);

        double delta_x = (x1 - x0) / check_point_num;
        double delta_y = (y1 - y0) / check_point_num;

        double cur_x = x0;
        double cur_y = y0;

        // Check if it is an obstacle
        // auto idx = i + j * map_.info.width;: This line calculates the index idx in the map data array for the given coordinates. Since map data is typically stored as a one-dimensional array, the 2D coordinates need to be converted to a 1D index. The specific calculation method is to multiply j by the map width and then add i.
        // return map_.data[idx] == 100;: Finally, the function returns a boolean value indicating whether the given coordinates are an obstacle. If the value in the map data array at idx is 100, it is considered an obstacle, and the function returns true; otherwise, it returns false.
        auto mapObstacle = [&](double x, double y) {
            auto i = std::floor((x - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
            auto j = std::floor((y - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
            auto idx = i + j * grid_->info.width;
            return grid_->data[idx] == 100 || grid_->data[idx] == -1;
            // 100 is the obstacle pixel value, -1 is unknown
        };

        for (int i = 0; i < check_point_num; ++i) {
            if (!isInMap(cur_x, cur_y)) {
                return true;
            }

            // Vec3i idx = getIndexFromPose({cur_x, cur_y, 0});
            // Both obstacle and unknown areas are impassable
            if(mapObstacle(cur_x, cur_y))  return true;
            // if (map_.atPosition("elevation", {cur_x, cur_y}) > 0) {
            //     return true;
            // }

            cur_x += delta_x;
            cur_y += delta_y;
        }
        return false;
    }


    /**
    * @brief Verify if the node is feasible. It requires obtaining the car's outline and then using the above method to check if the four edges pass through obstacles.
    * 
    * @param node 
    * @return true 
    * @return false 
    */
    bool HybridAstar::validityCheck(std::shared_ptr<Node3d> node) {
        
        int node_step_size = node->getStepSize(); // Represents how many waypoints are in a grid
        // cout << "Number of points in the node: " << node_step_size << endl;
        // Sub-waypoints
        const auto traversed_x = node->getXs();
        const auto traversed_y = node->getYs();
        const auto traversed_theta = node->getThetas();

        int start_index = 0;
        if (node_step_size > 1) {  
            // More than one node, there are intermediate nodes
            start_index = 1;   
            // The first one is the x, y, theta of the previous node
        }

        // Traverse the points in the node collection
        for (int i = start_index; i < node_step_size; ++i) {
            // If any point is outside the map boundary, discard the node
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
                // Indicates a collision

        }
        return true;
    }

    Vec3i HybridAstar::getIndexFromPose(Vec3d pose) {
        Vec3i index;
        index(0) = std::floor((pose(0) - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
        index(1) = std::floor((pose(1) - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
        // Ensure the angle remains within [-pi, +pi]
        mod2Pi(pose(2));
        index(2) = static_cast<int>((pose(2)) / theta_resolution_);
        return index;
    }



    /**
    * @brief Directly connect to the goal using a Dubins (DB) curve. Attempt to connect the current point to the goal point using a DB curve.
    * 
    * @param current_node 
    * @return true 
    * @return false 
    */
    bool HybridAstar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
        // std::shared_ptr<ReedSheppPath> reeds_shepp_to_check = std::make_shared<ReedSheppPath>();
        // // ReedShepp curves always connect from the current point to the goal
        // if (!reed_shepp_generator_->ShortestRSP(current_node, goal_node_,
        //                                         *reeds_shepp_to_check)) {
        //     return false;
        // }
        // std::cout << "ShortestDBP found";
        
        std::shared_ptr<DubinsPath_> dubins_to_check = std::make_shared<DubinsPath_>();
        // Generate the shortest Dubins curve
        dubins_generator_->ShortestDBP(current_node, goal_node_, *dubins_to_check);

        // std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(reeds_shepp_to_check->x, reeds_shepp_to_check->y, reeds_shepp_to_check->phi));

        std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
        dubins_to_check->x, dubins_to_check->y, dubins_to_check->phi));

        // Perform collision detection on the node generated by the DB curve. If a collision occurs, this path is not suitable.
        if (!validityCheck(node)) {
            // Discard if there is a collision
            return false;
        }
        // Package the waypoints of the ReedShepp curve connecting to the goal into a node and add it to the Hybrid A* collection.
        // std::shared_ptr<Node3d> goal_node = std::shared_ptr<Node3d>(new Node3d(
        // reeds_shepp_to_check->x, reeds_shepp_to_check->y, reeds_shepp_to_check->phi));
        std::shared_ptr<Node3d> goal_node = std::shared_ptr<Node3d>(new Node3d(
            dubins_to_check->x, dubins_to_check->y, dubins_to_check->phi));
            // These three should be of type double
        db_length = dubins_to_check->total_length;
        goal_node->setParent(current_node);

        // Vec3i index111 = getIndexFromPose({goal_node->getX(), goal_node->getY(), goal_node->getTheta()});
        // goal_node->setIndex(index111, map_size_x_, map_size_y_);

        close_set_.emplace(goal_node->getIndex(), goal_node);
        final_node_ = goal_node;
        return true;
    }


    inline void HybridAstar::mod2Pi(double &angle) {
        if (angle >  M_PI)  angle -= 2 * M_PI;
        if (angle < -M_PI)  angle += 2 * M_PI;
    }


    // Check if the given pose is outside the map boundaries
    inline bool HybridAstar::isInMap(double x, double y) {

        auto i = std::floor((x - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
        auto j = std::floor((y - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
        return (i >= 0 && j >= 0 && i < grid_->info.width && j < grid_->info.height);

        // if (x < x_min_ || x > x_max_ ||
        //     y < y_min_ || y > y_max_) {
        //     return false;
        // }
        
        // return true;
    }

} // namespace planning

