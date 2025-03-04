#include "hybrid_astar_searcher/calculate_heuristic.h"

using namespace std;


namespace planning {

GridSearch::GridSearch(const nav_msgs::OccupancyGridConstPtr& map, CollisionDetection& configurationSpace) :
    map_(map),
    cloud(new pcl::PointCloud<pcl::PointXY>){
    resolution_ = map->info.resolution;
    x_min_ = map->info.origin.position.x;
    x_max_ = x_min_ + map->info.width * resolution_;
    y_min_ = map->info.origin.position.y;
    y_max_ = y_min_ + map->info.height * resolution_;
    map_size_x_ = map->info.width; 
    map_size_y_ = map->info.height;
    configurationSpace_ptr_ = std::make_unique<CollisionDetection>(configurationSpace);
    // Visualize the curve waypoints before optimization
    ros::NodeHandle nh("/");
    path_point_vis_pub = nh.advertise<visualization_msgs::Marker>("db_path_point_marker", 10);
}

std::vector<Point2D> GridSearch::calculateHeuByAstar(Vec3d& start_pos, Vec3d& goal_pos) {
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;
    unordered_map<int, shared_ptr<Node2d>> close_set;

    Vec2i start_idx = getIndexFromPosition(Vec2d{start_pos(0), start_pos(1)});
    Vec2i goal_idx = getIndexFromPosition(Vec2d{goal_pos(0), goal_pos(1)});

    shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_);
    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_pq.emplace(start_node->getIndex(), start_node->getF());
    open_set.emplace(start_node->getIndex(), start_node);

    int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;
        // Start from the initial node, each time select the node with the lowest F cost
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        if (cur_node->getIndex() == goal_node->getIndex()) {
            final_node_ = cur_node;
            break;
        }
        
        close_set.emplace(cur_index, cur_node);
        // Eight-neighborhood search
        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        for (auto &neighbor_node : neighbor_nodes) {
            if (!isInGridMap(neighbor_node)) {
                continue;
            }
            if (isOnObstacle(neighbor_node)) {
                continue;
            }
            // Occupancy grid template - line collision detection
            if(!configurationSpace_ptr_->isTraversable(neighbor_node->getPosXInMap(), neighbor_node->getPosYInMap(), neighbor_node->getDIR())){
                continue;
            }
            if (close_set.find(neighbor_node->getIndex()) != close_set.end()) {
                continue;
            }
            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
                ++explored_node_num;
                // Euclidean distance from this node to the goal: H value
                double heuristic = EuclidDistance(neighbor_node->getPosXInMap(), neighbor_node->getPosYInMap(),
                                                goal_node->getPosXInMap(), goal_node->getPosYInMap());
                neighbor_node->setH(heuristic);
                // H value
                neighbor_node->setParent(cur_node);
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);
                // Add to open_set
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getF());
                // Store in open_pq ordered by F value
            } else {
                // Update G value: the distance traveled
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setDIR(neighbor_node->getDIR());
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }
        }
    }

    // A path has been found, extract the turning points
    std::vector<Point2D> path = {};

    if (final_node_ == nullptr) {
        cout << "No path found" << endl;
        return path;
    }

    // heu_astar_ = final_node_->getG() * resolution_;
    // cout << "Astar Heuristic:" << heu_astar_ << endl;
    cout << "Number of nodes explored: " << explored_node_num << endl;

    shared_ptr<Node2d> cur_node = final_node_;
    double yaw0 = cur_node->getDIR();
    auto dyaw = goal_pos(2) - yaw0;
    mod2Pi(dyaw);
    if (std::abs(dyaw) > M_PI_2) goal_pos(2) = goal_pos(2) + M_PI;
    // goal_pos(2) = (std::abs(goal_pos(2) - yaw0) > M_PI_2 ? goal_pos(2) + M_PI : goal_pos(2));
    mod2Pi(goal_pos(2));

    // Add the goal point to the path
    Point2D pose = {cur_node->getPosXInMap(), cur_node->getPosYInMap()};
    // Row and column indices
    path.push_back(pose);
    // Core while loop, backtracking process
    while (cur_node->getParentNode() != nullptr) {
        if (yaw0 != cur_node->getDIR()) {
            yaw0 = cur_node->getDIR();
            pose.x = cur_node->getPosXInMap();
            pose.y = cur_node->getPosYInMap();
            path.push_back(pose);
        }
        cur_node = cur_node->getParentNode();
    }
    pose.x = cur_node->getPosXInMap();
    pose.y = cur_node->getPosYInMap();
    path.push_back(pose);
    // Reverse to get a path consisting only of turning points
    reverse(path.begin(), path.end());

    // Check if the start orientation and the A* expansion orientation are in the same half-plane
    dyaw = start_pos(2) - yaw0;
    mod2Pi(dyaw);
    start_pos(2) = (std::abs(dyaw) > M_PI_2 ? start_pos(2) + M_PI : start_pos(2));
    mod2Pi(start_pos(2));

    // Visualize all A* waypoints
    // int id = 0; // Marker ID
    // visualization_msgs::Marker path_point;
    // auto point_marker = [&](std::vector<Point2D> const& points) {
    //     path_point.id = id++;
    //     path_point.header.frame_id = "map";
    //     path_point.header.stamp = ros::Time::now();
    //     path_point.type = visualization_msgs::Marker::POINTS;
    //     path_point.scale.x = 0.1;
    //     path_point.scale.y = 0.1;
    //     path_point.scale.z = 0.1;
    //     path_point.color.a = 1;
    //     path_point.color.r = 1;
    //     path_point.color.g = 0;
    //     path_point.color.b = 0;
    //     path_point.pose.orientation.w = 1.0;
    //     for (size_t i = 0; i < points.size(); i++) {
    //         geometry_msgs::Point p;
    //         p.x = (points[i].x + 0.5) * resolution_ + x_min_;
    //         p.y = (points[i].y + 0.5) * resolution_ + y_min_;
    //         // p.z = 0.05;
    //         path_point.points.push_back(p);
    //     }
    // };
    // point_marker(path);
    // path_point_vis_pub.publish(path_point);

    return path;
}

inline void GridSearch::mod2Pi(double &angle) {
    if (angle > M_PI)  angle -= 2 *M_PI;
    if (angle < - M_PI)  angle += 2 * M_PI;
}

 double GridSearch::calculateHeuByAstar(Vec2d start_pos, Vec2d goal_pos) {
    // cout << "A star search" << endl;
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;
    unordered_map<int, shared_ptr<Node2d>> close_set;

    Vec2i start_idx = getIndexFromPosition(start_pos);
    Vec2i goal_idx = getIndexFromPosition(goal_pos);
    // cout << "start_idx:" << start_idx << endl;

    shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_);
    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_pq.emplace(start_node->getIndex(), start_node->getF());
    open_set.emplace(start_node->getIndex(), start_node);

    // cout << "About to enter the loop" << endl;
    int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        if (cur_node->getIndex() == goal_node->getIndex()) {
            final_node_ = cur_node;
            break;
        }
        
        close_set.emplace(cur_index, cur_node);
        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        // cout << "Got neighbor nodes" << endl;
        for (auto &neighbor_node : neighbor_nodes) {
            if (!isInGridMap(neighbor_node)) {
                continue;
            }
            if (isOnObstacle(neighbor_node)) {
                continue;
            }
            if (close_set.find(neighbor_node->getIndex()) != close_set.end()) {
                continue;
            }

            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
                ++explored_node_num;
                double heuristic = 
                EuclidDistance(neighbor_node->getPosXInMap(), neighbor_node->getPosYInMap(),
                               goal_node->getPosXInMap(), goal_node->getPosYInMap());
                               // Euclidean distance
                neighbor_node->setH(heuristic);
                neighbor_node->setParent(cur_node);
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getF());
            } else {
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }
        }
    }
    if (final_node_ == nullptr) {
        cout << "No path found" << endl;
        return false;
    }
    // double heu_astar_ = final_node_->getG() * resolution_;
    // cout << "Astar Heuristic:" << heu_astar_ << endl;
    explored_node_num_ += explored_node_num;
    num++;
    cout << "Number of nodes explored: " << (int)(explored_node_num_ / num) << endl;
    return final_node_->getG() * resolution_;
}

void GridSearch::clear(){
    explored_node_num_ = 0;
    num = 0;
}

/**
 * @brief First generate a distance table to the goal, so that each expansion can directly look up the table. However, building this table also takes a lot of time.
 * Generate dp_map_ for table lookup to avoid online heuristic calculation and speed up the solving process.
 * @param goal_pos 
 * @return true 
 * @return false 
 */
bool GridSearch::generateDpMap(Vec2d goal_pos, Vec2d start_pos) {
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;

    dp_map_.clear();

    Vec2i goal_idx = getIndexFromPosition(goal_pos);
    Vec2i start_idx = getIndexFromPosition(start_pos);
    // Generate a limited zone for building the distance table, otherwise planning is too slow
    VecXd limit_zone(4);
    limit_zone(0) = std::min(goal_idx(0), start_idx(0)) - 100;
    limit_zone(1) = std::max(goal_idx(0), start_idx(0)) + 100;
    limit_zone(2) = std::min(goal_idx(1), start_idx(1)) - 100;
    limit_zone(3) = std::max(goal_idx(1), start_idx(1)) + 100;
    
    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_set.emplace(goal_node->getIndex(), goal_node);
    open_pq.emplace(goal_node->getIndex(), goal_node->getG());

    int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        dp_map_.emplace(cur_index, cur_node);
        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        // cout << "Got neighbor nodes" << endl;
        // Use breadth-first search to record the shortest traversal distance from each node to the goal node
        // neighbor_nodes stores the neighboring nodes around the current node
        for (auto &neighbor_node : neighbor_nodes) {
            if (!isInGridMap(neighbor_node)) {
                continue;
            }
            if (!isInLimitedMap(neighbor_node, limit_zone)) {
                continue;
            }            
            if (isOnObstacle(neighbor_node)) {
                continue;
            }
            if (dp_map_.find(neighbor_node->getIndex()) != dp_map_.end()) {
                continue;
            }
            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
                ++explored_node_num;
                neighbor_node->setParent(cur_node);
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getG());
            } else {
                // Update G value: the distance traveled, since we are moving from the goal
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }
        }
    }
    cout << "Number of nodes searched: " << explored_node_num << endl; 
    return true;
}

bool GridSearch::generateLocalDpMap(Vec2d goal_pos, Vec2d start_pos) {
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;

    dp_map_.clear();

    Vec2i goal_idx = getIndexFromPosition(goal_pos);
    Vec2i start_idx = getIndexFromPosition(start_pos);
    VecXd limit_zone(4);
    limit_zone(0) = std::min(goal_idx(0), start_idx(0)) - 40;
    limit_zone(1) = std::max(goal_idx(0), start_idx(0)) + 40;
    limit_zone(2) = std::min(goal_idx(1), start_idx(1)) - 40;
    limit_zone(3) = std::max(goal_idx(1), start_idx(1)) + 40;
    
    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_set.emplace(goal_node->getIndex(), goal_node);
    open_pq.emplace(goal_node->getIndex(), goal_node->getG());

    // int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        dp_map_.emplace(cur_index, cur_node);
        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        for (auto &neighbor_node : neighbor_nodes) {
            int index_x = neighbor_node->getPosXInMap();
            int index_y = neighbor_node->getPosYInMap();

            if (index_x < 0 || index_x >= map_size_x_ || index_y < 0 || index_y >= map_size_y_
                // 100 is the grid value for obstacles, -1 is the grid value for unknown areas
                || map_->data[neighbor_node->getIndex()] == 100 || map_->data[neighbor_node->getIndex()] == -1) {
                continue;
            }
            if (!isInLimitedMap(neighbor_node, limit_zone)) {
                continue;
            }            
            // Check if the key corresponding to neighbor_node->getIndex() already exists in dp_map_
            if (dp_map_.find(neighbor_node->getIndex()) != dp_map_.end()) {
                continue;
            }
            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
                // ++explored_node_num;
                neighbor_node->setParent(cur_node);
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getG());
            } else {
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }

        }

    }

    // cout << "Number of nodes searched: " << explored_node_num << endl; 
    return true;
}

// Check the distance to the destination.
double GridSearch::lookupInDpMap(Vec2d start_pos) {
    Vec2i start_idx = getIndexFromPosition(start_pos);
    shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_);

    if (dp_map_.find(start_node->getIndex()) != dp_map_.end()) {
        return dp_map_[start_node->getIndex()]->getG() * resolution_;
    } else {
        return numeric_limits<double>::infinity();
    }
}

vector<shared_ptr<Node2d>> GridSearch::getNeighborNodes(shared_ptr<Node2d> cur_node) {
    int cur_node_x = cur_node->getPosXInMap();
    int cur_node_y = cur_node->getPosYInMap();
    int cur_node_g = cur_node->getG();
    // Diagonal distance of the grid, grid length
    double diagonal_distance = sqrt(2);
    vector<shared_ptr<Node2d>> neighbors;
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            Vec2i neighbor_idx{cur_node_x + i, cur_node_y + j};
            shared_ptr<Node2d> neighbor = make_shared<Node2d>(neighbor_idx, map_size_x_);
            if (i == 0 && j == 0) continue;
            // Based on i and j, determine the orientation of the current expanded node (relative to cur_node). The orientation is globally unified, so there are only 8 possible directions.
            if (sqrt(i * i + j * j) > 1) {
                neighbor->setG(cur_node_g + diagonal_distance);
                // Diagonal
            } else {
                neighbor->setG(cur_node_g + 1);
            }
            // Assign the orientation, used to determine nodes with different orientations as turning points in the polyline
            std::pair<int, int> key = {i, j};
            // Find and retrieve the value
            auto iter = directionAngles.find(key);
            if (iter != directionAngles.end()) {
                neighbor->setDIR(iter->second);
            }
            neighbors.emplace_back(neighbor);
        }
    }
    return neighbors;
}

inline bool GridSearch::isInGridMap(shared_ptr<Node2d> node) {
    int index_x = node->getPosXInMap();
    int index_y = node->getPosYInMap();

    if (index_x < 0 || index_x >= map_size_x_ || index_y < 0 || index_y >= map_size_y_) {
        return false;
    }

    return true;
}

inline bool GridSearch::isInLimitedMap(shared_ptr<Node2d> node, VecXd lzone) {
    int index_x = node->getPosXInMap();
    int index_y = node->getPosYInMap();

    if (index_x < lzone(0) || index_x > lzone(1) || index_y < lzone(2) || index_y > lzone(3)) {
        return false;
    }

    return true;
}

inline bool GridSearch::isOnObstacle(shared_ptr<Node2d> node) {
    return (map_->data[node->getIndex()] == 100 || map_->data[node->getIndex()] == -1);
}

// Backtrack the entire path from the end point
 vector<Vec2d> GridSearch::getAstartPath() {
    shared_ptr<Node2d> cur_node = final_node_;
    vector<shared_ptr<Node2d>> vec;
    vector<Vec2d> res;

    while (cur_node->getParentNode() != nullptr) {
        vec.emplace_back(cur_node);
        cur_node = cur_node->getParentNode();
    }
    reverse(vec.begin(), vec.end());
    // cout << "vec size: " << vec.size() << endl;

    for (auto &node : vec) {
        res.push_back({node->getPosXInMap() * resolution_ + x_min_,
                       node->getPosYInMap() * resolution_ + y_min_ });
        // cout << "cur_node->getPosXInMap(): " << node->getPosXInMap() << endl;
    }
    return res;
}

inline double GridSearch::EuclidDistance(const double x1, const double y1,
                                  const double x2, const double y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

inline Vec2i GridSearch::getIndexFromPosition(Vec2d position) {
    Vec2i index;
    index(0) = std::floor((position(0) - x_min_) / resolution_ + 0.5);
    index(1) = std::floor((position(1) - y_min_) / resolution_ + 0.5);
    return index;
}

inline Vec2d GridSearch::getPositionFromIndex(Vec2i index) {
    Vec2d pos;
    pos(0) = x_min_ + (index(0) + 0.5) * resolution_;
    pos(1) = y_min_ + (index(1) + 0.5) * resolution_;
    return pos;
}

std::vector<Point2D> GridSearch::PathShorten(std::vector<Point2D> const& path){
    auto TaceSegSparse = [&](Point2D point_one, Point2D point_two) -> std::vector<Point2D> {
        std::vector<Point2D> path_sparse;
        path_sparse.push_back(point_one);
        const float k_seg_len = 1.0 / resolution_; 
        float dis = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
        int seg_num = dis / k_seg_len + 0.5;
        if (seg_num < 2) {
            return path_sparse;
        }
        // Use linear interpolation to generate new path points between each small segment.
        float delta_x = (point_two.x - point_one.x) / (1.f * seg_num);
        float delta_y = (point_two.y - point_one.y) / (1.f * seg_num);
        for (int i = 1; i < seg_num; i++) {
            // + 0.5f: To round the calculated point coordinates from real space to the nearest integer coordinates, ensuring the points are more accurately placed on the line.
            Point2D seg_point{static_cast<int>(point_one.x + delta_x * i + 0.5f), static_cast<int>(point_one.y + delta_y * i + 0.5f)};
            path_sparse.push_back(seg_point);
        }
        return path_sparse;
    };

    auto TacePathSparse = [=](std::vector<Point2D> basePoints) -> std::vector<Point2D> {
        if (basePoints.size() < 3) { 
            return basePoints;
        }
        std::vector<Point2D> ret;
        for (unsigned int i = 0; i < basePoints.size() - 1; i++) {
            std::vector<Point2D> temp = TaceSegSparse(basePoints[i], basePoints[i + 1]);
            ret.insert(ret.end(), temp.begin(), temp.end());
        }
        ret.emplace_back(basePoints.back());
        return ret;
    };

    // Optimization: Straighten and shorten the path.
    auto path_points_sparse = TacePathSparse(path);
    // Return the interpolated A* path.
    std::vector<Point2D> path_adjusted;
    bool is_path_adjust_work = false;
    do {
        PathAdjust pp(path_points_sparse.size());
        if (path_points_sparse.size() < 3) {
            break;
        }
        // Use the line segments between adjacent path points as edges.
        for (size_t i = 0; i < path_points_sparse.size() - 1; i++) {
            auto point_one = path_points_sparse[i];
            auto point_two = path_points_sparse[i + 1];
            // Calculate the distance between adjacent points.
            float dis = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
            // Use the distance as the weight of the edge.
            pp.AddEdge(i, i + 1, dis);
        }
        
        for (size_t i = 0; i < path_points_sparse.size() - 2; i++) {
            // linehit
            int meetHit = 0;
            for (size_t j = i + 2; j < path_points_sparse.size(); j++) {
                Point2D p1{path_points_sparse[i].x, path_points_sparse[i].y};
                Point2D p2{path_points_sparse[j].x, path_points_sparse[j].y};
                if (!LineHit(p1, p2)) {
                    meetHit++;  
                    if (meetHit > 5) {
                        break;
                    } else {
                        continue;
                    }
                } else {
                    auto point_one = path_points_sparse[i];
                    auto point_two = path_points_sparse[j];
                    float dis = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
                    pp.AddEdge(i, j, dis);
                }
            }
        }
        std::vector<int> ret = pp.GetShortestPath();
        for (auto item : ret) {
            path_adjusted.push_back(path_points_sparse[item]);
        }
        if (path_adjusted.size() < path_points_sparse.size()) {
            // FLOGD("--PATH ADJUST WORKS.");
            is_path_adjust_work = true;
        }
    } while (0);
    
    if (is_path_adjust_work == true) {
        return path_adjusted;
    } else {
        return path;
    }
}

// Directly interpolate the sparse original A* path to obtain path points —————— for comparison experiments
std::vector<Point2D> GridSearch::PathInterpolation(std::vector<Point2D> const& path){
    auto TaceSegSparse = [&](Point2D point_one, Point2D point_two) -> std::vector<Point2D> {
        std::vector<Point2D> path_sparse;
        // Store the starting point.
        path_sparse.push_back(point_one);
        const float k_seg_len = 0.3 / resolution_; 
        // Each small segment has a length of 0.3m, corresponding to 3 pixels.

        float dis = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
        int seg_num = dis / k_seg_len + 0.5;
        if (seg_num < 2) {
            return path_sparse;
        }
        float delta_x = (point_two.x - point_one.x) / (1.f * seg_num);
        float delta_y = (point_two.y - point_one.y) / (1.f * seg_num);
        for (int i = 1; i < seg_num; i++) {
            Point2D seg_point{static_cast<int>(point_one.x + delta_x * i + 0.5f), static_cast<int>(point_one.y + delta_y * i + 0.5f)};
            path_sparse.push_back(seg_point);
        }
        return path_sparse;
    };

    auto TacePathSparse = [=](std::vector<Point2D> basePoints) -> std::vector<Point2D> {
        if (basePoints.size() < 3) {
            return basePoints;
        }
        std::vector<Point2D> ret;
        for (unsigned int i = 0; i < basePoints.size() - 1; i++) {
            std::vector<Point2D> temp = TaceSegSparse(basePoints[i], basePoints[i + 1]);
            // ret.emplace_back(basePoints[i]);
            ret.insert(ret.end(), temp.begin(), temp.end());
        }
        ret.emplace_back(basePoints.back());
        return ret;
    };

    return TacePathSparse(path);
}

bool GridSearch::LineHit(const Point2D pose1, const Point2D pose2){
  std::pair<double, double> p1;
  p1.first = (pose1.x + 0.5) * resolution_ + x_min_;
  p1.second = (pose1.y + 0.5) * resolution_ + y_min_;

  std::pair<double, double> p2;
  p2.first = (pose2.x + 0.5) * resolution_ + x_min_;
  p2.second = (pose2.y + 0.5) * resolution_ + y_min_;

  std::pair<double, double> ptmp = {};

  double dist = sqrt((p2.second - p1.second) * (p2.second - p1.second) +
                (p2.first - p1.first) * (p2.first - p1.first));
  if (dist < resolution_) return true;
  else {
    int value = int(floor(dist / resolution_)); // Here, dist is the real distance.
    double theta = atan2((p2.second - p1.second), (p2.first - p1.first));
    int n = 1;

    for (int i = 0; i < value; i++) {
      ptmp.first = p1.first + resolution_ * cos(theta) * n;
      ptmp.second = p1.second + resolution_ * sin(theta) * n;
      if (collision(ptmp.first, ptmp.second)) return false;
      n++;
    }
    return true;
  }
}

inline bool GridSearch::collision(const double x, const double y)
{
  unsigned int mx, my, mxy;
  mx = (int)((x - x_min_) / resolution_);
  my = (int)((y - y_min_) / resolution_);
  mxy = mx + my * map_size_x_;

  if (mx < 0 || mx >= map_size_x_ || my < 0 || my >= map_size_y_ || 
      map_->data[mxy] == 100 || map_->data[mxy] == -1) return true;

  return false;
}

inline double GridSearch::Distance(const Point2D& a, const Point2D& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void GridSearch::getPointsOnPolyline(const std::vector<Point2D>& polyline, float interval, std::vector<GridPoint>& points) {
    // First, calculate the total grid length of the shortest polyline path.
    double totalLength = 0;
    for (u_int i = 0; i < polyline.size() - 1; ++i) {
        totalLength += Distance(polyline[i], polyline[i + 1]);
    }
    // Take intervals based on the length.
    u_int numPoints = std::ceil(totalLength / interval);
    points.resize(numPoints);

    u_int pointIndex = 0;
    double lengthMoved = 0;
    // Starting from the beginning of the polyline, move along the polyline by the specified distance to get the position of each point.
    for (u_int i = 0; i < polyline.size() - 1 && pointIndex < numPoints; ++i) {
        double segmentLength = Distance(polyline[i], polyline[i + 1]);
        while (lengthMoved + interval <= segmentLength && pointIndex < numPoints) {
            double ratio = (lengthMoved + interval) / segmentLength;
            points[pointIndex].x = polyline[i].x + std::round(ratio * (polyline[i + 1].x - polyline[i].x));
            points[pointIndex].y = polyline[i].y + std::round(ratio * (polyline[i + 1].y - polyline[i].y));
            points[pointIndex].distance_to_goal = totalLength - (lengthMoved + interval);
            lengthMoved += interval;
            ++pointIndex;
        }
        totalLength -= segmentLength;
        lengthMoved -= segmentLength;
    }
}

void toPCLPointCloud(const std::vector<GridPoint>& points, pcl::PointCloud<pcl::PointXY>::Ptr cloud_) {
    for (const auto& point : points) {
        pcl::PointXY p = {static_cast<float>(point.x), static_cast<float>(point.y)};
        cloud_->push_back(p);
    }
}

void GridSearch::InitLookUpTable(Vec3d& goal_pos, Vec3d& start_pos){
    std::vector<Point2D> ret = calculateHeuByAstar(start_pos, goal_pos);

    const float k_seg_len = 0.3 / resolution_; 
    // Each small segment has a length of 0.3m (same as planning::step_size_), corresponding to 3 pixels.
    
    #if 1
    // The straightened A* shortest polyline path.
    std::vector<Point2D> polyline = PathShorten(ret);
    // Get points on the shortest polyline and their distances to the goal.
    astar_points.clear();
    getPointsOnPolyline(polyline, k_seg_len, astar_points);
    
    #else
    // Control experiment — directly use the original A* path as the reference path.
    getPointsOnPolyline(ret, k_seg_len, astar_points);
    // std::vector<Point2D> polyline = PathInterpolation(ret);
    #endif

    ROS_INFO("astar_points=%ld", astar_points.size());

    // Visualize astar_points.
    // int id = 0; // Marker ID.
    // visualization_msgs::Marker path_point;
    // auto point_marker = [&](std::vector<Point2D> const& points) {
    //     path_point.id = id++;
    //     path_point.header.frame_id = "map";
    //     path_point.header.stamp = ros::Time::now();
    //     path_point.type = visualization_msgs::Marker::POINTS;
    //     path_point.scale.x = 0.1;
    //     path_point.scale.y = 0.1;
    //     path_point.scale.z = 0.1;
    //     path_point.color.a = 1;
    //     path_point.color.r = 1;
    //     path_point.color.g = 0;
    //     path_point.color.b = 0;
    //     path_point.pose.orientation.w = 1.0;
    //     for (size_t i = 0; i < points.size(); i++) {
    //         geometry_msgs::Point p;
    //         p.x = (points[i].x + 0.5) * resolution_ + x_min_;
    //         p.y = (points[i].y + 0.5) * resolution_ + y_min_;
    //         // p.z = 0.05;
    //         path_point.points.push_back(p);
    //     }
    // };
    // point_marker(ret);
    // path_point_vis_pub.publish(path_point);

    // Store in a KD-tree.
    // Convert std::vector<GridPoint> to pcl::PointCloud<pcl::PointXY>.
    cloud->clear();
    toPCLPointCloud(astar_points, cloud);
    // Create KD-tree.
    kdtree.setInputCloud(cloud);
    cout << "Point cloud size: " << cloud->size() << endl;
    
    std::vector<geometry_msgs::PoseStamped> mret;
    std::vector<geometry_msgs::PoseStamped> mret_;
    for (auto const& idx : ret) {
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = (idx.x + 0.5) * resolution_ + x_min_;
        p.pose.position.y = (idx.y + 0.5) * resolution_ + y_min_;
        mret.push_back(p);
    }
    vis.publishPrimitiveAstar(mret);
    for (auto const& idx : polyline) {
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = (idx.x + 0.5) * resolution_ + x_min_;
        p.pose.position.y = (idx.y + 0.5) * resolution_ + y_min_;
        mret_.push_back(p);
    }
    vis.publishSimpleAstar(mret_);
}

double GridSearch::CheckLookUpTable(Vec2d start_pos) {
    Vec2i start_idx = getIndexFromPosition(start_pos);
    pcl::PointXY search_point = {static_cast<float>(start_idx(0)), static_cast<float>(start_idx(1))}; 
    // Search point
    int K = 1; 
    // Number of points to consider. Temporarily set to 1.
    std::unordered_set<int> excludedPoints;
    // Remove distant points.
    const float dis2 = 40. * 40.;

    while (true) {
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if (kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            for (int i = 0; i < pointIdxNKNSearch.size(); ++i) {
                if (excludedPoints.find(pointIdxNKNSearch[i]) == excludedPoints.end()) {
                    pcl::PointXY nearestPoint = kdtree.getInputCloud()->points[pointIdxNKNSearch[i]];

                    float distanceSquared = pointNKNSquaredDistance[i];
                    if (distanceSquared > dis2) {
                        // excludedPoints.insert(pointIdxNKNSearch[i]);
                        return numeric_limits<double>::infinity(); 
                    }

                    Point2D pose1 = {start_idx(0), start_idx(1)};
                    Point2D pose2 = {static_cast<int>(nearestPoint.x), static_cast<int>(nearestPoint.y)};
                    if (LineHit(pose1, pose2)) {
                        return (std::sqrt(distanceSquared) + astar_points[pointIdxNKNSearch[i]].distance_to_goal) * resolution_; // Return the real-world distance.
                    }
                    excludedPoints.insert(pointIdxNKNSearch[i]);
                }
            }
            K++;
            if (K > 7) return numeric_limits<double>::infinity();
        } else {
            throw std::runtime_error("No connectable point found");
            return numeric_limits<double>::infinity();
        }
    }
}

} // namespace planning

