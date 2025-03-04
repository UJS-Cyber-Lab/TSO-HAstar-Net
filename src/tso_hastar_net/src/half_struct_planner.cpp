#include "half_struct_planner.h"


namespace chibot::topologyplanning {

  HalfStructPlanner::HalfStructPlanner(const nav_msgs::OccupancyGridConstPtr& grid, ros::NodeHandle &nh):
    grid_(grid), 
    node_num_(0),
    graph_(nullptr),
    ContactS_paths(connection_point_num),
    ContactG_paths(connection_point_num),
    ContactS_paths_(connection_point_num),
    ContactG_paths_(connection_point_num),
    ContactSG_paths(connection_point_num),
    graph_bk_(nullptr) {
    hybrid_astar_ptr.reset(new HybridAstar(grid, nh));

    // Create the voronoi object and initialize it with the map.
    int size_x = grid_->info.width;
    int size_y = grid_->info.height;
    bool **bin_map = NULL;
    bin_map = new bool *[size_x];
    for (int i = 0; i < size_x; ++i) {
      bin_map[i] = new bool[size_y];
    }
    for (int j = size_y - 1; j >= 0; j--) {
      for (int i = 0; i < size_x; i++) {
        auto index_ = j * grid_->info.width + i;
        // Both obstacle grids and unknown grids are treated as obstacle grids.
        if (grid_->data[index_] == 100 || grid_->data[index_] == -1) {
          bin_map[i][j] = true;
        } else {
          bin_map[i][j] = false;
        }
      }
    }
    voronoiDiagram.initializeMap(size_x, size_y, bin_map);
    voronoiDiagram.update();
    voronoiDiagram.prune();

    // voronoiDiagram.visualize("../result.pgm");   
    // cout << "Voronoi diagram visualization" << std::endl;
    // ROS_INFO("voronoi");
    // voronoi.header.frame_id = "map";
    // voronoi.header.stamp = ros::Time();
    // voronoi.ns = "voronoi";
    // voronoi.id = 0;
    // voronoi.type = visualization_msgs::Marker::SPHERE_LIST;
    // voronoi.action = visualization_msgs::Marker::ADD;
    // voronoi.color.b = 1.0;
    // voronoi.color.a = 1.0;
    // voronoi.scale.x = 0.1;
    // voronoi.scale.y = 0.1;
    // voronoi.scale.z = 0.1;
    // voronoi.pose.orientation.w = 1.0;
    // geometry_msgs::Point p;
    // for (int j = size_y - 1; j >= 0; --j) {
    //     for (int i = 0; i < size_x; ++i) {
    //         if (voronoiDiagram.isVoronoi(i, j)) {
    //             Vec2d pos;
    //             pos(0) = grid_.info.origin.position.x + (i + 0.5) * grid_.info.resolution;
    //             pos(1) = grid_.info.origin.position.y + (j + 0.5) * grid_.info.resolution;
    //             p.x = pos(0);
    //             p.y = pos(1);
    //             p.z = 0.05;
    //             voronoi.points.push_back(p);
    //         }
    //     }
    // }
  }

  HalfStructPlanner::~HalfStructPlanner() {
    if (node_num_ == 0) return;
    for (auto i = 0; i < node_num_; ++i) {
      delete [] graph_[i];
      delete [] graph_bk_[i];
    }
    delete [] graph_;
    delete [] graph_bk_;
  };

  void HalfStructPlanner::init(ros::NodeHandle &nh) {
    nh.getParam("/hybrid_astar_dubins_opt/Net/connection_point_num", connection_point_num);
    nh.getParam("/hybrid_astar_dubins_opt/Net/connection_points_range", connection_points_range);
    nh.getParam("/hybrid_astar_dubins_opt/Net/step_size", step_size);
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/traffic_route", 1);
    res_pub_ = nh.advertise<nav_msgs::Path>("/traffic_route_points", 1);
    // Visualize the curve before optimization, the route planned by Hybrid A*
    path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/db_path_marker", 10);
    path_point_vis_pub = nh.advertise<visualization_msgs::Marker>("/db_path_point_marker", 10); // Visualize the waypoints of the curve before optimization
    optimized_path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/path", 10); // Visualize the smoothed path
    smoother_ptr.reset(new Smoother()); // Initialize the smoother
    // voronoi_pub = nh.advertise<visualization_msgs::Marker>("voronoi", 10); // Visualize the Voronoi diagram
    SG_path_vis_pub = nh.advertise<geometry_msgs::PoseArray>("/SG_path_marker", 100);
    topology_path_vis_pub = nh.advertise<geometry_msgs::PoseArray>("/topology_path_marker", 100);
    // A complete alternative topological route, whether to choose it depends on its length compared to the direct connection
    full_path_vis_pub = nh.advertise<geometry_msgs::PoseArray>("/full_path_marker", 100);
  }

  void HalfStructPlanner::set_traffic_route(lines_type const& lines) {
    traffic_routes_ = lines; // Contains the coordinates of the start and end points, information about which edges this edge can connect to, etc. A traffic_route represents an edge.
    std::string root_path;
    ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
    auto name = ss.str();
    std::replace(name.begin(), name.end(), ':', '_');
    auto dir = root_path + "/path/";
    auto file_path_ = dir + name;

    vector<vector<Vec3d>> astarpath; // Unoptimized path set.
    vector<vector<Vec3d>> astarpath_; // Optimized path set.
    for (const auto& traffic_route_ : traffic_routes_) {
      Vec3d start_pose, goal_pose;
      start_pose[0] = traffic_route_.a.x;
      start_pose[1] = traffic_route_.a.y;
      start_pose[2] = traffic_route_.a.theta;
      goal_pose[0] = traffic_route_.b.x;
      goal_pose[1] = traffic_route_.b.y;
      goal_pose[2] = traffic_route_.b.theta;
      result.theta.clear();
      result.x.clear();
      result.y.clear();
      if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
        vector<Vec3d> path2smooth;
        path2smooth.clear();
        for (int i = 0; i < result.x.size(); ++i) {
          path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});
        }
        astarpath.push_back(path2smooth);
        smoother_ptr->optimize(voronoiDiagram, path2smooth, grid_); // L(Limited-memory)-bfgs smoothing
        vector<Vec3d> smooth_path;
        smooth_path.clear();
        smoother_ptr->getSmoothPath(smooth_path);
        write_file(file_path_, smooth_path, result.total_length); // Write the optimized paths to the file.
        astarpath_.push_back(smooth_path);

        // for (size_t i = 0; i < result.x.size(); ++i)
        // {
        //     vis.publishVehicleBoxes({smooth_path[i](0), smooth_path[i](1), smooth_path[i](2)}, i); // Vehicle outline based on the smoothed path points
        //     vis.publishVehicleBoxes({result.x[i], result.y[i], result.theta[i]}, i); // Vehicle outline based on the unoptimized path points
        // }
      } else {
        ROS_ERROR("search fail");
        return;
      }
    }
    // visPath(astarpath); // Visualize the unoptimized route
    visOptPath(astarpath_); // Visualize the optimized paths
  }

  void HalfStructPlanner::set_traffic_route_topology(lines_type const& lines) {
    traffic_routes_ = lines;
    std::string root_path;
    ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
    auto name = ss.str();
    std::replace(name.begin(), name.end(), ':', '_');
    auto dir = root_path + "/topology_path/";
    auto file_path_ = dir + name;

    vector<vector<Vec3d>> astarpath;
    vector<vector<Vec3d>> astarpath_;
    for (const auto& traffic_route_ : traffic_routes_) {
      Vec3d start_pose, goal_pose;
      goal_pose[0] = traffic_route_.b.x;
      goal_pose[1] = traffic_route_.b.y;
      goal_pose[2] = traffic_route_.b.theta;
      if (traffic_route_.go_list.empty()) continue;
      for (auto const& line : traffic_route_.go_list) {
        if (line >= traffic_routes_.size()) continue;
        start_pose[0] = traffic_routes_[line].a.x;
        start_pose[1] = traffic_routes_[line].a.y;
        start_pose[2] = traffic_routes_[line].a.theta;
        result.theta.clear();
        result.x.clear();
        result.y.clear();
        if (hybrid_astar_ptr->plan(goal_pose, start_pose, result)) {
          vector<Vec3d> path2smooth_;
          path2smooth_.clear();
          // Discard the endpoints.
          for (int i = 1; i < result.x.size() - 1; ++i) {
            path2smooth_.push_back({result.x[i], result.y[i], result.theta[i]});
          }
          astarpath.push_back(path2smooth_);
          smoother_ptr->optimize(voronoiDiagram, path2smooth_, grid_);
          vector<Vec3d> smooth_path_;
          smooth_path_.clear();
          smoother_ptr->getSmoothPath(smooth_path_);
          write_file(file_path_, smooth_path_, result.total_length);
          astarpath_.push_back(smooth_path_);
        } else {
          ROS_ERROR("search fail");
          return;
        }
      }
    }
    // visPath(astarpath);
    visOptPath(astarpath_);
  }

  auto HalfStructPlanner::get_path(geometry_msgs::PoseStamped const& start,
                                  geometry_msgs::PoseStamped const& goal) -> std::vector<geometry_msgs::PoseStamped> {
    std::vector<geometry_msgs::PoseStamped> result_p{};
    std::unique_ptr<std::vector<Eigen::Vector3d>> result_ = std::make_unique<std::vector<Eigen::Vector3d>>();
    std::unique_ptr<std::vector<Eigen::Vector3d>> result_plo = std::make_unique<std::vector<Eigen::Vector3d>>();
    if (!is_traffic_plan()) return result_p;

    for (auto i = 0; i < node_num_; ++i) {
      for (auto j = 0; j < node_num_; ++j) {
        graph_[i][j] = graph_bk_[i][j]; // Cost matrix
      }
    }

    // 0 corresponds to the start point; 1 corresponds to the end point.
    nodes_[0] = start;
    nodes_[1] = goal;
    calculate_graph(); // Update the topology and cost values.
    auto node_list = dijkstra(); // node_list stores the waypoint indices.
    node_list.emplace(node_list.begin(), 0); // Insert the starting point.
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    double l_s = 0., l_p;
    for (int i = 0; i < node_list.size() - 1; ++i) {
      try {
        std::pair<int, int> inputKey = {node_list[i], node_list[i + 1]};
        // Retrieve all poses based on the node indices to form a complete path
        const std::vector<Eigen::Vector3d>* sgPathPtr = mappingTable[inputKey];
        result_plo->insert(result_plo->end(), sgPathPtr->begin(), sgPathPtr->end());
      } catch (const std::bad_alloc& ba) {
        std::cerr << "Caught a bad_alloc exception: " << ba.what() << '\n';
      }
      l_p += graph_[node_list[i]][node_list[i + 1]]; // Accumulate the lengths of each segment
    }

    std::vector<Eigen::Vector3d> result_plo_copy = *result_plo;
    *result_ = std::move(*result_plo);
    if (distance(start, goal) < 15. && line_safe(start, goal)) {
      // Allow direct connection between the start and end points. Later, the length is obtained through planning, and the shortest route is determined by comparing it with the shortest route obtained through Dijkstra's search.
      Vec3d start_pose = Eigen::Vector3d{nodes_[0].pose.position.x, nodes_[0].pose.position.y, tf2::getYaw(nodes_[0].pose.orientation)};
      Vec3d goal_pose = Eigen::Vector3d{nodes_[1].pose.position.x, nodes_[1].pose.position.y, tf2::getYaw(nodes_[1].pose.orientation)};
      result.theta.clear();
      result.x.clear();
      result.y.clear();
      vector<Vec3d> smooth_path;
      if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
        vector<Vec3d> path2smooth;
        path2smooth.clear();
        for (int i = 0; i < result.x.size(); ++i) {
          path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});
        }
        smoother_ptr->optimize(voronoiDiagram, path2smooth, grid_);
        smooth_path.clear();
        smoother_ptr->getSmoothPath(smooth_path);
      }
      l_s = result.total_length; // Total length of the direct connection
      // Take the direct connection
      if (l_p > 3 * l_s) {
        result_->clear();
        for (const auto& vec : smooth_path) {
          result_->push_back(Eigen::Vector3d(vec.x(), vec.y(), vec.z()));
        }
      }
    }

    for (auto const& node : *result_) {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      p.pose.position.x = node[0];
      p.pose.position.y = node[1];
      auto o = node[2];
      p.pose.orientation.z = std::sin(o / 2);
      p.pose.orientation.w = std::cos(o / 2);
      result_p.emplace_back(p);
    }
    path.poses = result_p;
    visfullPath(result_plo_copy);
    res_pub_.publish(path);
    // Display the waypoint indices on this path.
    show_graph_(node_list);
    ContactS_paths.clear();
    ContactG_paths.clear();
    ContactS_paths_.clear();
    ContactG_paths_.clear();
    ContactSG_paths.clear();
    return result_p;
  }

  auto HalfStructPlanner::is_traffic_plan() -> bool {
    return !traffic_routes_.empty();
  }

  // Preprocess the graph and calculate costs
  void HalfStructPlanner::calculate_pre_graph(lines_type const& lines, vector<double> const& distance_table) {
    traffic_routes_ = std::move(lines);
    auto num = traffic_routes_.size();
    if (traffic_routes_.empty()) {
      ROS_WARN("Receive empty traffic routes, planner not working!");
      return;
    }
    // Number of start and end points in the network (traffic_routes_.size()) * 2 (each start point is also an end point) + start and end points of planning (2) + connection points (connection_point_num * 2)
    node_num_ = static_cast<int>(traffic_routes_.size() * 2 + 2 + connection_point_num * 2);
    // Index meaning:
    // 0: Start point 1: End point
    // 2 ~ 1 + connection_point_num: New nodes introduced by the start point
    // 2 + connection_point_num ~ 1 + 2 * connection_point_num: New nodes introduced by the end point
    // 2 + 2 * connection_point_num ~ end: Traffic route nodes (processed by this function)
    nodes_.resize(node_num_);
    graph_ = new double*[node_num_];
    graph_bk_ = new double*[node_num_];
    for (auto i = 0; i < node_num_; ++i) {
      graph_[i] = new double[node_num_];
      graph_bk_[i] = new double[node_num_];
      for (auto j = 0; j < node_num_; ++j) graph_[i][j] = std::numeric_limits<double>::max();
      graph_[i][i] = 0.0;
    }

    for (auto i = 0; i < traffic_routes_.size(); ++i) {
      auto start = 2 + 2 * connection_point_num + 2 * i;
      auto end = 2 + 2 * connection_point_num + 2 * i + 1;
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      auto o = traffic_routes_[i].a.theta;
      p.pose.orientation.z = std::sin(o / 2);
      p.pose.orientation.w = std::cos(o / 2);
      p.pose.position.x = traffic_routes_[i].a.x;
      p.pose.position.y = traffic_routes_[i].a.y;
      nodes_[start] = p;
      o = traffic_routes_[i].b.theta;
      p.pose.orientation.z = std::sin(o / 2);
      p.pose.orientation.w = std::cos(o / 2);
      p.pose.position.x = traffic_routes_[i].b.x;
      p.pose.position.y = traffic_routes_[i].b.y;
      nodes_[end] = p;
      graph_[start][end] = distance_table[i];
      // Fill the mapping table
      const std::vector<Eigen::Vector3d>& sgPathRef = SG_paths[i];
      mappingTable[{start, end}] = &sgPathRef;
    }

    auto j = 0;
    for (auto i = 0; i < traffic_routes_.size(); ++i) {
      auto end = 2 + 2 * connection_point_num + 2 * i + 1;
      for (auto const& node : traffic_routes_[i].go_list) {
        if (node >= traffic_routes_.size()) { // Invalid
          ROS_ERROR("line %d go list %d out of range", i, node);
          continue;
        }
        auto start = 2 + 2 * connection_point_num + 2 * node;
        graph_[end][start] = distance_table[num + j];
        const std::vector<Eigen::Vector3d>& tpPathRef = topology_paths[j];
        mappingTable[{end, start}] = &tpPathRef;
        j++;
      }
    }
    // Backup
    for (auto i = 0; i < node_num_; ++i) {
      for (auto j = 0; j < node_num_; ++j) {
        graph_bk_[i][j] = graph_[i][j];
      }
    }
    show_graph(); // Display waypoint indices
  }

  // Graph update: Add new start and end points to the topology
  void HalfStructPlanner::calculate_graph() {
    ContactS_paths.clear();
    ContactG_paths.clear();
    ContactS_paths_.clear();
    ContactG_paths_.clear();
    ContactSG_paths.clear();
    std::set<setnode> start_set, goal_set; // A set of information about the connections from the start point to each node, and from the end point to each node, an ordered set
    auto start = point_t{nodes_[0].pose.position.x, nodes_[0].pose.position.y};
    auto goal = point_t{nodes_[1].pose.position.x, nodes_[1].pose.position.y};
    setnode sn{}, gn{};

    for (auto i = 0; i < traffic_routes_.size(); ++i) { 
      // Find the connection point from the car's start point to each road network. We see that this connection point only appears on the edges, not on the topology.
      std::vector<Eigen::Vector3d> const& my_paths = SG_paths[i];
      sn = nearest_point_of_segment(start, &my_paths, 1);
      if (sn.nearest_dist < connection_points_range) {
        sn.line = i;
        start_set.insert(sn);
      }
      gn = nearest_point_of_segment(goal, &my_paths, 0);
      if (gn.nearest_dist < connection_points_range) {
        gn.line = i;
        goal_set.insert(gn);
      }
    }

    int line_s, line_g; 
    for (auto i = 0; i < connection_point_num; ++i) {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      if (i < start_set.size()) {
      auto sen = 2 + i;
      line_s = 2 + 2 * connection_point_num + 2 * std::next(start_set.begin(), i)->line;
      line_g = line_s + 1;
        // Also, plan the route from the start and end points to the connection points.
        Vec3d start_pose = Eigen::Vector3d{nodes_[0].pose.position.x, nodes_[0].pose.position.y, tf2::getYaw(nodes_[0].pose.orientation)}; // Start point
        auto contact = std::next(start_set.begin(), i)->nearest_point;
        Vec3d goal_pose = Eigen::Vector3d{contact.x, contact.y, contact.theta};
        double lb, lf;
        result.theta.clear();
        result.x.clear();
        result.y.clear();
        vector<Vec3d> smooth_path;
        if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
          vector<Vec3d> path2smooth_b;
          path2smooth_b.clear();
          for (int i = 0; i < result.x.size() - 1; ++i) {
            path2smooth_b.push_back({result.x[i], result.y[i], result.theta[i]});
          }
          smoother_ptr->optimize(voronoiDiagram, path2smooth_b, grid_);
          smooth_path.clear();
          smoother_ptr->getSmoothPath(smooth_path);
          lb = result.total_length;
        }
        graph_[0][sen] = lb; 
        contact = std::next(start_set.begin(), i)->nearest_point;
        auto it = std::next(start_set.begin(), i);
        auto cn = *it;
        start_set.erase(it);
        start_set.insert(cn);
        std::vector<Eigen::Vector3d>::const_iterator startIt;
        std::vector<Eigen::Vector3d>::const_iterator endIt;
        startIt = smooth_path.begin();
        endIt = smooth_path.end();
        ContactS_paths.push_back(std::vector<Eigen::Vector3d>(startIt, endIt));
        const std::vector<Eigen::Vector3d>& CSPathRef = ContactS_paths[i];
        mappingTable[{0, sen}] = &CSPathRef;

        p.pose.position.x = contact.x;
        p.pose.position.y = contact.y;
        auto o = contact.theta;
        p.pose.orientation.z = std::sin(o / 2);
        p.pose.orientation.w = std::cos(o / 2);
        nodes_[sen] = p;
        graph_[sen][line_g] = graph_[line_s][line_g] - static_cast<double>(step_size * std::next(start_set.begin(), i)->nearest_index); // Update the distance from the connection point to the end point of the edge where the connection point is located.
        auto path_pose = SG_paths[std::next(start_set.begin(), i)->line];
        int startIndex = std::next(start_set.begin(), i)->nearest_index;
        startIt = path_pose.begin() + startIndex + 1;
        endIt = path_pose.end();
        ContactS_paths_.push_back(std::vector<Eigen::Vector3d>(startIt, endIt));
        const std::vector<Eigen::Vector3d>& CSPathRef_ = ContactS_paths_[i];
        mappingTable[{sen, line_g}] = &CSPathRef_;
      }
    
      // Similarly, update the end point and its connection points.
      if (i < goal_set.size()) {
        auto gen = 2 + connection_point_num + i;
        line_s = 2 + 2 * connection_point_num + 2 * std::next(goal_set.begin(), i)->line;
        Vec3d goal_pose = Eigen::Vector3d{nodes_[1].pose.position.x, nodes_[1].pose.position.y, tf2::getYaw(nodes_[1].pose.orientation)};
        auto contact = std::next(goal_set.begin(), i)->nearest_point;
        Vec3d start_pose = Eigen::Vector3d{contact.x, contact.y, contact.theta};
        result.theta.clear();
        result.x.clear();
        result.y.clear();
        double lb, lf;
        vector<Vec3d> smooth_path;
        if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
          vector<Vec3d> path2smooth_b;
          path2smooth_b.clear();
          for (int i = 1; i < result.x.size(); ++i) {
            path2smooth_b.push_back({result.x[i], result.y[i], result.theta[i]});
          }
          smoother_ptr->optimize(voronoiDiagram, path2smooth_b, grid_);
          smooth_path.clear();
          smoother_ptr->getSmoothPath(smooth_path);
          lb = result.total_length;
        }
        graph_[gen][1] = lb;
        contact = std::next(goal_set.begin(), i)->nearest_point;
        auto it = std::next(goal_set.begin(), i);
        auto cn = *it;
        goal_set.erase(it);
        goal_set.insert(cn);

        std::vector<Eigen::Vector3d>::const_iterator startIt;
        std::vector<Eigen::Vector3d>::const_iterator endIt;
        startIt = smooth_path.begin();
        endIt = smooth_path.end();
        ContactG_paths.push_back(std::vector<Eigen::Vector3d>(startIt, endIt));
        const std::vector<Eigen::Vector3d>& CSPathRef = ContactG_paths[i];
        mappingTable[{gen, 1}] = &CSPathRef;
        p.pose.position.x = contact.x;
        p.pose.position.y = contact.y;
        auto o = contact.theta;
        p.pose.orientation.z = std::sin(o / 2);
        p.pose.orientation.w = std::cos(o / 2);
        nodes_[gen] = p;
        graph_[line_s][gen] = step_size * std::next(goal_set.begin(), i)->nearest_index;
        auto path_pose = SG_paths[std::next(goal_set.begin(), i)->line];
        int startIndex = std::next(goal_set.begin(), i)->nearest_index;
        startIt = path_pose.begin();
        endIt = path_pose.begin() + startIndex;
        ContactG_paths_.push_back(std::vector<Eigen::Vector3d>(startIt, endIt));
        const std::vector<Eigen::Vector3d>& CGPathRef_ = ContactG_paths_[i];
        mappingTable[{line_s, gen}] = &CGPathRef_;
      }
    }
    
    // Update the cost from the new points introduced by the start point to the new points introduced by the end point, addressing the case where the start and end points are on the same line.
    auto m = 0;
    for (auto i = 0; i < connection_point_num; ++i) {
      if (i < start_set.size()) {
        auto sen = 2 + i;
        line_s = 2 + 2 * connection_point_num + 2 * std::next(start_set.begin(), i)->line;
        line_g = line_s + 1;
          for (auto j = 0; j < connection_point_num; ++j) {
            if (j < goal_set.size()) {
              auto gen = 2 + connection_point_num + j;
              if (std::next(goal_set.begin(), j)->line == std::next(start_set.begin(), i)->line
              && graph_[line_s][line_g] - graph_[line_s][gen] <= graph_[sen][line_g]) {
                  graph_[sen][gen] = graph_[sen][line_g] - (graph_[line_s][line_g] - graph_[line_s][gen]);
                  auto path_pose = SG_paths[std::next(goal_set.begin(), j)->line];
                  int startIndex = std::next(start_set.begin(), i)->nearest_index;
                  int endIndex = std::next(goal_set.begin(), j)->nearest_index;
                  ContactSG_paths.push_back(std::vector<Eigen::Vector3d>(path_pose.begin() + startIndex, path_pose.begin() + endIndex - 1));
                  const std::vector<Eigen::Vector3d>& SGPathRef_ = ContactSG_paths[m++];
                  mappingTable[{sen, gen}] = &SGPathRef_;
              }
            }
          }
      }
    }
  }

  void HalfStructPlanner::show_graph() {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    geometry_msgs::Point p1, p2;
    geometry_msgs::Pose p;
    p.orientation.w = 1.0;
    auto make_arrow_marker = [&](double r, double g, double b, geometry_msgs::Point const& p1, geometry_msgs::Point const& p2) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.3;
      marker.scale.y = 0.2;
      marker.scale.z = 0.4;
      marker.color.a = 0.5;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.pose.orientation.w = 1.0;
      marker.points.resize(2);
      marker.ns = "arrow";
      marker.id = id++;
      marker.points[0] = p1;
      marker.points[1] = p2;
      marker_array.markers.emplace_back(marker);
    };
    auto make_text_marker = [&](double r, double g, double b, geometry_msgs::Pose const& p, std::string const& t) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.z = 5.0;
      marker.color.a = 1.0;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.pose = p;
      marker.ns = "text";
      marker.id = id++;
      marker.text = t;
      marker_array.markers.emplace_back(marker);
    };
    for (auto i = 0; i < node_num_; ++i) {
      p.position.x = nodes_[i].pose.position.x-0.75;
      p.position.y = nodes_[i].pose.position.y;
      make_text_marker(0.0, 0.0, 0.0, p, std::to_string(i));
    }
    vis_pub_.publish(marker_array);
  }

  void HalfStructPlanner::show_graph_(std::vector<int> node_list) {
    visualization_msgs::MarkerArray marker_array_;
    int id = 0;
    geometry_msgs::Point p1, p2;
    geometry_msgs::Pose p;
    p.orientation.w = 1.0;
    auto make_text_marker = [&](double r, double g, double b, geometry_msgs::Pose const& p, std::string const& t) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.z = 1.0;
      marker.color.a = 1.0;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.pose = p;
      marker.ns = "text";
      marker.id = id++;
      marker.text = t;
      marker_array_.markers.emplace_back(marker);
    };
    
    for (auto i = 0; i < node_list.size(); ++i) {
      p.position.x = nodes_[node_list[i]].pose.position.x - 0.5;
      p.position.y = nodes_[node_list[i]].pose.position.y;
      make_text_marker(1.0, 0.0, 0.0, p, std::to_string(node_list[i]));
    }
    vis_pub_.publish(marker_array_);
    marker_array_.markers.clear();
  }

  // Calculate the nearest point on a certain route from the starting (ending) point, and flag whether it is the starting or ending point.
  auto HalfStructPlanner::nearest_point_of_segment(point_t const& p, vector<Eigen::Vector3d> const* paths, bool flag) -> setnode {
    float dist = FLT_MAX;
    int ii = 0;
    float min_dist = FLT_MAX;
    int min_index = -1;
    for (const auto& path : *paths) {
      dist = static_cast<float>(std::pow(p.x - path[0], 2) + std::pow(p.y - path[1], 2));
      if (dist < min_dist) {
        min_dist = dist;
        min_index = ii;
      }
      ++ii;
    }

    setnode result{};
    int min_index_;
    if (flag) {
      // To enable smooth planning of the route between the start/end point and the connection point, let the connection point take a point after the nearest connection point.
      if (min_index == 0) {
        min_index_ = min_index;
      } else {
        min_index_ = min_index + 8;
        if (min_index_ >= paths->size() - 1) min_index_ = paths->size() - 1;
      }
      result.nearest_point.x = (*paths)[min_index_][0];
      result.nearest_point.y = (*paths)[min_index_][1];
      result.nearest_point.theta = (*paths)[min_index_][2];
      result.nearest_index = min_index_;
    } else {
      if (min_index == paths->size() - 1) {
        min_index_ = min_index;
      } else {
        min_index_ = min_index - 8;
        if (min_index_ <= 0) min_index_ = 0;
      }
      result.nearest_point.x = (*paths)[min_index_][0];
      result.nearest_point.y = (*paths)[min_index_][1];
      result.nearest_point.theta = (*paths)[min_index_][2];
      result.nearest_index = min_index_;
    }
    result.nearest_dist = std::sqrt(min_dist);
    return result;
  }

  auto HalfStructPlanner::distance(point_t const& a, point_t const& b) -> double {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  // Print the cost matrix.
  void HalfStructPlanner::print_graph() {
    std::cout << "node num: " << node_num_ << std::endl;
    std::cout << SET_FONT_ON;
    std::cout << SET_FONT_1;
    for (auto i = 0; i < node_num_; ++i) std::cout << GREEN << std::setw(3) << i << " " << WHITE;
    std::cout << "\n";
    for (auto i = 0; i < node_num_; ++i) {
      for (auto j = 0; j < node_num_; ++j) {
        if (graph_[i][j] != std::numeric_limits<double>::max()) {
          std::string str{"000 "};
          auto gij = std::to_string(graph_[i][j]);
          for (auto k = 0; k < 3; ++k) {
            if (gij.size() > k) str[k] = gij[k];
          }
          std::cout << str.c_str();
        } else {
          std::cout << "*** "; // Infinite cost.
        }
      }
      std::cout << GREEN << " " << i << "\n" << WHITE;
    }
    std::cout << SET_FONT_ON;
    std::cout << SET_FONT_0;
  }

  // Use Dijkstra's algorithm to plan a path with the minimum distance cost.
  auto HalfStructPlanner::dijkstra() -> std::vector<int> {
    std::vector<int> result;
    struct dijnode {
      int node;
      int prev_node; // The previous node connected to it.
      double cost;
      bool operator<(dijnode const& rhs) const {
        return cost < rhs.cost;
      }
      bool operator==(dijnode const& rhs) const {
        return node == rhs.node;
      }
    };

    std::set<dijnode> open_list;
    std::vector<dijnode> close_list;
    open_list.insert(dijnode{0, 0, 0.0});
    while (!open_list.empty()) {
      auto node = open_list.extract(open_list.begin()).value();
      if (node.node == 1) {
        while (node.prev_node != 0) {
          result.emplace_back(node.node);
          auto prev = std::find_if(close_list.begin(), close_list.end(),
                                  [&](dijnode const& a) { return a.node == node.prev_node; });
          if (prev == close_list.end()) {
            ROS_ERROR("No whole path while graph search success, that should not happen!");
            break;
          }
          node = *prev;
        }

        result.emplace_back(node.node);
        std::reverse(result.begin(), result.end());
        // Print the path.
        std::cout << RED << "0";
        for (auto const& r : result) std::cout << RED << "->" << r;
        std::cout << WHITE << std::endl;
        break;
      }

      // Here, find the shortest path.
      close_list.emplace_back(node);
      for (auto i = 0; i < node_num_; ++i) {
        if (i == node.node) continue;
        if (std::find_if(close_list.begin(), close_list.end(),
                        [&](dijnode const& a) { return a.node == i; }) != close_list.end()) continue;
        if (graph_[node.node][i] == std::numeric_limits<double>::max()) continue;

        dijnode open_node{i, node.node, node.cost + graph_[node.node][i]};
        auto iter2 = std::find(open_list.begin(), open_list.end(), open_node);
        if (iter2 != open_list.end()) {
          if (iter2->cost > open_node.cost) {
            open_list.erase(iter2);
            open_list.insert(open_node);
          }
        } else {
          open_list.insert(open_node);
        }
      }
    }
    return result;
  }

  bool HalfStructPlanner::line_safe(point_t const& a, point_t const& b) {
    auto check_point_num = static_cast<int>(std::hypot(a.x - b.x, a.y - b.y) / (grid_->info.resolution * 5));
    double delta_x = (b.x - a.x) / check_point_num;
    double delta_y = (b.y - a.y) / check_point_num;
    double cur_x = a.x;
    double cur_y = a.y;

    auto mapObstacle = [&](double x, double y) {
        auto i = std::floor((x - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
        auto j = std::floor((y - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
        auto idx = i + j * grid_->info.width;
        return grid_->data[idx] == 100 || grid_->data[idx] == -1; // 100 is the obstacle pixel value, -1 is unknown.
    };

    for (int i = 0; i < check_point_num; ++i) {
        if (mapObstacle(cur_x, cur_y)) return false;
        cur_x += delta_x;
        cur_y += delta_y;
    }
    return true;
  }

  void HalfStructPlanner::visOptPath(std::vector<vector<Eigen::Vector3d>> paths) {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    auto path_marker = [&](vector<Vec3d> const& paths) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "opt_db_path";
      marker.id = id++;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.scale.x = 0.1; 
      marker.color.a = 1.0; 
      marker.color.r = 0.0; 
      marker.color.g = 0.0;
      marker.color.b = 1.0; 
      for (const auto& path : paths) {
          geometry_msgs::Point point;
          point.x = path[0];
          point.y = path[1];
          point.z = 0.05;
          marker.points.push_back(point);
      }
      marker_array.markers.emplace_back(marker);
    };

    for (auto const& path : paths) {
        path_marker(path);
    }
    optimized_path_vis_pub.publish(marker_array);
  }


  // Visualize the path before smoothing.
  void HalfStructPlanner::visPath(std::vector<vector<Eigen::Vector3d>> paths) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker path_point;
    int id = 0;
    auto path_marker = [&](vector<Vec3d> const& paths) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "db_path";
      marker.id = id++;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.scale.x = 0.1; 
      marker.color.a = 1.0; 
      marker.color.r = 0.0; 
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      for (const auto& path : paths) {
          geometry_msgs::Point point;
          point.x = path[0];
          point.y = path[1];
          point.z = 0.05;
          marker.points.push_back(point);
      }
      marker_array.markers.emplace_back(marker);
    };
    auto point_marker = [&](vector<Vec3d> const& paths) {
      path_point.id = id++;
      path_point.header.frame_id = "map";
      path_point.header.stamp = ros::Time::now();
      path_point.type = visualization_msgs::Marker::POINTS;
      path_point.scale.x = 0.1;
      path_point.scale.y = 0.1;
      path_point.scale.z = 0.1;
      path_point.color.a = 1;
      path_point.color.r = 1;
      path_point.color.g = 0;
      path_point.color.b = 0;
      path_point.pose.orientation.w = 1.0;
      for (size_t i = 0; i < paths.size(); i++) {
        geometry_msgs::Point p;
        p.x = paths[i][0];
        p.y = paths[i][1];
        p.z = 0.05;
        path_point.points.push_back(p);
      }
    };

    for (auto const& path : paths) {
        path_marker(path);
    }
    path_vis_pub.publish(marker_array);
  }

  auto HalfStructPlanner::write_file(std::string& file_path, std::vector<Eigen::Vector3d> const& path, double length) -> bool {
    std::ofstream out(file_path.c_str(), std::ios_base::out | std::ios_base::app);
    // Allow appending content.
    if (!out.is_open()) {
      ROS_ERROR("Open file %s failed!", file_path.c_str());
      return false;
    }
    out << std::to_string(length) << "\n";

    for (auto const& p : path) {
      out << std::to_string(p[0]) << " "
          << std::to_string(p[1]) << " "
          << std::to_string(p[2]) << "\n";
    }
    out << "EOP" << "\n";
    out.close();
    return true;
  }

  // Display the route between the start and end points.
  void HalfStructPlanner::visSGPath(const std::vector<vector<Eigen::Vector3d>>& paths) {
      SG_paths = std::move(paths);

      int id = 0;
      geometry_msgs::PoseArray multi_pose_array;
      multi_pose_array.header.frame_id = "map";

      auto path_marker = [&](vector<Vec3d> const& paths) {
        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "map";
        for (const auto& path : paths) {
          geometry_msgs::Pose pose;
          pose.position.x = path[0];
          pose.position.y = path[1];
          pose.position.z = 0.05;
          pose.orientation.z = std::sin(path[2] / 2.0);
          pose.orientation.w = std::cos(path[2] / 2.0);
          pose_array.poses.push_back(pose);
        }
        multi_pose_array.poses.insert(multi_pose_array.poses.end(), pose_array.poses.begin(), pose_array.poses.end());
      };

      for (auto const& path : SG_paths) {
          path_marker(path);
      }
      SG_path_vis_pub.publish(multi_pose_array);
  }

  void HalfStructPlanner::vistopologyPath(const std::vector<vector<Eigen::Vector3d>>& paths) {
      topology_paths = std::move(paths);
      int id = 0;
      geometry_msgs::PoseArray multi_pose_array;
      multi_pose_array.header.frame_id = "map";

      auto path_marker = [&](vector<Vec3d> const& paths) {
        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "map";
        for (const auto& path : paths) {
          geometry_msgs::Pose pose;
          pose.position.x = path[0];
          pose.position.y = path[1];
          pose.position.z = 0.05;
          pose.orientation.z = std::sin(path[2] / 2.0);
          pose.orientation.w = std::cos(path[2] / 2.0);
          pose_array.poses.push_back(pose);
        }
        multi_pose_array.poses.insert(multi_pose_array.poses.end(), pose_array.poses.begin(), pose_array.poses.end());
      };

      for (auto const& path : topology_paths) {
          path_marker(path);
      }
      topology_path_vis_pub.publish(multi_pose_array);
  }

  // Display the complete alternative topological route.
  void HalfStructPlanner::visfullPath(const std::vector<Eigen::Vector3d>& poses) {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    auto pose_marker = [&](Vec3d const& pose) {
      geometry_msgs::Pose pose_;
      pose_.position.x = pose[0];
      pose_.position.y = pose[1];
      pose_.position.z = 0.05;
      pose_.orientation.z = std::sin(pose[2] / 2.0);
      pose_.orientation.w = std::cos(pose[2] / 2.0);
      pose_array.poses.push_back(pose_);
    };
    for (auto const& pose : poses) {
        pose_marker(pose);
    }
    full_path_vis_pub.publish(pose_array);
  }

  // Hybrid A* planning.
  void HalfStructPlanner::HybridAstarplan(Vec3d start_pose, Vec3d goal_pose) {
    HybridAstarResult result;
    vector<vector<Vec3d>> astarpath;
    vector<vector<Vec3d>> astarpath_;
    auto hybrid_astar_start_ = chrono::high_resolution_clock::now();
    if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
        vector<Vec3d> path2smooth;
        for (int i = 0; i < result.x.size(); ++i) {
            path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});
        }
        astarpath.push_back(path2smooth);
        visPath(astarpath);
        smoother_ptr->optimize(voronoiDiagram, path2smooth, grid_);
        vector<Vec3d> smooth_path;
        smoother_ptr->getSmoothPath(smooth_path);
        astarpath_.push_back(smooth_path);
        visOptPath(astarpath_);
    } else {
        ROS_ERROR("search fail");
        return;
    }
    auto hybrid_astar_end = chrono::high_resolution_clock::now();
    chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start_;
    cout << "Shortest path length: " << result.total_length << "m" << endl;
    cout << "Shortest path planning time (with optimization): " << hybrid_astar_use_time.count() * 1000 << "ms" << endl;

    // for (size_t i = 0; i < result.x.size(); ++i)
    // {
    //     vis.publishVehicleBoxes({smooth_path[i](0), smooth_path[i](1), smooth_path[i](2)}, i); // Vehicle outline based on the smoothed path points
    //     vis.publishVehicleBoxes({result.x[i], result.y[i], result.theta[i]}, i); // Vehicle outline based on the unoptimized path points
    // }
  }


}
