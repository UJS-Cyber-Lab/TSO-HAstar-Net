#include "half_struct_planner.h"


namespace chibot::topologyplanning {

  // Member initialization lists are used to initialize member variables of a class before the constructor starts executing. The member initialization list is located after the parameter list of the constructor, separated by a colon, and is used to assign initial values to member variables.

  /* If the constructor of HalfStructPlanner requires a nav_msgs::OccupancyGrid object, the best way is to make it accept a parameter of type nav_msgs::OccupancyGridConstPtr,
    instead of nav_msgs::OccupancyGrid or nav_msgs::OccupancyGrid&. This way, you can avoid copying the entire nav_msgs::OccupancyGrid object when calling the constructor.
    This way, HalfStructPlanner will hold a pointer to nav_msgs::OccupancyGrid instead of the entire object. When you call half_struct_planner_ = std::make_shared<HalfStructPlanner>(msg);, no copying will occur, only the pointer in msg will be copied to the internals of HalfStructPlanner.
    Note that this approach assumes that the nav_msgs::OccupancyGrid object in msg remains valid throughout the lifetime of HalfStructPlanner. If the original object is destroyed or modified, the pointer in HalfStructPlanner may become invalid. If you cannot guarantee this, then you may need to copy the nav_msgs::OccupancyGrid object in the constructor of HalfStructPlanner. */
  HalfStructPlanner::HalfStructPlanner(const nav_msgs::OccupancyGridConstPtr& grid):
  // nav_msgs::OccupancyGrid const& grid
  // const nav_msgs::OccupancyGridConstPtr& msg
  // const nav_msgs::OccupancyGrid::Ptr map
    grid_(grid), 
    node_num_(0),
    graph_(nullptr),
    ContactS_paths(EXTRA_POINTS_NUM),
    ContactG_paths(EXTRA_POINTS_NUM),
    ContactS_paths_(EXTRA_POINTS_NUM),
    ContactG_paths_(EXTRA_POINTS_NUM),
    ContactSG_paths(EXTRA_POINTS_NUM),
    graph_bk_(nullptr) {
    // {
    // grid_ = grid;
    hybrid_astar_ptr.reset(new HybridAstar(grid));

    // create the voronoi object and initialize it with the map
    // Generate the Voronoi diagram
    int size_x = grid_->info.width;
    int size_y = grid_->info.height;
    // Initialize a binary map for building the Voronoi diagram
    bool **bin_map = NULL;
    // Create a size_x * size_y boolean 2D array bin_map
    bin_map = new bool *[size_x];
    for (int i = 0; i < size_x; ++i) {
      // Column index
      bin_map[i] = new bool[size_y];
    }
    for (int j = size_y - 1; j >= 0; j--) {
      for (int i = 0; i < size_x; i++) {
        auto index_ = j * grid_->info.width + i;
        // Get the grid index from the row and column indices
        // Both obstacle grids and unknown grids are treated as obstacle grids
        if (grid_->data[index_] == 100 || grid_->data[index_] == -1) {
          bin_map[i][j] = true;
          // Indicates that this position is an obstacle
        } else {
          bin_map[i][j] = false;
        }
      }
    }
    // Finally, the 2D array bin_map records whether each cell on the map is passable
    voronoiDiagram.initializeMap(size_x, size_y, bin_map);
    // Initialize the Voronoi diagram
    voronoiDiagram.update();
    // Update the distance map and Voronoi diagram
    voronoiDiagram.prune();
    // Prune the diagram

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
    //         if (voronoiDiagram.isVoronoi(i, j)) { // Nodes on the Voronoi diagram
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

  // Destructor
  HalfStructPlanner::~HalfStructPlanner() {
    if (node_num_ == 0) return;
    for (auto i = 0; i < node_num_; ++i) {
      /* delete [] graph_[i]; is a line of code used to release dynamically allocated memory.
        Based on the context of the code, graph_ is a 2D array, and each row of it points to a dynamically allocated memory block. The line delete [] graph_[i]; releases the memory block pointed to by the i-th row.
        In C++, the new operator is used for memory allocation, and the delete operator is used for memory deallocation. When we dynamically allocate memory using the new operator, we must use the corresponding delete operator to release that memory to avoid memory leaks.
        In this specific line of code, delete [] graph_[i]; indicates the release of the memory block pointed to by the i-th row. The [] signifies that a contiguous block of memory is being released, not just a single object. This syntax is applicable to arrays or pointers to dynamically allocated memory blocks.
        It is important to ensure that the memory block has been dynamically allocated and is not being double-freed before executing delete. Otherwise, it may lead to undefined behavior or program crashes. */
      delete [] graph_[i];
      delete [] graph_bk_[i];
    }
    /* delete [] graph_; is a line of code used to release the memory occupied by a dynamically allocated 2D array.
      Based on the context of the code, graph_ should be a pointer to a dynamically allocated 2D array. The line delete [] graph_; releases the memory occupied by the entire 2D array.
      In C++, when using the new operator for dynamic memory allocation, we need to use the corresponding delete operator to release that memory to avoid memory leaks. For 2D arrays, we need to use delete [] twice to release the memory—once to release the memory of each row and then to release the memory of the row pointers. */
    delete [] graph_;
    delete [] graph_bk_;
  };

  // Initialization
  void HalfStructPlanner::init() {
    ros::NodeHandle nh("/");
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("traffic_route", 1);
    // Visualize the sequence number of the traffic network
    res_pub_ = nh.advertise<nav_msgs::Path>("traffic_route_points", 1);
    // Publish the shortest path on the traffic network, a series of pose points, and visualize the pose points

    // Visualize the curve before optimization, the route planned by Hybrid A*
    path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("db_path_marker", 10);
    // Visualize the waypoints of the curve before optimization
    path_point_vis_pub = nh.advertise<visualization_msgs::Marker>("db_path_point_marker", 10);
    // Visualize the smoothed path
    optimized_path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/path", 10);

    // voronoiDiagram_ptr.reset(new DynamicVoronoi());
    smoother_ptr.reset(new Smoother()); // Initialize the smoother

    // Visualize the Voronoi diagram
    // voronoi_pub = nh.advertise<visualization_msgs::Marker>("voronoi", 10);

    // Visualize the route between start and end points
    SG_path_vis_pub = nh.advertise<geometry_msgs::PoseArray>("SG_path_marker", 100);
    // Visualize the topology
    topology_path_vis_pub = nh.advertise<geometry_msgs::PoseArray>("topology_path_marker", 100);
    // A complete alternative topological route, whether to choose it depends on its length compared to the direct connection
    full_path_vis_pub = nh.advertise<geometry_msgs::PoseArray>("full_path_marker", 100);
  }

  // Set traffic rules (start and end segments)
  void HalfStructPlanner::set_traffic_route(lines_type const& lines) {
    traffic_routes_ = lines;
    // Contains the coordinates of the start and end points, information about which edges this edge can connect to, etc. A traffic_route represents an edge (excluding topology).

    // voronoi_pub.publish(voronoi); // Visualize the Voronoi diagram

    // Save the route
    std::string root_path;
    // Parameter service, value taken from the launch file
    ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
    auto name = ss.str();
    // Save the route between the start and end points
    auto dir = root_path + "/path/";
    auto file_path_ = dir + name;

    // Plan for each line
    // First, save all generated paths, then visualize them together
    vector<vector<Vec3d>> astarpath;
    // Unoptimized route set
    vector<vector<Vec3d>> astarpath_;
    // Optimized route set

    // Visualize vis;
    for (const auto& traffic_route_ : traffic_routes_) {
      // Route connected by start and end points
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
          // Load the path to be smoothed
        }
        astarpath.push_back(path2smooth);
        // Save the unoptimized path

        // l(Limited-memory)-bfgs smoothing
        smoother_ptr->optimize(voronoiDiagram, path2smooth, grid_);
        // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_);
        // This is the gradient descent version

        vector<Vec3d> smooth_path;
        smooth_path.clear();
        smoother_ptr->getSmoothPath(smooth_path);

        write_file(file_path_, smooth_path, result.total_length);
        // Write the optimized route to a file

        astarpath_.push_back(smooth_path);
        // Save the optimized path

        // for (size_t i = 0; i < result.x.size(); ++i)
        // {
        //  // Vehicle outline based on the smoothed path points
        //     vis.publishVehicleBoxes({smooth_path[i](0), smooth_path[i](1), smooth_path[i](2)}, i);
        //  // Vehicle outline based on the unoptimized path points
        //     vis.publishVehicleBoxes({result.x[i], result.y[i], result.theta[i]}, i);
        // }
      } else {
        ROS_ERROR("search fail");
        return;
      }
    }
    // visPath(astarpath);
    // Visualize the unoptimized route
    visOptPath(astarpath_);
    // Visualize the optimized route
  }

  // Set traffic rules (topology segment)
  void HalfStructPlanner::set_traffic_route_topology(lines_type const& lines) {
    traffic_routes_ = lines;
    // Contains the coordinates of the start and end points, information about which edges this edge can connect to, etc. A traffic_route represents an edge (excluding topology).

    // voronoi_pub.publish(voronoi); // Visualize the Voronoi diagram

    // Save the route
    std::string root_path;
    // Parameter service, value taken from the launch file
    ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
    auto name = ss.str();
    // Save the topology route
    auto dir = root_path + "/topology_path/";
    auto file_path_ = dir + name;

    // Plan for each line
    // First, save all generated paths, then visualize them together
    vector<vector<Vec3d>> astarpath;
    // Unoptimized route set
    vector<vector<Vec3d>> astarpath_;
    // Optimized route set

    // Visualize vis;
    for (const auto& traffic_route_ : traffic_routes_) {
      Vec3d start_pose, goal_pose;
      // Topology start point
      goal_pose[0] = traffic_route_.b.x;
      goal_pose[1] = traffic_route_.b.y;
      goal_pose[2] = traffic_route_.b.theta;

      // For topology
      if (traffic_route_.go_list.empty()) continue;
      for (auto const& line : traffic_route_.go_list) {
        if (line >= traffic_routes_.size()) continue;
        // Topology end point
        start_pose[0] = traffic_routes_[line].a.x;
        start_pose[1] = traffic_routes_[line].a.y;
        start_pose[2] = traffic_routes_[line].a.theta;
        result.theta.clear();
        result.x.clear();
        result.y.clear();
        if (hybrid_astar_ptr->plan(goal_pose, start_pose, result)) {
          vector<Vec3d> path2smooth_;
          path2smooth_.clear();
          // Discard topology endpoints, as they are included in the direct connection segment
          for (int i = 1; i < result.x.size() - 1; ++i) {
            path2smooth_.push_back({result.x[i], result.y[i], result.theta[i]});
            // Load the path to be smoothed
          }
          astarpath.push_back(path2smooth_);
          // Save the unoptimized route
          // l(Limited-memory)-bfgs smoothing
          smoother_ptr->optimize(voronoiDiagram, path2smooth_, grid_);
          // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_);
          // This is the gradient descent version

          vector<Vec3d> smooth_path_;
          smooth_path_.clear();
          smoother_ptr->getSmoothPath(smooth_path_);

          write_file(file_path_, smooth_path_, result.total_length);
          // Write the optimized topology route to a file

          astarpath_.push_back(smooth_path_);
          // Save the optimized route
        } else {
          ROS_ERROR("search fail");
          return;
        }
      }
    }
    visPath(astarpath);
    visOptPath(astarpath_);
  }

  // Get the pose points on the path from the start to the end point
  auto HalfStructPlanner::get_path(geometry_msgs::PoseStamped const& start,
                                  geometry_msgs::PoseStamped const& goal) -> std::vector<geometry_msgs::PoseStamped> {
    // std::vector<Eigen::Vector3d> result_{}; // Save one of the topology segment or direct connection segment
    // std::vector<Eigen::Vector3d>* result_ = new std::vector<Eigen::Vector3d>();
    // Save one of the topology segment or direct connection segment
    // std::vector<Eigen::Vector3d> result_plo{};
    // Always save the topology part
    // std::vector<Eigen::Vector3d>* result_plo = new std::vector<Eigen::Vector3d>();
    std::vector<geometry_msgs::PoseStamped> result_p{};
    // Topology route represented by poses

    std::unique_ptr<std::vector<Eigen::Vector3d>> result_ = std::make_unique<std::vector<Eigen::Vector3d>>();
    std::unique_ptr<std::vector<Eigen::Vector3d>> result_plo = std::make_unique<std::vector<Eigen::Vector3d>>();

    if (!is_traffic_plan()) return result_p;
    // No traffic rules have been set

    for (auto i = 0; i < node_num_; ++i) {
      for (auto j = 0; j < node_num_; ++j) {
        graph_[i][j] = graph_bk_[i][j];
        // Cost matrix
      }
    }

    ROS_INFO("4");
    // 0 corresponds to the start point; 1 corresponds to the end point
    nodes_[0] = start;
    nodes_[1] = goal;
    calculate_graph();
    // Update the graph topology and cost values, adding the topology after the car's start and end points, connecting the start point to its node, and the end point to its node
    // node_list stores the waypoint indices
    ROS_INFO("6");
    auto node_list = dijkstra();
    // Use Dijkstra's algorithm for planning to get a path. The idea is based on the distance cost value, and since only the cost on the road network is considered, the route is always on the road network.
    node_list.emplace(node_list.begin(), 0);
    // Insert the start point
    // ROS_INFO("5");
    nav_msgs::Path path;
    path.header.frame_id = "map";
    // Parent frame
    path.header.stamp = ros::Time::now();

    double l_s = 0., l_p;
    // Output consecutive elements
    for (int i = 0; i < node_list.size() - 1; ++i) {
      // Total length of the topology
      // Use the mapping table
      try {
        std::pair<int, int> inputKey = {node_list[i], node_list[i + 1]};
        // Retrieve all poses based on the node indices to form a complete path
        const std::vector<Eigen::Vector3d>* sgPathPtr = mappingTable[inputKey];
        // Get the pointer
        // Directly use the pointer
        result_plo->insert(result_plo->end(), sgPathPtr->begin(), sgPathPtr->end());
        // Append SG_paths to the end of result_plo

        // const std::vector<Eigen::Vector3d>& sgPathRef = *sgPathPtr;
        // Get a reference to the vector data pointed to by the pointer
        // result_plo.insert(result_plo.end(), sgPathRef.begin(), sgPathRef.end());
        // Append SG_paths to the end of result_plo

      } catch (const std::bad_alloc& ba) {
        // Catch std::bad_alloc exception
        std::cerr << "Caught a bad_alloc exception: " << ba.what() << '\n';
        // Here you can take some recovery measures, such as releasing other memory, notifying the user, etc.
      }
      l_p += graph_[node_list[i]][node_list[i + 1]];
      // Accumulate the lengths of each segment
    }

    ROS_INFO("1");
    // Create a copy to preserve the original data in result_plo
    std::vector<Eigen::Vector3d> result_plo_copy = *result_plo;
    // Use move semantics to transfer the contents of result_plo to result_
    *result_ = std::move(*result_plo);
    // Check the straight-line distance between the start and end points. If it is less than a threshold, skip the separate HA* planning.
    if (distance(start, goal) < 15. && line_safe(start, goal)) {
      // Allow direct connection between the start and end points. Later, the length is obtained through planning, and the shortest route is determined by comparing it with the shortest route obtained through Dijkstra's search.
      Vec3d start_pose = Eigen::Vector3d{nodes_[0].pose.position.x, nodes_[0].pose.position.y, tf2::getYaw(nodes_[0].pose.orientation)}; // Start point
      Vec3d goal_pose = Eigen::Vector3d{nodes_[1].pose.position.x, nodes_[1].pose.position.y, tf2::getYaw(nodes_[1].pose.orientation)}; // End point
      result.theta.clear();
      result.x.clear();
      result.y.clear();
      vector<Vec3d> smooth_path;
      if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
        vector<Vec3d> path2smooth;
        path2smooth.clear();
        for (int i = 0; i < result.x.size(); ++i) {
          path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});
          // Load the path to be smoothed
        }
        // l(Limited-memory)-bfgs smoothing
        smoother_ptr->optimize(voronoiDiagram, path2smooth, grid_);

        smooth_path.clear();
        smoother_ptr->getSmoothPath(smooth_path);
      }
      l_s = result.total_length;
      // Total length of the direct connection
      // Take the direct connection
      if (l_p > 3 * l_s) {
        result_->clear();
        for (const auto& vec : smooth_path) {
          result_->push_back(Eigen::Vector3d(vec.x(), vec.y(), vec.z()));
        }
      }
    }

    ROS_INFO("3");
    for (auto const& node : *result_) {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      // p.header.stamp
      p.pose.position.x = node[0];
      p.pose.position.y = node[1];
      auto o = node[2];
      p.pose.orientation.z = std::sin(o / 2);
      // Orientation
      p.pose.orientation.w = std::cos(o / 2);
      result_p.emplace_back(p);
    }
    path.poses = result_p;
    ROS_INFO("2");

    visfullPath(result_plo_copy);
    // Display the poses in red, mainly to distinguish between direct connection and topology connection
    res_pub_.publish(path);
    // Publish the path
    // Display the waypoint indices on this path
    show_graph_(node_list);

    ContactS_paths.clear();
    ContactG_paths.clear();
    ContactS_paths_.clear();
    ContactG_paths_.clear();
    ContactSG_paths.clear();
    return result_p;
  }

  // Check if traffic rules have been set
  auto HalfStructPlanner::is_traffic_plan() -> bool {
    return !traffic_routes_.empty();
  }

  // Visualize traffic rules (road network)
  void HalfStructPlanner::show_traffic_route() {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    // Marker ID
    int serial = 0;
    // Road network text serial number
    geometry_msgs::Point p1, p2;
    // Represent the start and end coordinates of the arrow marker
    geometry_msgs::Pose p;
    // Represent the pose of the text marker
    /* This code is a C++ function used to generate a MarkerArray containing arrow markers. MarkerArray is a message type in ROS used to display multiple markers in visualization tools.
      In this code, a MarkerArray object marker_array is first defined, along with two integer variables id and serial.
      Then, two geometry_msgs::Point type variables p1 and p2 are defined, representing the start and end coordinates of the arrow marker, respectively.
      Using a lambda expression from C++11, an anonymous function named make_arrow_marker is defined. This function takes four parameters: r, g, b represent the values of the red, green, and blue color channels, and p1 and p2 represent the start and end coordinates of the arrow marker.
      Inside the function, a visualization_msgs::Marker object marker is created, and its various attributes are set, such as the marker type as ARROW, the action as ADD, the scale as 0.08x0.2x0.4, the color transparency as 0.5, the color values as the input r, g, b, and the pose orientation as the default value.
      Then, the size of the marker's points attribute is adjusted to 2, and the start and end coordinates are set to the input p1 and p2.
      Finally, the marker's namespace (ns) is set to "arrow", and the id is incremented and assigned to marker.id. The marker is added to marker_array.markers.
      The purpose of this code is to generate a Marker with an arrow and add it to the MarkerArray. In ROS, MarkerArray can be used to publish multiple markers together, allowing multiple markers to be displayed simultaneously in visualization tools. */
    // Display arrows
    // [&] is a capture list, indicating that all external variables are captured by reference. This means that all variables in the context can be directly accessed inside the lambda function body.
    auto make_arrow_marker = [&](double r, double g, double b, geometry_msgs::Point const& p1, geometry_msgs::Point const& p2) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::ARROW;
      // Arrow
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
      // Arrow start and end coordinates
      marker.points[0] = p1;
      marker.points[1] = p2;
      marker_array.markers.emplace_back(marker);
    };
    /* This code is another lambda expression in the C++ function, used to create a Marker containing a text marker. Similar to the previous function, this function takes four parameters: red, green, blue channel values, and a geometry_msgs::Pose type variable p, representing the pose of the text marker.
      Inside the function, a visualization_msgs::Marker object named marker is first created, and its various attributes are set. The type value is TEXT_VIEW_FACING, indicating that the text marker will always face the camera, and the action value is ADD, indicating the addition of a new marker. The scale.z value of this marker is 1.0, representing the height of the text marker as 1 unit.
      Next, the marker's color transparency is set to 0.5, and the color values are set to the input r, g, b. The marker's pose is set to the input p. The namespace ns is named "text", and the id value is incremented and assigned to marker.id. Finally, the text content is set to marker.text by converting serial++ to a string.
      The marker object is then added to the marker_array.markers container. This function generates a new Marker containing a text marker and adds it to the MarkerArray. When viewing the MarkerArray in ROS visualization tools, multiple markers, including arrow markers and text markers, will be displayed simultaneously. */
    // Display numeric labels, representing waypoint numbers
    auto make_text_marker = [&](double r, double g, double b, geometry_msgs::Pose const& p) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.3;
      // Set the line width: 0.3m
      marker.scale.y = 0.3;
      marker.scale.z = 1.0;
      marker.color.a = 0.5;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.pose = p;
      marker.ns = "text";
      marker.id = id++;
      marker.text = std::to_string(serial++);
      marker_array.markers.emplace_back(marker);
    };
    // Visualization of the road network topology (from one edge to another)

    /* This code is a range-based for loop used to iterate over the elements in a container named traffic_routes_ (traffic_routes_ = lines;).
      In each iteration of the loop, line is a reference to the current element in traffic_routes_, and you can access the value or attributes of the element through line.
      The loop automatically iterates over all elements in traffic_routes_ and executes the code block in the loop body in sequence. In each iteration, you can use line to manipulate the current element.
      This loop syntax makes traversing the elements of a container or iterable object more concise and readable. */
    for (auto const& line : traffic_routes_) {
      // traffic_routes_ contains start and end coordinates, road network topology information
      // Start and end coordinates
      p1.x = line.a.x;
      p1.y = line.a.y;
      p2.x = line.b.x;
      p2.y = line.b.y;
      // Text marker coordinates
      p.position.x = (p1.x + p2.x) / 2;
      p.position.y = (p1.y + p2.y) / 2;
      auto o = std::atan2(p2.y - p1.y, p2.x - p2.x);
      // Orientation, represented by quaternion
      p.orientation.z = std::sin(o / 2);
      p.orientation.w = std::cos(o / 2);
      make_arrow_marker(0.0, 1.0, 0.0, p1, p2);
      // Anonymous function, defined above
      make_text_marker(0.0, 0.0, 0.0, p);

      // Using a return statement inside a for loop will cause the function to return immediately, terminating the entire loop. This means that even if the for loop has not iterated over all elements, the execution of the return statement will cause the loop to end prematurely.
      if (line.go_list.empty()) continue;
      // If the road network topology list is empty, continue the loop
      // If there is road network topology, draw from the end of one edge to the start of another route
      for (auto const& node : line.go_list) {
        if (node >= traffic_routes_.size()) continue;
        // If the number of topology edges exceeds the original edges, it is invalid
        // node corresponds to which edge in the original planning route (starting from 0), which is the edge index, and this index starts from 0 from top to bottom in the road network file
        p1.x = traffic_routes_[node].a.x;
        // Here, a represents the start point of the connected edge
        p1.y = traffic_routes_[node].a.y;
        /* All straight-line markers */
        make_arrow_marker(0.0, 0.0, 1.0, p2, p1);
      }
    }
    vis_pub_.publish(marker_array);
    // Publish the road network and its topology for visualization in RViz
  }

  // Visualize traffic rules (road network) without labels
  void HalfStructPlanner::show_traffic_route_() {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    // Marker ID
    int serial = 0;
    // Road network text serial number
    int seq = 0;
    // Determine if it is the last point
    bool flag;
    // Arc orientation
    geometry_msgs::Point p1, p2;
    // Represent the start and end coordinates of the arrow marker
    geometry_msgs::Pose p;
    // Represent the pose of the text marker
    /* This code is a C++ function used to generate a MarkerArray containing arrow markers. MarkerArray is a message type in ROS used to display multiple markers in visualization tools.
      In this code, a MarkerArray object marker_array is first defined, along with two integer variables id and serial.
      Then, two geometry_msgs::Point type variables p1 and p2 are defined, representing the start and end coordinates of the arrow marker, respectively.
      Using a lambda expression from C++11, an anonymous function named make_arrow_marker is defined. This function takes four parameters: r, g, b represent the values of the red, green, and blue color channels, and p1 and p2 represent the start and end coordinates of the arrow marker.
      Inside the function, a visualization_msgs::Marker object marker is created, and its various attributes are set, such as the marker type as ARROW, the action as ADD, the scale as 0.08x0.2x0.4, the color transparency as 0.5, the color values as the input r, g, b, and the pose orientation as the default value.
      Then, the size of the marker's points attribute is adjusted to 2, and the start and end coordinates are set to the input p1 and p2.
      Finally, the marker's namespace (ns) is set to "arrow", and the id is incremented and assigned to marker.id. The marker is added to marker_array.markers.
      The purpose of this code is to generate a Marker with an arrow and add it to the MarkerArray. In ROS, MarkerArray can be used to publish multiple markers together, allowing multiple markers to be displayed simultaneously in visualization tools. */
    // Display arrows
    // [&] is a capture list, indicating that all external variables are captured by reference. This means that all variables in the context can be directly accessed inside the lambda function body.
    auto make_arrow_marker = [&](double r, double g, double b, geometry_msgs::Point const& p1, geometry_msgs::Point const& p2) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::ARROW;
      // Arrow
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.08;
      marker.scale.y = 0.45;
      marker.scale.z = 0.8;
      marker.color.a = 0.5;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.pose.orientation.w = 1.0;
      marker.points.resize(2);
      marker.ns = "arrow";
      marker.id = id++;
      // Arrow start and end coordinates
      marker.points[0] = p1;
      marker.points[1] = p2;
      marker_array.markers.emplace_back(marker);
    };
    // Visualization of the road network topology (from one edge to another) – arc markers

    /* This code is a range-based for loop used to iterate over the elements in a container named traffic_routes_ (traffic_routes_ = lines;).
      In each iteration of the loop, line is a reference to the current element in traffic_routes_, and you can access the value or attributes of the element through line.
      The loop automatically iterates over all elements in traffic_routes_ and executes the code block in the loop body in sequence. In each iteration, you can use line to manipulate the current element.
      This loop syntax makes traversing the elements of a container or iterable object more concise and readable. */
    for (auto const& line : traffic_routes_) {
      // traffic_routes_ contains start and end coordinates, road network topology information
      // Start and end coordinates
      p1.x = line.a.x;
      p1.y = line.a.y;
      p2.x = line.b.x;
      p2.y = line.b.y;
      make_arrow_marker(0.0, 1.0, 0.0, p1, p2);
      // Anonymous function, defined above

      // Using a return statement inside a for loop will cause the function to return immediately, terminating the entire loop. This means that even if the for loop has not iterated over all elements, the execution of the return statement will cause the loop to end prematurely.
      if (line.go_list.empty()) continue;
      // If the road network topology list is empty, continue the loop
      // If there is road network topology, draw from the end of one edge to the start of another route
      for (auto const& node : line.go_list) {
        if (node >= traffic_routes_.size()) continue;
        // If the number of topology edges exceeds the original edges, it is invalid
        // node corresponds to which edge in the original planning route (starting from 0), which is the edge index, and this index starts from 0 from top to bottom in the road network file
        p1.x = traffic_routes_[node].a.x;
        // Here, a represents the start point of the connected edge
        p1.y = traffic_routes_[node].a.y;
        if (std::abs(p1.x - p2.x) == 0) continue;
        // Overlapping points
        // ROS_INFO("node= %d", node);
        auto o = std::atan2(p2.y - p1.y, p2.x - p2.x);
        // Orientation, represented by quaternion
        p.orientation.z = std::sin(o / 2);
        p.orientation.w = std::cos(o / 2);
      }
    }
    vis_pub_.publish(marker_array);
    // Visualization in RViz
  }

  // Preprocess the graph and calculate costs
  void HalfStructPlanner::calculate_pre_graph(lines_type const& lines, vector<double> const& distance_table) {
    // traffic_routes_ = lines;
    // Contains start and end coordinates, information about which edges this edge can connect to, etc. A traffic_route represents an edge.
    // Use std::move to transfer the contents of lines to traffic_routes_
    traffic_routes_ = std::move(lines);
    auto num = traffic_routes_.size();
    // Number of start and end edges

    // No traffic rules have been planned yet
    if (traffic_routes_.empty()) {
      ROS_WARN("Receive empty traffic routes, planner not working!");
      return;
    }
    /* static_cast is a type conversion operator in C++. It is used to perform static type conversion, converting an expression to the specified type.
      Using static_cast for type conversion allows type checking at compile time and helps improve code readability and safety. It can be used for conversions between various standard types, including integer, floating-point, pointer, and reference types.
      In the given code, static_cast is used to convert the result of the expression traffic_routes_.size() * 2 + 2 + EXTRA_POINTS_NUM * 2 from a numeric type to an integer. This is because node_num_ is an integer variable, and the calculation result needs to be converted to an integer before assignment.
      It is important to note that static_cast does not perform runtime checks during type conversion. It assumes that the developer has ensured the safety of the conversion. Therefore, when using static_cast for type conversion, ensure that the conversion is reasonable and safe to avoid potential errors or undefined behavior. */
    // Number of start and end points in the road network (traffic_routes_.size()) * 2 (each start point is also an end point) + start and end points of the car (2) + new nodes introduced by the car's start and end points (EXTRA_POINTS_NUM * 2)
    node_num_ = static_cast<int>(traffic_routes_.size() * 2 + 2 + EXTRA_POINTS_NUM * 2);
    // Index meaning:
    // 0: Start point 1: End point
    // 2 ~ 1 + EXTRA_POINTS_NUM: New nodes introduced by the start point
    // 2 + EXTRA_POINTS_NUM ~ 1 + 2 * EXTRA_POINTS_NUM: New nodes introduced by the end point
    // 2 + 2 * EXTRA_POINTS_NUM ~ end: Traffic route nodes (processed by this function)
    nodes_.resize(node_num_);
    // Set the number of waypoints to node_num_, and path planning will be performed between these points
    /* This line of code creates a pointer graph_ of type double* and allocates a dynamic array of size node_num_ to store elements of type double*.
      Specifically, graph_ is a pointer to an array of pointers. By using the new operator, we allocate memory on the heap for node_num_ elements of type double* and assign the address of this memory space to graph_.
      This way, graph_ can be used to store node_num_ pointers of type double*, each of which can point to a double value or an array of double values. This dynamic array creation allows for easy storage and manipulation of data of uncertain size.
      Note that after using graph_, delete[] graph_; should be called to release the previously allocated memory to avoid memory leaks. */
    graph_ = new double*[node_num_];
    graph_bk_ = new double*[node_num_];
    for (auto i = 0; i < node_num_; ++i) {
      /* This line of code creates an array of pointers to double and assigns each pointer to a dynamic array of size node_num_.
        Specifically, graph_ is a pointer to a pointer, and graph_[i] accesses the i-th pointer. By using the new operator, we allocate memory on the heap for each graph_[i] of size node_num_ and assign the address of this array to graph_[i].
        This way, graph_[i] can be used to store an array of size node_num_ of type double, where each element can store a double value.
        Note that after using each dynamic array in graph_, delete[] graph_[i]; should be called to release the previously allocated memory to avoid memory leaks. Finally, delete[] graph_; should be called to release the memory of the graph_ pointer array itself. */
      graph_[i] = new double[node_num_];
      graph_bk_[i] = new double[node_num_];
      for (auto j = 0; j < node_num_; ++j) graph_[i][j] = std::numeric_limits<double>::max();
      // Temporarily set the cost to infinity
      graph_[i][i] = 0.0;
      // The cost from a point to itself is 0
    }

    ROS_INFO("graph_ Init!");
    // Process the preset edges of traffic routes. There are traffic_routes_.size() edges, each with a start and end node.
    for (auto i = 0; i < traffic_routes_.size(); ++i) {
      auto start = 2 + 2 * EXTRA_POINTS_NUM + 2 * i;
      // Start point of a road network (edge)
      auto end = 2 + 2 * EXTRA_POINTS_NUM + 2 * i + 1;
      // End point of a road network (edge)
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      // Parent frame
      auto o = traffic_routes_[i].a.theta;
      p.pose.orientation.z = std::sin(o / 2);
      // Orientation
      p.pose.orientation.w = std::cos(o / 2);
      p.pose.position.x = traffic_routes_[i].a.x;
      // Start point coordinates of the edge
      p.pose.position.y = traffic_routes_[i].a.y;
      nodes_[start] = p;
      // Start node (start point of the edge)
      o = traffic_routes_[i].b.theta;
      p.pose.orientation.z = std::sin(o / 2);
      // Orientation
      p.pose.orientation.w = std::cos(o / 2);
      p.pose.position.x = traffic_routes_[i].b.x;
      // End point coordinates of the edge
      p.pose.position.y = traffic_routes_[i].b.y;
      nodes_[end] = p;
      // End node
      // All traffic routes are straight lines. If arbitrary curves are supported, the curve length needs to be calculated.
      // graph_[][] is a 2D array list storing the cost between pairs of nodes.
      // Here, the length of a road network (edge) is stored as the cost.
      // graph_[start][end] = distance(nodes_[end], nodes_[start]);
      graph_[start][end] = distance_table[i];

      // Fill the mapping table
      // Use a reference to store the reference of SG_paths
      const std::vector<Eigen::Vector3d>& sgPathRef = SG_paths[i];
      mappingTable[{start, end}] = &sgPathRef;
    }

    // ROS_INFO("graph_ Init2!");

    auto j = 0;
    // Process the start points that the end points of traffic routes can connect to and the edges they form.
    for (auto i = 0; i < traffic_routes_.size(); ++i) {
      auto end = 2 + 2 * EXTRA_POINTS_NUM + 2 * i + 1;
      // End point of a road network
      // The end point of the last edge connects to the start point of the first edge, using the straight-line cost.
      for (auto const& node : traffic_routes_[i].go_list) {
        if (node >= traffic_routes_.size()) { // Invalid
          ROS_ERROR("line %d go list %d out of range", i, node);
          continue;
        }
        auto start = 2 + 2 * EXTRA_POINTS_NUM + 2 * node;
        // Start point of the connected edge
        // The end point of a road network can connect to the start point of another road network and the edge they form—length cost.
        // graph_[end][start] = distance(nodes_[end], nodes_[start]);
        graph_[end][start] = distance_table[num + j];

        // Fill the mapping table
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

    show_graph();
    // Display waypoint indices
  }

  // Graph update: Add new start and end points to the topology
  // Graph update: The start point (start of all semi-structured roads) connects to the new start point, and the end point connects to the new end point. If the start and end points are on the same line, check if a direct connection is possible.
  void HalfStructPlanner::calculate_graph() {
    ContactS_paths.clear();
    ContactG_paths.clear();
    ContactS_paths_.clear();
    ContactG_paths_.clear();
    ContactSG_paths.clear();
    // ROS_INFO("25");
    /* When using the std::set container to store setnode objects, the container automatically sorts the elements based on the definition of the operator< function, ensuring that the elements in the container are arranged in ascending order of the nearest_dist attribute. */
    std::set<setnode> start_set, goal_set; // A set of information about the connections from the start point to each node, and from the end point to each node, an ordered set
    auto start = point_t{nodes_[0].pose.position.x, nodes_[0].pose.position.y};
    // Start point
    auto goal = point_t{nodes_[1].pose.position.x, nodes_[1].pose.position.y};
    // End point
    setnode sn{}, gn{}; // {} indicates initialization
    // This loop retrieves all information about the connections from the car's start (end) point to each road network node, including the node coordinates and distance cost.
    // ROS_INFO("35");
    // Instead of brute-force traversal, some data structure methods could be used.
    // However, this full traversal ensures that valid connection points are found.

    for (auto i = 0; i < traffic_routes_.size(); ++i) { 
      // traffic_routes_: current number of road networks, excluding topology
      // Find the connection point from the car's start point to each road network. We see that this connection point only appears on the edges, not on the topology.
      // ROS_INFO("55");
      // Here, the input is still passed by reference
      std::vector<Eigen::Vector3d> const& my_paths = SG_paths[i];
      sn = nearest_point_of_segment(start, &my_paths, 1); // Input the car's start point, the start and end points of the edge (road network)
      // The connection point is the end point of the edge, and the path from the start point to the connection point crosses obstacles.
      // if (sn.nearest_point == traffic_routes_[i].b || !line_safe(start, sn))
      // ROS_INFO("55_");
      // if (sn.nearest_point == traffic_routes_[i].b )
      //   sn.nearest_dist = std::numeric_limits<double>::max();
      //   // Set the distance to infinity as an infinite cost
      /* This code inserts the setnode object named sn into the std::set container named start_set. The insert function adds the given element to the container and automatically sorts and deduplicates the elements based on the container type (here, std::set) and its properties (here, distance).
        If an element equal to sn already exists in start_set (according to the comparison method of the setnode type), the insertion operation is ignored, and no new element is added. If no element equal to sn exists in start_set, sn is added to start_set, maintaining the ordered nature of the container. */
      if (sn.nearest_dist < 100) {
        // Squared value
        sn.line = i;
        // The i-th edge (road network)
        start_set.insert(sn);
      }

      // ROS_INFO("25");
      // Find the connection point from the end point to the edge ab
      gn = nearest_point_of_segment(goal, &my_paths, 0);
      // ROS_INFO("45_");
      // if (gn.nearest_point == traffic_routes_[i].a)
      //   gn.nearest_dist = std::numeric_limits<double>::max();
      if (gn.nearest_dist < 100) {
        gn.line = i;
        goal_set.insert(gn);
      }
      // The connection point does not need to be the start point of the edge. In fact, when the car's start and end points are the same, the connection point for the car's end point is the start point of the edge.
      // if (gn.nearest_point == traffic_routes_[i].a || !line_safe(goal, gn))

      // ROS_INFO("45");
    }

    // ROS_INFO("15");
    /* Next, the code loops based on the value of EXTRA_POINTS_NUM. In each iteration, the code stores the nodes from the start set into nodes_ and updates the corresponding edge weights in the graph. Specifically, the coordinates of the nodes in the start set are stored in nodes_, and the distance between the nodes in the start set and the corresponding edge nodes is stored in the corresponding position in the graph.
      Similarly, the code stores the nodes from the end set into nodes_ and updates the corresponding edge weights in the graph. */
    int line_s, line_g; 
    // Used to store the indices of the start and end points of the current connection point's corresponding edge in the node list nodes_. These indices will be used to update the cost matrix.

    for (auto i = 0; i < EXTRA_POINTS_NUM; ++i) {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      // Store the start point and its connection points from the start set into nodes_. We only store the first few nodes (EXTRA_POINTS_NUM), discarding the rest. The first few nodes are those with smaller edge lengths (already sorted).
      if (i < start_set.size()) {
        // ROS_INFO("start_set.size: %lu", start_set.size());
        auto sen = 2 + i; // nodes_[0] stores the start point, nodes[1] stores the end point, so we skip these two points here.
        line_s = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(start_set.begin(), i)->line;
        // ROS_INFO("line: %d", std::next(start_set.begin(), i)->line);
        line_g = line_s + 1;
        // The index of the end point of the edge where the start point's connection point is located.

        // If the distance from the start point to the connection point is < EXTRA_POINTS_RANGE (3), the first connection point (the one closest to the start point) must be updated, as there must be at least one edge from the start point to the connection point.
        if (i < 1 || std::next(start_set.begin(), i)->nearest_dist < EXTRA_POINTS_RANGE) {

          // Also, plan the route from the start and end points to the connection points.
          Vec3d start_pose = Eigen::Vector3d{nodes_[0].pose.position.x, nodes_[0].pose.position.y, tf2::getYaw(nodes_[0].pose.orientation)}; // Start point
          auto contact = std::next(start_set.begin(), i)->nearest_point;
          Vec3d goal_pose = Eigen::Vector3d{contact.x, contact.y, contact.theta};
          // Connection point
          double lb, lf;
          result.theta.clear();
          result.x.clear();
          result.y.clear();
          vector<Vec3d> smooth_path;
          if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
            vector<Vec3d> path2smooth_b;
            path2smooth_b.clear();
            // The start point's connection point does not include the last point.
            for (int i = 0; i < result.x.size() - 1; ++i) {
              path2smooth_b.push_back({result.x[i], result.y[i], result.theta[i]});
              // Load the path to be smoothed
            }

            // l(Limited-memory)-bfgs smoothing
            smoother_ptr->optimize(voronoiDiagram, path2smooth_b, grid_);
            // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_);
            // This is the gradient descent version.

            smooth_path.clear();
            smoother_ptr->getSmoothPath(smooth_path);

            lb = result.total_length;
          }
          // ROS_INFO("16");
          // contact = std::next(start_set.begin(), i)->nearest_point_f;
          // goal_pose = Eigen::Vector3d {contact.x, contact.y, contact.theta}; // Connection point
          //   result.theta.clear();
          //   result.x.clear();
          //   result.y.clear();
          //   vector<Vec3d> smooth_path_f;
          //   if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
          //       vector<Vec3d> path2smooth_f;
          //       path2smooth_f.clear();
          //       for (int i = 0; i < result.x.size(); ++i) {
          //           path2smooth_f.push_back({result.x[i], result.y[i], result.theta[i]}); // Load the path to be smoothed
          //       }

          //       // l(Limited-memory)-bfgs smoothing
          //       smoother_ptr->optimize(voronoiDiagram, path2smooth_f, grid_);
          //       // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); // This is the gradient descent version.

          //       smooth_path_f.clear();
          //       smoother_ptr->getSmoothPath(smooth_path_f);

          //       lf = result.total_length;
          //   }

          graph_[0][sen] = lb; 
          // Update the distance from the start point [0] to the connection point [sen].
          // auto smooth_path = (lb < lf ? smooth_path_b : smooth_path_f);
          // Take the closest connection point.
          // contact = (lb < lf ? std::next(start_set.begin(), i)->nearest_point_b : std::next(start_set.begin(), i)->nearest_point_f);
          contact = std::next(start_set.begin(), i)->nearest_point;

          auto it = std::next(start_set.begin(), i);
          auto cn = *it;
          // ROS_INFO("cn.line: %d", cn.line);
          // Modify the nearest_index of the element.
          // cn.nearest_index = (lb < lf ? it->nearest_index_b : it->nearest_index_f);
          // ROS_INFO("cn.nearest_index: %d", cn.nearest_index);
          // First, remove the element from the set. Since the set is sorted by path length, reinserting it will place it back in the same position.
          start_set.erase(it);
          // Reinsert the modified element into the set.
          start_set.insert(cn);

          std::vector<Eigen::Vector3d>::const_iterator startIt;
          std::vector<Eigen::Vector3d>::const_iterator endIt;
          startIt = smooth_path.begin();
          endIt = smooth_path.end();
          // ROS_INFO("17");
          ContactS_paths.push_back(std::vector<Eigen::Vector3d>(startIt, endIt));
          // std::vector<Eigen::Vector3d> subVec(endIt - startIt);
          // std::copy(startIt, endIt, subVec.data());

          const std::vector<Eigen::Vector3d>& CSPathRef = ContactS_paths[i];
          // This is the actual curve.
          mappingTable[{0, sen}] = &CSPathRef;

          // Take all poses from the connection point to the end of the edge. This is a straight line.
          // auto v1 = Eigen::Vector3d {nodes_[0].pose.position.x, nodes_[0].pose.position.y, tf2::getYaw(nodes_[0].pose.orientation)};    
          // auto v2 = Eigen::Vector3d {std::next(start_set.begin(), i)->nearest_point.x, std::next(start_set.begin(), i)->nearest_point.y, std::next(start_set.begin(), i)->nearest_point.theta};    
          //           mappingTable[{0, sen}] ={v1, v2};

          // ROS_INFO("18");
          p.pose.position.x = contact.x;
          // Connection point x coordinate.
          p.pose.position.y = contact.y;
          auto o = contact.theta;
          p.pose.orientation.z = std::sin(o / 2);
          p.pose.orientation.w = std::cos(o / 2);
          nodes_[sen] = p;
          // nodes_ index 2+i stores the connection point's position and orientation.
          // ROS_INFO("14");
          // Cost matrix calculation.
          // graph_[0][sen] = std::next(start_set.begin(), i)->nearest_dist; // Update the distance from the start point [0] to the connection point [sen].
          // This value can only be approximated.
          // Most are directly obtained from Dubins curves, with a segment size of 0.3—now all are 0.3.
          // ROS_INFO("graph_[line_s][line_g]: %f", graph_[line_s][line_g]);
          // ROS_INFO("18");

          graph_[sen][line_g] = graph_[line_s][line_g] - static_cast<double>(0.5 * std::next(start_set.begin(), i)->nearest_index); // Update the distance from the connection point to the end point of the edge where the connection point is located.
          // ROS_INFO("11");
          // Indicates which connection point on that edge.
          auto path_pose = SG_paths[std::next(start_set.begin(), i)->line];
          int startIndex = std::next(start_set.begin(), i)->nearest_index;
          // ROS_INFO("startIndex: %d", std::next(start_set.begin(), i)->nearest_index);
          startIt = path_pose.begin() + startIndex + 1;
          endIt = path_pose.end();
          // std::vector<Eigen::Vector3d> subVec_(endIt - startIt);
          // std::copy(startIt, endIt, subVec_.data());

          ContactS_paths_.push_back(std::vector<Eigen::Vector3d>(startIt, endIt));
          const std::vector<Eigen::Vector3d>& CSPathRef_ = ContactS_paths_[i];
          // This is the actual curve.
          mappingTable[{sen, line_g}] = &CSPathRef_;
        }
      }
    }
    // ROS_INFO("12");
    // Similarly, update the end point and its connection points.
    if (i < goal_set.size()) {
      auto gen = 2 + EXTRA_POINTS_NUM + i;
      line_s = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(goal_set.begin(), i)->line;
      // The index of the start point of the edge where the start point's connection point is located.
      // ROS_INFO("line_2: %d", std::next(goal_set.begin(), i)->line);

      if (i < 1 || std::next(goal_set.begin(), i)->nearest_dist < EXTRA_POINTS_RANGE) {

          Vec3d goal_pose = Eigen::Vector3d{nodes_[1].pose.position.x, nodes_[1].pose.position.y, tf2::getYaw(nodes_[1].pose.orientation)}; // End point
          auto contact = std::next(goal_set.begin(), i)->nearest_point;
          Vec3d start_pose = Eigen::Vector3d{contact.x, contact.y, contact.theta};
          // Connection point
          result.theta.clear();
          result.x.clear();
          result.y.clear();
          double lb, lf;
          vector<Vec3d> smooth_path;
          if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
            vector<Vec3d> path2smooth_b;
            path2smooth_b.clear();
            // The end point's connection point does not include the first point.
            for (int i = 1; i < result.x.size(); ++i) {
              path2smooth_b.push_back({result.x[i], result.y[i], result.theta[i]});
              // Load the path to be smoothed
            }

            // l(Limited-memory)-bfgs smoothing
            smoother_ptr->optimize(voronoiDiagram, path2smooth_b, grid_);
            // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_);
            // This is the gradient descent version.

            smooth_path.clear();
            smoother_ptr->getSmoothPath(smooth_path);

            lb = result.total_length;
          }
          // ROS_INFO("22");
          // contact = std::next(goal_set.begin(), i)->nearest_point_f;
          // start_pose = Eigen::Vector3d {contact.x, contact.y, contact.theta}; 
          // // Connection point
          // result.theta.clear();
          // result.x.clear();
          // result.y.clear();
          // vector<Vec3d> smooth_path_f;
          // if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
          //     vector<Vec3d> path2smooth_f;
          //     path2smooth_f.clear();
          //     for (int i = 0; i < result.x.size(); ++i) {
          //         path2smooth_f.push_back({result.x[i], result.y[i], result.theta[i]}); 
          //         // Load the path to be smoothed
          //     }

          //     // l(Limited-memory)-bfgs smoothing
          //     smoother_ptr->optimize(voronoiDiagram, path2smooth_f, grid_);
          //     // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); 
          //     // This is the gradient descent version.

          //     smooth_path_f.clear();
          //     smoother_ptr->getSmoothPath(smooth_path_f);

          //     lf = result.total_length;
          // }

          graph_[gen][1] = lb;
          // Update the distance from the start point [0] to the connection point [sen].
          // auto smooth_path = (lb < lf ? smooth_path_b : smooth_path_f);
          contact = std::next(goal_set.begin(), i)->nearest_point;

          // Modify the value.
          auto it = std::next(goal_set.begin(), i);
          auto cn = *it;
          // Modify the nearest_index of the element.
          // ROS_INFO("cn.line: %d", cn.line);
          // Modify the nearest_index of the element.
          // cn.nearest_index = (lb < lf ? it->nearest_index_b : it->nearest_index_f);
          // ROS_INFO("cn.nearest_index: %d", cn.nearest_index);
          // First, remove the element from the set.
          goal_set.erase(it);
          // Reinsert the modified element into the set.
          goal_set.insert(cn);

          std::vector<Eigen::Vector3d>::const_iterator startIt;
          std::vector<Eigen::Vector3d>::const_iterator endIt;
          startIt = smooth_path.begin();
          endIt = smooth_path.end();
          // std::vector<Eigen::Vector3d> subVec(endIt - startIt);
          // std::copy(startIt, endIt, subVec.data());

          // Take all poses from the connection point to the end of the edge. This is a straight line.
          // auto v1 = Eigen::Vector3d {std::next(goal_set.begin(), i)->nearest_point.x, std::next(goal_set.begin(), i)->nearest_point.y, std::next(goal_set.begin(), i)->nearest_point.theta};    
          // auto v2 = Eigen::Vector3d {nodes_[1].pose.position.x, nodes_[1].pose.position.y, tf2::getYaw(nodes_[1].pose.orientation)};    
          //           mappingTable[{gen, 1}] ={v1, v2};
          ContactG_paths.push_back(std::vector<Eigen::Vector3d>(startIt, endIt));
          // std::vector<Eigen::Vector3d> subVec(endIt - startIt);
          // std::copy(startIt, endIt, subVec.data());

          const std::vector<Eigen::Vector3d>& CSPathRef = ContactG_paths[i];
          // This is the actual curve.
          mappingTable[{gen, 1}] = &CSPathRef;
          // const std::vector<Eigen::Vector3d>& sPathRef = smooth_path;
          // mappingTable[{gen, 1}] = &sPathRef;

          // mappingTable[{gen, 1}] ={subVec};      

          p.pose.position.x = contact.x;
          p.pose.position.y = contact.y;
          auto o = contact.theta;
          p.pose.orientation.z = std::sin(o / 2);
          p.pose.orientation.w = std::cos(o / 2);
          nodes_[gen] = p;
          // graph_[gen][1] = std::next(goal_set.begin(), i)->nearest_dist;
          // Update the cost from the connection point to the car's end point.
          graph_[line_s][gen] = 0.5 * std::next(goal_set.begin(), i)->nearest_index;
          // Update the cost from the start point of the edge where the connection point is located to the connection point.

          // ROS_INFO("graph_[line_s][gen]: %f", graph_[line_s][gen]);         
          // ROS_INFO("nearest_index: %d", std::next(goal_set.begin(), i)->nearest_index);         
          // Indicates which connection point on that edge.
          auto path_pose = SG_paths[std::next(goal_set.begin(), i)->line];
          int startIndex = std::next(goal_set.begin(), i)->nearest_index;
          startIt = path_pose.begin();
          endIt = path_pose.begin() + startIndex;
          // std::vector<Eigen::Vector3d> subVec_(endIt - startIt);
          // std::copy(startIt, endIt, subVec_.data());
          // Take all poses from the connection point to the start of the edge.
          ContactG_paths_.push_back(std::vector<Eigen::Vector3d>(startIt, endIt));
          const std::vector<Eigen::Vector3d>& CGPathRef_ = ContactG_paths_[i];
          // This is the actual curve.
          mappingTable[{line_s, gen}] = &CGPathRef_;
          // ROS_INFO("21");

        }
    }
    

    // Update the cost from the new points introduced by the start point to the new points introduced by the end point, addressing the case where the start and end points are on the same line.
    auto m = 0;
    for (auto i = 0; i < EXTRA_POINTS_NUM; ++i) {
      if (i < start_set.size()) {
        auto sen = 2 + i;
        // Index of the start point's connection point.
        line_s = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(start_set.begin(), i)->line;
        // Index of the start point of the edge where the start point's connection point is located.
        // ROS_INFO("line_3: %d", std::next(start_set.begin(), i)->line);
        line_g = line_s + 1;
        // Index of the end point of the edge where the start point's connection point is located.

        if (std::next(start_set.begin(), i)->nearest_dist < EXTRA_POINTS_RANGE) {

          for (auto j = 0; j < EXTRA_POINTS_NUM; ++j) {
          if (j < goal_set.size()) {
            auto gen = 2 + EXTRA_POINTS_NUM + j;
            // Index of the end point's connection point.
            if (std::next(goal_set.begin(), j)->nearest_dist < EXTRA_POINTS_RANGE) {
              // The two connection points of the start and end points are on the same edge (original road network), and the distance from the end point's connection point to the end of their edge is less than the distance from the start point's connection point to the end of their edge.
              if (std::next(goal_set.begin(), j)->line == std::next(start_set.begin(), i)->line
              && graph_[line_s][line_g] - graph_[line_s][gen] <= graph_[sen][line_g]) {
                  // When the connection points of the start and end points coincide, no cost is added.
                  // ROS_INFO("line_4: %d", std::next(goal_set.begin(), j)->line);
                  graph_[sen][gen] = graph_[sen][line_g] - (graph_[line_s][line_g] - graph_[line_s][gen]);
                  // Use the two connection points of the start and end points as the distance cost. If the above condition is not met, the cost is infinite.
                  // Indicates which connection point on that edge.
                  // ROS_INFO("graph_[line_s][line_g]: %f", graph_[line_s][line_g]);         
                  // ROS_INFO("graph_[line_s][gen]: %f", graph_[line_s][gen]);         
                  // ROS_INFO("graph_[line_s][line_g] - graph_[line_s][gen]: %f", graph_[line_s][line_g] - graph_[line_s][gen]);         
                  // ROS_INFO("graph_[sen][line_g]: %f", graph_[sen][line_g]);
                  // ROS_INFO("graph_[sen][gen]: %f", graph_[sen][gen]);

                  auto path_pose = SG_paths[std::next(goal_set.begin(), j)->line];
                  int startIndex = std::next(start_set.begin(), i)->nearest_index;
                  int endIndex = std::next(goal_set.begin(), j)->nearest_index;
                  // std::vector<Eigen::Vector3d> subVec(path_pose.begin() + startIndex, path_pose.begin() + endIndex -1 );
                  ContactSG_paths.push_back(std::vector<Eigen::Vector3d>(path_pose.begin() + startIndex, path_pose.begin() + endIndex - 1));

                  // ROS_INFO("20");
                  // Take all poses from the connection point to the end of the edge.
                  const std::vector<Eigen::Vector3d>& SGPathRef_ = ContactSG_paths[m++];
                  // This is the actual curve.
                  mappingTable[{sen, gen}] = &SGPathRef_;
                }
              }
            }
          }
        }
      }
    }

    // print_graph(); // Print the cost matrix.
    // show_graph();
    // show_graph(); // Display waypoint indices.
  }

  // display the label of the waypoint
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
      // for (auto j = 0; j < node_num_; ++j) {
      //   if (graph_[i][j] == std::numeric_limits<double>::max()) continue;
      //   p1.x = nodes_[i].pose.position.x;
      //   p1.y = nodes_[i].pose.position.y;
      //   p2.x = nodes_[j].pose.position.x;
      //   p2.y = nodes_[j].pose.position.y;
      //   make_arrow_marker(1.0, 0.0, 0.0, p1, p2);
      // }
    }
    vis_pub_.publish(marker_array);
  }

  // display updated waypoint labels
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
      p.position.x = nodes_[node_list[i]].pose.position.x - 0.5
      // deviation
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
    // Initialize a maximum value.
    int min_index = -1;
    // std::vector<Eigen::Vector3d> const & my_paths = paths;
    for (const auto& path : *paths) {
      // ROS_INFO("y=%f", path[1]);
      // Calculate the distance between two points as the cost.
      dist = static_cast<float>(std::pow(p.x - path[0], 2) + std::pow(p.y - path[1], 2));
      // Squared value.

      // ROS_INFO("66_");
      if (dist < min_dist) {
        min_dist = dist;
        min_index = ii;
      }
      ++ii;
      // Increment the index.
    }

    setnode result{};
    int min_index_;
    if (flag) {
      // Start point = 1
      // To enable smooth planning of the route between the start/end point and the connection point, let the connection point take a point after the nearest connection point.
      if (min_index == 0) {
        min_index_ = min_index;
      } else {
        min_index_ = min_index + 8;
        if (min_index_ >= paths->size() - 1) min_index_ = paths->size() - 1;
      }
      result.nearest_point.x = (*paths)[min_index_][0];
      result.nearest_point.y = (*paths)[min_index_][1];
      // Connection point y-coordinate.
      result.nearest_point.theta = (*paths)[min_index_][2];
      result.nearest_index = min_index_;
    } else {
      // End point = 0
      if (min_index == paths->size() - 1) {
        min_index_ = min_index;
      } else {
        min_index_ = min_index - 8;
        if (min_index_ <= 0) min_index_ = 0;
      }
      result.nearest_point.x = (*paths)[min_index_][0];
      result.nearest_point.y = (*paths)[min_index_][1];
      // Connection point y-coordinate.
      result.nearest_point.theta = (*paths)[min_index_][2];
      result.nearest_index = min_index_;
    }

    result.nearest_dist = min_dist;
    // This value is needed for sorting.
    return result;
  }

  // Calculate the distance between two nodes as the cost.
  auto HalfStructPlanner::distance(point_t const& a, point_t const& b) -> double {
    return std::hypot(a.x - b.x, a.y - b.y);
    // Euclidean distance.
  }

  // Print the cost matrix.
  void HalfStructPlanner::print_graph() {
    std::cout << "node num: " << node_num_ << std::endl;
    /* In each loop, use the std::cout object to output a piece of colored text, and control the field width with std::setw.
      In each loop, std::cout << GREEN sets the color of the subsequent text to green (assuming the relevant ANSI color codes are defined), std::setw(3) controls the output width to 3 characters for alignment, and WHITE resets the output color to the default value. The output format is: i (right-aligned, width of 3) + space.
      This line of code is typically used to display node indices on the command-line interface, making it easier for users to view and understand the relationships between nodes. */
    std::cout << SET_FONT_ON;
    // Start setting the font size.
    std::cout << SET_FONT_1;
    // Set the font size to 3.
    for (auto i = 0; i < node_num_; ++i) std::cout << GREEN << std::setw(3) << i << " " << WHITE;
    std::cout << "\n";
    for (auto i = 0; i < node_num_; ++i) {
      for (auto j = 0; j < node_num_; ++j) {
        if (graph_[i][j] != std::numeric_limits<double>::max()) {
          std::string str{"000 "};
          auto gij = std::to_string(graph_[i][j]);
          for (auto k = 0; k < 3; ++k) {
            if (gij.size() > k) str[k] = gij[k];
            // Replace the first three characters of str with the first three characters of gij.
          }
          std::cout << str.c_str();
        } else {
          std::cout << "*** ";
          // Infinite cost.
        }
      }
      std::cout << GREEN << " " << i << "\n" << WHITE;
    }
    std::cout << SET_FONT_ON;
    // Start setting the font size.
    std::cout << SET_FONT_0;
    // Reset the font size to the default value.
  }

  // Use Dijkstra's algorithm to plan a path with the minimum distance cost.
  auto HalfStructPlanner::dijkstra() -> std::vector<int> {
    std::vector<int> result;
    // Save the path, the final result to be returned.

    struct dijnode {
      int node;
      // Node number.
      int prev_node;
      // The previous node connected to it.
      double cost;
      // Distance cost.
      // Overloaded function for sorting, from small to large.
      bool operator<(dijnode const& rhs) const {
        /* In the operator<() function, the cost member variable of the current object is compared with the cost member variable of the passed object (i.e., the already sorted rhs).
          If the cost of the current object is less than the cost of the passed object, it returns true, indicating that the current object should be placed before the passed object.
          According to the C++ standard library implementation, the priority queue uses operator<() to determine the priority of elements. Therefore, when elements are inserted into the priority queue, they are sorted in ascending order. */
        return cost < rhs.cost;
      }
      bool operator==(dijnode const& rhs) const {
        // Custom equality comparison method, used later in std::find(open_list.begin(), open_list.end(), open_node).
        return node == rhs.node;
      }
    };

    // Find the path from 0-1 (start to end) in the graph_ cost matrix (so only those road networks are passable).
    /* std::set is an ordered associative container where elements are arranged in a certain order, usually from small to large. Here, open_list is a std::set container storing dijnode type objects. Since dijnode can be directly compared in size (defined by the < operator), std::set can automatically maintain the order of elements and quickly perform search, insert, and delete operations.
      std::vector is a dynamic array that can change its size at any time. Here, close_list is a std::vector storing dijnode type objects, used to store already processed nodes, equivalent to a candidate list. After traversing all nodes, nodes already added to close_list no longer need to be considered. */
    std::set<dijnode> open_list;
    std::vector<dijnode> close_list;
    // Nodes that have been processed are placed in close_list.
    open_list.insert(dijnode{0, 0, 0.0});
    // Insert the start point as the initial node into open_list.
    // Continue looping as long as open_list is not empty.
    while (!open_list.empty()) {
      /* This code uses the extract() function of the std::set container and assigns the returned node object.
        The std::set::extract() function is used to extract (remove) an element at a specified position from the set and return a std::set::node_type object containing that element. Here, by calling open_list.extract(open_list.begin()), we extract the first element from open_list and return it. This operation also removes this element from open_list.
        Then, using the .value() member function, we obtain the value of the extracted element and assign it to a variable named node. Thus, node holds the value of the first element previously in open_list.
        In summary, this code extracts and removes the first element from open_list and assigns its value to the variable node. */
      auto node = open_list.extract(open_list.begin()).value();

      // Start building the path here (close_list).
      // Finally, node 1 (end point) is obtained, indicating that open_list has been fully processed.
      if (node.node == 1) {
        while (node.prev_node != 0) {
          // The end point is not connected to the start point (first round), and the node is not connected to the start point (subsequent rounds).
          result.emplace_back(node.node);
          // First, place the end point into the result list (first round), then place the node into the result list (subsequent rounds).
          /* This code uses the STL library's std::find_if() function and a lambda expression to find an element in close_list that satisfies a certain condition.
            The std::find_if() function is used to find the first element in a specified range that satisfies a specific condition and returns an iterator to that element. Here, we pass std::find_if() a starting iterator close_list.begin() and an ending iterator close_list.end() to search for the element that satisfies the condition in the close_list container.
            The lambda expression [](dijnode const& a) { return a.node == node.prev_node; } defines the search condition. Here, [&] means capturing all external variables by reference and passing them into the lambda expression for use within it. dijnode const& a is the parameter list of the lambda function, indicating that it accepts a constant reference parameter named a. { return a.node == node.prev_node; } is the body of the lambda function, used to determine whether the node member of node a is equal to node.prev_node.
            If the search is successful, i.e., an element satisfying the condition is found in close_list, std::find_if() returns an iterator to the matched element; otherwise, it returns the ending iterator close_list.end().
            The prev in the code is an iterator used to store the position (iterator) of the element in close_list that satisfies the condition. If an element satisfying the condition is found, prev will point to that element; otherwise, it will point to close_list.end().
            Note that when using std::find_if() to search for elements, you need to customize the lambda expression to define the search condition based on the specific situation, and ensure that the element type supports the equality comparison operator (==), or a custom equality comparison method is defined. */
          auto prev = std::find_if(close_list.begin(), close_list.end(),
                                  [&](dijnode const& a) { return a.node == node.prev_node; });
                                  // node (1).prev_node is the node before the end point, which generally exists. If it doesn't, the path is incomplete (first round).
          if (prev == close_list.end()) {
            ROS_ERROR("No whole path while graph search success, that should not happen!");
            break;
          }
          node = *prev;
          // Update node to the node before the end point (first round).
        }

        result.emplace_back(node.node);
        // node.node is now the next node after the start point.
        // result.emplace_back(node.prev_node); 
        // No need to specifically go to the start point.
        // Reverse the path to arrange it in the correct order.
        std::reverse(result.begin(), result.end());
        // Turn it into a path from the next node after the start point to the end point.
        // Print.
        std::cout << RED << "0";
        for (auto const& r : result) std::cout << RED << "->" << r;
        // Print the path.
        std::cout << WHITE << std::endl;
        break;
        // Exit the big loop, end.
      }

      // Here, find the shortest path.

      close_list.emplace_back(node);
      // Nodes placed in close_list will no longer be processed.
      // Fill open_list.
      for (auto i = 0; i < node_num_; ++i) {
        if (i == node.node) continue;
        // Same as the current node in open_list.
        /* This code uses the STL library's std::find_if() function and a lambda expression to check if node i has already been processed in close_list.
          The std::find_if() function is used to find the first element in a specified range that satisfies a specific condition and returns an iterator to that element. Here, we pass std::find_if() a starting iterator close_list.begin() and an ending iterator close_list.end() to search for whether node i has already been processed in the close_list container.
          The lambda expression [](dijnode const& a) { return a.node == i; } defines the search condition. Here, [&] means capturing all external variables by reference and passing them into the lambda expression for use within it. dijnode const& a is the parameter list of the lambda function, indicating that it accepts a constant reference parameter named a. { return a.node == i; } is the body of the lambda function, used to determine whether node a is equal to node i.
          If the search is successful, i.e., node i has already been processed in close_list, std::find_if() returns the close_list.end() iterator; otherwise, it returns an iterator to the matched element. If node i has already been processed, the continue statement skips this iteration and continues processing the next node. If node i has not been processed, the following code continues to execute. */
        if (std::find_if(close_list.begin(), close_list.end(),
                        [&](dijnode const& a) { return a.node == i; }) != close_list.end()) continue;
                        // Check if node i has already been processed in close_list. If processed, proceed to the next iteration.

        // Current node i has not been processed yet.

        if (graph_[node.node][i] == std::numeric_limits<double>::max()) continue; // Ignore nodes with infinite cost between them.

        // Current node i and the node with the minimum cost extracted from open_list. graph_[node.node][i]: cost from the previous node to the current node.
        dijnode open_node{i, node.node, node.cost + graph_[node.node][i]};
        // Process the remaining nodes one by one.
        /* This code uses the STL library's std::find() function to search for an element equal to open_node in open_list.
          The std::find() function accepts a starting iterator open_list.begin() and an ending iterator open_list.end() and searches for an element equal to open_node within the specified range. If a matching element is found, it returns an iterator to that element; if no matching element is found, it returns the ending iterator open_list.end().
          Here, open_list is a std::set container, and open_node is an object of type dijnode. Since std::set is an ordered collection, the search operation has a time complexity of O(log n). It performs a binary search based on the size of the elements (via the < operator).
          The iter2 in the code is an iterator used to store the position (iterator) of the element in open_list that is equal to open_node. If a matching element is found, iter2 will point to that element; otherwise, it will point to open_list.end().
          Note that when using std::find() to search for elements, ensure that the element type supports the equality comparison operator (==), or a custom equality comparison method is defined. */
        auto iter2 = std::find(open_list.begin(), open_list.end(), open_node);
        if (iter2 != open_list.end()) {
          // There is an element in open_list equal to open_node.
          // The current node in open_list has a higher cost.
          if (iter2->cost > open_node.cost) {
            open_list.erase(iter2);
            // Remove.
            open_list.insert(open_node);
            // Insert.
          }
        } else {
          // == open_list.end(), no element in open_list is equal to open_node.
          open_list.insert(open_node);
        }
      }
    }

    return result;
  }

  // If the cost value is less than 66, return true, indicating no obstacles.
  // auto HalfStructPlanner::is_free_(const geometry_msgs::PoseStamped& pose) const -> bool {
  //   auto cost = pose_cost_(pose); // Get the cost value of a specific pose.
  //   // ROS_INFO("cost= %d", cost);
  //   return cost < 66;
  // }


  // auto HalfStructPlanner::line_safe(point_t const& a, point_t const& b) -> bool {
  //   // if (!costmap_) return true; // Whether the costmap is used.
  //   // The costmap is used.
  //   auto num = static_cast<int>(distance(a, b) / 0.1); // 0.1 is the map resolution.
  //   uint32_t mx, my;
  //   double x, y;
  //   for (auto i = 0; i < num; ++i) {
  //     x = a.x + i * (b.x - a.x) / num;
  //     y = a.y + i * (b.y - a.y) / num;
  //       /* This code first calls the worldToMap function of the costmap_ object, passing in the x, y parameters and the mx, my reference parameters. The worldToMap function converts the given world coordinates (x, y) into grid coordinates (mx, my) in the costmap.
  //          If the conversion is successful, i.e., the worldToMap function returns true, the condition expression (!costmap_->worldToMap(x, y, mx, my)) evaluates to false, and the code continues execution. */
  //     // if (!costmap_->worldToMap(x, y, mx, my)) return false;
  //       /* This code first calls the getCost function of the costmap_ object, passing in the mx, my parameters. The getCost function retrieves the cost value of the grid cell at coordinates (mx, my) in the costmap.
  //          Then, the code checks if the retrieved cost value is greater than or equal to 66. If it is, the condition expression evaluates to true, and the code executes the return false statement, indicating that the grid cell is unreachable.
  //          Therefore, this code checks whether the given grid coordinates (mx, my) are reachable. If the cost value of the grid cell is greater than or equal to 66, the cell is considered unreachable, and the function returns false; otherwise, the cell is considered reachable, and the code continues execution. */
  //     // if (costmap_->getCost(mx, my) >= 66) return false;
  //   }

  //   return true;
  // }


  /**
  * @brief Check if points on a line are in obstacles (interpolation).
  **/
  // bool HalfStructPlanner::line_safe(point_t const& a, setnode const& b) {
  //     // if (!grid_) return true;

  //     auto check_point_num = static_cast<int>(b.nearest_dist / grid_.info.resolution); // 0.1 is the resolution.

  //     double delta_x = (b.nearest_point.x - a.x) / check_point_num;
  //     double delta_y = (b.nearest_point.y - a.y) / check_point_num;

  //     double cur_x = a.x;
  //     double cur_y = a.y;

  //     // Check if it is an obstacle.
  //     // auto idx = i + j * map_.info.width;: This line calculates the index idx of the given coordinates in the map's data array. Since map data is typically stored in a one-dimensional array, the 2D coordinates need to be converted into a 1D index. The specific calculation method is to multiply j by the map width and add i.
  //     // return map_.data[idx] == 100;: Finally, the function returns a boolean value indicating whether the given coordinates are an obstacle. If the value in the map's data array at idx is 100, the coordinates are considered an obstacle, and the function returns true; otherwise, it returns false.
  //     auto mapObstacle = [&](double x, double y) {
  //         auto i = std::floor((x - grid_.info.origin.position.x) / grid_.info.resolution + 0.5);
  //         auto j = std::floor((y - grid_.info.origin.position.y) / grid_.info.resolution + 0.5);
  //         auto idx = i + j * grid_.info.width;
  //         return grid_.data[idx] == 100 || grid_.data[idx] == -1; // 100 is the obstacle pixel value, -1 is unknown.
  //     };

  //     for (int i = 0; i < check_point_num; ++i) {
  //         // Both obstacle and unknown areas are impassable.
  //         if (mapObstacle(cur_x, cur_y)) return false;
  //         cur_x += delta_x;
  //         cur_y += delta_y;
  //     }
  //     return true;
  // }

  bool HalfStructPlanner::line_safe(point_t const& a, point_t const& b) {
    auto check_point_num = static_cast<int>(std::hypot(a.x - b.x, a.y - b.y) / (grid_->info.resolution * 5));

    double delta_x = (b.x - a.x) / check_point_num;
    double delta_y = (b.y - a.y) / check_point_num;

    double cur_x = a.x;
    double cur_y = a.y;

    // Check if it is an obstacle.
    // auto idx = i + j * map_.info.width;: This line calculates the index idx of the given coordinates in the map's data array. Since map data is typically stored in a one-dimensional array, the 2D coordinates need to be converted into a 1D index. The specific calculation method is to multiply j by the map width and add i.
    // return map_.data[idx] == 100;: Finally, the function returns a boolean value indicating whether the given coordinates are an obstacle. If the value in the map's data array at idx is 100, the coordinates are considered an obstacle, and the function returns true; otherwise, it returns false.
    auto mapObstacle = [&](double x, double y) {
        auto i = std::floor((x - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
        auto j = std::floor((y - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
        auto idx = i + j * grid_->info.width;
        return grid_->data[idx] == 100 || grid_->data[idx] == -1;
        // 100 is the obstacle pixel value, -1 is unknown.
    };

    for (int i = 0; i < check_point_num; ++i) {

        // Vec3i idx = getIndexFromPose({cur_x, cur_y, 0});
        // Both obstacle and unknown areas are impassable.
        if (mapObstacle(cur_x, cur_y)) return false;
        // if (map_.atPosition("elevation", {cur_x, cur_y}) > 0) {
        //     return true;
        // }

        cur_x += delta_x;
        cur_y += delta_y;
    }
    return true;
  }

  /**
  * @brief Visualize the smoothed path.
  * 
  * @param path 
  */
  void HalfStructPlanner::visOptPath(std::vector<vector<Eigen::Vector3d>> paths) {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;
    // Marker ID
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
      // Line width
      marker.color.a = 1.0; 
      // Fully opaque
      marker.color.r = 0.0; 
      marker.color.g = 0.0;
      marker.color.b = 1.0; 
      // Blue color
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
        // Visualize the path
    }
    optimized_path_vis_pub.publish(marker_array);
  }

  /**
  * @brief Visualize the path before smoothing.
  * 
  * @param path 
  */
  void HalfStructPlanner::visPath(std::vector<vector<Eigen::Vector3d>> paths) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker path_point;
    int id = 0;
    // Marker ID
    // Visualize the path
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
      // Line width
      marker.color.a = 1.0; 
      // Fully opaque
      marker.color.r = 0.0; 
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      // Green color
      for (const auto& path : paths) {
          geometry_msgs::Point point;
          point.x = path[0];
          point.y = path[1];
          point.z = 0.05;
          marker.points.push_back(point);
      }
      marker_array.markers.emplace_back(marker);
    };
    // Display waypoints
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
        // Display the path
        path_marker(path);
        // Anonymous function, defined above
        // Display waypoints
        // point_marker(path);
    }
    path_vis_pub.publish(marker_array);
    // Visualize the curve before optimization, the route planned by Hybrid A*.
    // path_point_vis_pub.publish(path_point);
    // Visualize the waypoints of the curve before optimization.
  }

  // Save the optimized path waypoints.
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
      // Receive all pose points of the start and end route.
      // SG_paths = paths;
      // Use std::move to transfer the contents of paths to SG_paths.
      SG_paths = std::move(paths);

      int id = 0; // Marker ID.
      // Create a message containing multiple PoseArrays.
      geometry_msgs::PoseArray multi_pose_array;
      multi_pose_array.header.frame_id = "map";

      auto path_marker = [&](vector<Vec3d> const& paths) {
        // Create a PoseArray message containing multiple poses.
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
          // Visualize the route.
      }
      // Publish the message containing multiple PoseArrays.
      SG_path_vis_pub.publish(multi_pose_array);
  }

  // Display the topological route.
  void HalfStructPlanner::vistopologyPath(const std::vector<vector<Eigen::Vector3d>>& paths) {
      // Receive all pose points of the topological route.
      // topology_paths = paths;
      topology_paths = std::move(paths);

      int id = 0;
      // Marker ID.
      // Create a message containing multiple PoseArrays.
      geometry_msgs::PoseArray multi_pose_array;
      multi_pose_array.header.frame_id = "map";

      auto path_marker = [&](vector<Vec3d> const& paths) {
        // Create a PoseArray message containing multiple poses.
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
          // Visualize the route.
      }
      // Publish the message containing multiple PoseArrays.
      topology_path_vis_pub.publish(multi_pose_array);
  }

  // Display the complete alternative topological route.
  void HalfStructPlanner::visfullPath(const std::vector<Eigen::Vector3d>& poses) {
    // Create a PoseArray message containing multiple poses.
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
        // Visualize the route.
    }
    full_path_vis_pub.publish(pose_array);
  }

  // Hybrid A* planning.
  void HalfStructPlanner::HybridAstarplan(Vec3d start_pose, Vec3d goal_pose) {
    HybridAstarResult result;
    vector<vector<Vec3d>> astarpath;
    // Unoptimized route set.
    vector<vector<Vec3d>> astarpath_;
    // Optimized route set.

    // vector<Vec3d> path2smooth;

    // double path_length = std::numeric_limits<double>::max();
    // Start time of the shortest path planning (with optimization).
    auto hybrid_astar_start_ = chrono::high_resolution_clock::now();
    // Initialization.
    // auto hybrid_astar_end_ = chrono::high_resolution_clock::now();

    // Search 3 times and take the shortest path.
    // for (auto j = 0; j < 3; j++) {
      // Start timing.
      // auto hybrid_astar_start = chrono::high_resolution_clock::now();

      if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
          vector<Vec3d> path2smooth;
          // path2smooth.clear();
          for (int i = 0; i < result.x.size(); ++i) {
              path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});
              // Load the path to be smoothed.
          }

          astarpath.push_back(path2smooth);
          // Save the unoptimized path.
          // Visualize the unoptimized path and waypoints.
          visPath(astarpath); // Visualize the unoptimized route.
          // Visualization of the original A* and polyline A* is placed in the plan file.

          // l(Limited-memory)-bfgs smoothing.
          smoother_ptr->optimize(voronoiDiagram, path2smooth, grid_);
          // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_);
          // This is the gradient descent version.

          vector<Vec3d> smooth_path;
          // smooth_path.clear();
          smoother_ptr->getSmoothPath(smooth_path);
          astarpath_.push_back(smooth_path);
          // Save the optimized path.
          visOptPath(astarpath_);

          // for (size_t i = 0; i < result.x.size(); ++i)
          // {
          //   // Vehicle outline based on the smoothed path points.
          //   // vis.publishVehicleBoxes({smooth_path[i](0), smooth_path[i](1), smooth_path[i](2)}, i);
          //   // Vehicle outline based on the unoptimized path points.
          //   vis.publishVehicleBoxes({result.x[i], result.y[i], result.theta[i]}, i);
          // }

      } else {
          ROS_ERROR("search fail");
          return;
      }
        // End time of planning (with optimization).
        auto hybrid_astar_end = chrono::high_resolution_clock::now();

      // if (path_length > result.total_length) {
      //     path_length = result.total_length;
      //     // Save the timestamp of this iteration.
      //     hybrid_astar_start_ = hybrid_astar_start;
      //     hybrid_astar_end_ = hybrid_astar_end;
      //     path2smooth.clear();
      //     for (int i = 0; i < result.x.size(); ++i) {
      //         path2smooth.push_back({result.x[i], result.y[i], result.theta[i]}); // Load the path to be smoothed.
      //     }
      // }
    // }

    // Visualize the unoptimized path and waypoints.
    // astarpath.push_back(path2smooth); // Save the unoptimized path.
    // visPath(astarpath); // Visualize the unoptimized route.
    // Visualization of the original A* and polyline A* is placed in the plan file.

    chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start_;
    cout << "Shortest path length: " << result.total_length << "m" << endl;
    cout << "Shortest path planning time (with optimization): " << hybrid_astar_use_time.count() * 1000 << "ms" << endl;
  }

}

