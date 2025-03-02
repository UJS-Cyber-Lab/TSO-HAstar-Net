#include "simple_astar_test.h"


namespace chibot::topologyplanning {

  SimpleAstarTest::SimpleAstarTest():
  // line_ready_(false),
  has_start(false),
  has_goal(false),
  cur_state_(StateValue::Run){}

  // Initialization.
  void SimpleAstarTest::init() {
    ros::NodeHandle nh;
    // Subscribe to the grid map. This must be placed in the main program with a callback function, and it should be fast in and fast out.
    map_sub_ = nh.subscribe("/map", 1, &SimpleAstarTest::mapReceived, this);
    // Set the topology start point set.
    start_sub = nh.subscribe("/initialpose", 1, &SimpleAstarTest::startCallback, this);
    // Set the topology end point set.
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &SimpleAstarTest::goalCallback, this);

    // Various service calls, server setup, an important part.
    // task_srv_ = nh.advertiseService(TASK_SRV, &SimpleAstarTest::task_service, this);
  }

  // Main loop function, generates a path given the start and goal points.
  void SimpleAstarTest::run() {
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        // Task state machine, state transitions occur in various events.
        switch (cur_state_) {
            case StateValue::Idle:
                // Do nothing, cancel goal and reset value in event. Canceling navigation and resetting operations are done in the event.

                // if (line_ready_) {
                //   // half_struct_planner_->set_traffic_route(lines); // Plan and save the route between the start and end points.
                //   half_struct_planner_->set_traffic_route_topology(lines); // Plan the topological route.
                //   lines.clear();
                //   ROS_INFO("Ready!");
                //   line_ready_ = false;
                // }

                // if (path_ready_) {
                //   // Visualize the start and end route, green.
                //   half_struct_planner_->visSGPath(paths);
                //   // Visualize the topological route, blue.
                //   half_struct_planner_->vistopologyPath(paths_);
                //   half_struct_planner_->calculate_pre_graph(lines, distance_table); // Display waypoint indices, set the distance matrix.
                //   ROS_INFO("Finish!");
                //   paths.clear();
                //   paths_.clear();
                //   lines.clear();
                //   distance_table.clear();
                //   cur_state_ = StateValue::Run;
                //   path_ready_ = false;
                // }

                break;

            case StateValue::Pause:
                // pause_state();
                break;

            case StateValue::Record_Traffic_route:
                // Record the road network, i.e., the topological relationships.
                break;

            case StateValue::Run:
                // Path following / move to the start of the path / obstacle avoidance, etc.
                // running_state();
                if (has_start && has_goal) {
                    // ROS_INFO("Start!");
                    // Retrieve the path points (a set of points with poses) from the traffic rules based on the start and end points.
                    // auto poses = half_struct_planner_->get_path(start, goal); // poses is a set of points with poses.

                    // Vec2d start_pose{start.pose.position.x, start.pose.position.y};
                    // Vec2d goal_pose{goal.pose.position.x, goal.pose.position.y};
                    // hybrid_astar_ptr->Astarplan(start_pose, goal_pose);

                    Vec3d start_pose = Eigen::Vector3d{start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation)}; // Start point.
                    Vec3d goal_pose = Eigen::Vector3d{goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation)}; // End point.
                    half_struct_planner_->HybridAstarplan(start_pose, goal_pose);

                    // if (poses.empty()) {
                    //   ROS_INFO("Traffic path not found!");
                    //   has_start = false;
                    //   has_goal = false;
                    //   return;
                    // }

                    has_start = false;
                    has_goal = false;
                }
                break;

            default:
                ROS_ERROR("Error StateValue!");
                break;
        }
        rate.sleep();
    }
  }


/**
   * @brief Receive the starting pose, set the topological starting set, and set the start and end points for use
   * 
   * @param msg 
   */
  // void SimpleAstarTest::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
  //     if (cur_state_ != StateValue::Record_Traffic_route) return;
  //     Vec3d start_pose;
  //     start_pose[0] = msg.pose.pose.position.x;
  //     start_pose[1] = msg.pose.pose.position.y;
  //     start_pose[2] = tf2::getYaw(msg.pose.pose.orientation);
  //     // Yaw angle
  //     start_pose_set.emplace_back(start_pose);
  //     // has_start = true;
  //     // ROS_INFO("receive start position");
  //     // Best to visualize
  //     vis.publishStartPoses(start_pose);
  // }


  // Describe the starting pose
  void SimpleAstarTest::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
    // if (cur_state_ != StateValue::Run) return;
    
    Vec3d start_pose;
    
    start_pose[0] = msg.pose.pose.position.x;
    start_pose[1] = msg.pose.pose.position.y;
    start_pose[2] = tf2::getYaw(msg.pose.pose.orientation);
    // Yaw angle
    
    start.header = msg.header;
    start.pose.position = msg.pose.pose.position;
    start.pose.orientation = msg.pose.pose.orientation;
    has_start = true;
    // ROS_INFO("receive start position");

    vis.publishStartPoses(start_pose); // Visualize the starting point
  }


  /**
  * @brief Receive the goal pose, a set used to simulate the end pose of the teaching path
  * 
  * @param msg 
  */
  // void SimpleAstarTest::goalCallback(const geometry_msgs::PoseStamped msg) {
  //     if (cur_state_ != StateValue::Record_Traffic_route) return;
  //     Vec3d goal_pose;
  //     goal_pose[0] = msg.pose.position.x;
  //     goal_pose[1] = msg.pose.position.y;
  //     goal_pose[2] = tf2::getYaw(msg.pose.orientation); // Yaw angle
  //     goal_pose_set.emplace_back(goal_pose);
  //     // has_goal = true;
  //     // ROS_INFO("receive goal position");
  //     vis.publishGoalPoses(goal_pose);
  // }


  // Describe the goal pose
  void SimpleAstarTest::goalCallback(geometry_msgs::PoseStamped::ConstPtr const& msg) {
      // if (cur_state_ != StateValue::Run) return;
      Vec3d goal_pose;
      goal_pose[0] = msg->pose.position.x;
      goal_pose[1] = msg->pose.position.y;
      goal_pose[2] = tf2::getYaw(msg->pose.orientation);
      // Yaw angle
      goal = *msg;

      has_goal = true;
      // ROS_INFO("receive goal position");
      vis.publishGoalPoses(goal_pose);
  }


  // Receive map information
  // msg is a parameter of type const nav_msgs::OccupancyGridConstPtr&, which is a reference to a smart pointer (boost::shared_ptr) pointing to an object of type nav_msgs::OccupancyGrid. This means that msg itself does not own the object it points to, but merely holds a pointer to that object.
  void SimpleAstarTest::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg) {
      // Mutex lock
      // By creating a scoped_lock object lock, the function is locked to ensure thread safety. This means that while the following code is being executed, other threads attempting to access map_mutex_ will be blocked.
      boost::mutex::scoped_lock lock(map_mutex_);
      // grid_ = *msg;
      // The lifecycle of the scoped_lock object lock has ended, and it has left this code block. The lock object will automatically be destructed, thereby releasing the occupancy of map_mutex_, i.e., automatically unlocking.

      // Semi-structured road
      half_struct_planner_ = std::make_shared<HalfStructPlanner>(msg); // Create HalfStructPlanner class
      half_struct_planner_->init(); // Initialize

      // A* and Hybrid * initialization
      // hybrid_astar_ptr.reset(new HybridAstar(grid_));
      ROS_INFO("Map received!");
  }


  // Task switching server, server-side
    // auto SimpleAstarTest::task_service(tso_hastar_net::chibot_task::Request& req, tso_hastar_net::chibot_task::Response& resp) -> bool {
    //   std::string root_path;
    //   // Parameter service, value is taken from the launch file
    //   ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
    //   // First, determine the type input by the client
    //   switch (req.type) {
    //     // Record start and end points
    //     case tso_hastar_net::chibot_task::Request::RECORD_TRAFFIC_ROUTE: {
    //       switch (req.command) {
    //         // Start recording
    //         // Because the command parameter is not set when the service is called, the parameter defaults to 0. To reuse 0, "START" is set here.
    //         case tso_hastar_net::chibot_task::Request::START: {
    //           if (cur_state_ != StateValue::Idle) {
    //             resp.message = "Not in IDLE status, ignore record command.";
    //             return true;
    //           }
    //           // The car is in idle state:
    //           auto name = req.path_name;
    //           if (name == "") { // No road network file name (meaning the parameter is not set when calling the service), assign a default road network file name
    //             std::stringstream ss;
    //             ss << "inspection_traffic_route_" << std::to_string(++route_num); 
    //           // Naming rule
    //             name = ss.str();
    //           }
    //           auto dir = req.dir;
    //           if (dir == "") { 
    //           // Folder name not set
    //             dir = root_path + "/topology_map/"; 
    //           // Map folder
    //           }
    //           map_path_ = dir + name; // Full path

    //           // Loop until the traffic_route file does not exist, meaning it is a new file, then exit the loop
    //           // while (check_file_exist(map_path_)) {
    //           //   resp.message = map_path_ + " already exist, will generate new traffic route file.";
    //           //   std::stringstream ss;
    //           //   ss << "inspection_traffic_route_" << std::to_string(++route_num); 
    //           // Naming rule
    //           //   name = ss.str();
    //           //   map_path_ = dir + name; // New file name
    //           // }

    //           start_pose_set.clear();
    //           goal_pose_set.clear();
    //           cur_state_ = StateValue::Record_Traffic_route; 
    //           // Switch to recording state
    //           resp.message = "Start recording traffic route, save to " + map_path_ + " .";
    //           return true;
    //         }

    //         case tso_hastar_net::chibot_task::Request::KEEP_TRAFFIC_ROUTE: 
    //         // Save road network
    //         case tso_hastar_net::chibot_task::Request::DISCARD: { 
    //         // Discard road network
    //           if (cur_state_ != StateValue::Record_Traffic_route) {
    //             resp.message = "Not in Record Traffic Route status, ignore command.";
    //             return true;
    //           }
    //           cur_state_ = StateValue::Idle;
    //           // Discard current recording
    //           if (req.command == tso_hastar_net::chibot_task::Request::DISCARD) {
    //             route_num--; // Continue to use the current file name
    //             resp.message = "Do not save traffic route, discard recorded data.";
    //             // Clear operation at the end
    //           } else {
    //             // Save road network
    //             // Save the start and end points of the semi-structured road, manually write the topological relationship into the file: end point --> start point of the other side
    //             // Write the road points (start and end points of the road network in order into the file), allowing data to be appended
    //             if (!write_traffic_route(map_path_, start_pose_set, goal_pose_set)) {
    //               resp.message = "Write file failed.";
    //             } else {
    //               resp.message = "Save traffic route successful.";
    //             }
    //           }
    //           vis.vis_clear(); // Clear visualization
    //           start_pose_set.clear();
    //           goal_pose_set.clear();
    //           return true;
    //         }
    //         default:
    //           resp.message = "Illegal service command.";
    //           break;
    //       }
    //       return true;
    //     }

    //     // Load traffic rules, then start planning the route
    //     case tso_hastar_net::chibot_task::Request::LOAD_TRAFFIC_ROUTE: {
    //       // This type does not care about the command parameter
    //       if (cur_state_ != StateValue::Idle) {
    //         resp.message = "Not in IDLE status, ignore load_traffic_route command.";
    //         return true;
    //       }
    //       // The car is in idle state:
    //       auto name = req.path_name;
    //       if (name == "") { 
    //       // Road network file name not specified (meaning the parameter is not set when calling the service)
    //         resp.message = "Not set traffic route file.";
    //         return true;
    //       }
    //       auto dir = req.dir;
    //       if (dir == "") { 
    //       // Folder name not set
    //         dir = root_path + "/topology_map/"; 
    //       // Map folder
    //       }
    //       map_path_ = dir + name; 
    //       // Full path

    //       // Need to check file name duplication
    //       if (!check_file_exist(map_path_)) { 
    //       // File does not exist
    //         resp.message = "File does not exist, need record new traffic route.";
    //         return true;
    //       }

    //       if (!read_traffic_route(map_path_, lines)) { 
    //       // Read road point information from the file into lines, one line is one edge (without topology)
    //         resp.message = "read file failed.";
    //       } else {
    //         resp.message = "read file successful, got " + std::to_string(lines.size()) + " lines";
    //       }

    //       line_ready_ = true;

    //       return true;
    //     }

    //     // Load route
    //     case tso_hastar_net::chibot_task::Request::LOAD_PATH: {
    //       // This type does not care about the command parameter
    //       if (cur_state_ != StateValue::Idle) {
    //         resp.message = "Not in IDLE status, ignore load_traffic_route command.";
    //         return true;
    //       }
    //       // Idle state:
    //       auto name = req.path_name;
    //       if (name == "") { 
    //       // Road network file name not specified (meaning the parameter is not set when calling the service)
    //         resp.message = "Not set path file.";
    //         return true;
    //       }
    //       // Borrow dir to place the topology_path path in topology_path
    //       auto dir = req.dir;
    //       if (dir == "") { 
    //       // Folder name not set
    //         resp.message = "Not set topology path file.";
    //         return true;
    //       }
    //       file_path_ = root_path + "/path/" + name; 
    //       // Full path

    //       // Need to check file name duplication
    //       if (!check_file_exist(file_path_)) { 
    //       // File does not exist
    //         resp.message = "File does not exist, need record new traffic route.";
    //         return true;
    //       }

    //       if (!read_file(file_path_, paths)) { 
    //       // Read road point information from the file into lines, one line is one edge (without topology)
    //         resp.message = "read path file failed.";
    //         return true;
    //       } else {
    //         file_path_ = root_path + "/topology_path/" + dir; // Full path of topology_path
    //         if (!read_file(file_path_, paths_)) { 
    //         // Read road point information from the file into lines, with topology
    //           resp.message = "read topology path file failed.";
    //           return true;
    //         } 
    //         resp.message = "read path file successful.";

    //         map_path_ = root_path + "/topology_map/" + "inspection_traffic_route_2"; 
    //         // Full path and topological relationship
    //         read_traffic_route(map_path_, lines);
        
    //         // Visualize in the main loop
    //         path_ready_ = true;
    //       }



    //       return true;
    //     }

    //     default: {
    //       ROS_ERROR("Illegal service type %d", req.type);
    //       break;
    //     }
    //   }

    //   return true;
    // }



  // Save the stored waypoints to a file
    // auto SimpleAstarTest::write_traffic_route(std::string& file_path, std::vector<Vec3d> const& line_start, std::vector<Vec3d> const& line_goal) -> bool {
    //   if ( (line_start.size()+line_goal.size()) % 2 != 0) {
    //     ROS_ERROR("line points is record, currently not support.");
    //     return false;
    //   }
    //   std::ofstream out(file_path.c_str(), std::ios_base::out | std::ios_base::app);
    //   if (!out.is_open()) {
    //     ROS_ERROR("Open file %s failed!", file_path.c_str());
    //     return false;
    //   }
    //   for (auto i = 0; i < line_start.size(); i ++) {
    //     out << std::to_string(line_start[i][0]) << " " << std::to_string(line_start[i][1]) << " " <<  std::to_string(line_start[i][2])
    //         << " ===> "
    //         << std::to_string(line_goal[i][0]) << " " << std::to_string(line_goal[i][1]) << " " <<  std::to_string(line_goal[i][2])
    //         << "\n";
    //   }

    //   out.close();
    //   return true;
    // }

    // Check if the file exists, accepts a file path as a parameter
    // auto SimpleAstarTest::check_file_exist(std::string& file_path) -> bool {
    // std::ifstream exist(file_path.c_str());
    // return !!exist; // !!exist converts the exist object to a boolean value
    // }


    // Read waypoint information from the file
    // auto SimpleAstarTest::read_traffic_route(std::string& file_path, lines_type& lines) -> bool {
    //   std::ifstream in(file_path.c_str());
    //   if (!in.is_open()) {
    //     ROS_ERROR("Open file %s failed!", file_path.c_str());
    //     return false;
    //   }
    //   line_t line;
    //   std::string contend, temp;
    //   std::vector<std::string> temps;
    //   while (getline(in, contend)) {
    //     temps.clear();
    //     temp.clear();
    //     for (auto const& c : contend) {
    //       if (c != ' ' && c != '=' && c != '>') {
    //         temp += c;
    //       } else if (!temp.empty()) {
    //         temps.emplace_back(temp);
    //         temp.clear();
    //       }
    //     }
    //     if (!temp.empty()) temps.emplace_back(temp);
    //     if (temps.size() < 6) continue;
    //     line.a.x = std::stod(temps[0]);
    //     line.a.y = std::stod(temps[1]);
    //     line.a.theta = std::stod(temps[2]);
    //     line.b.x = std::stod(temps[3]);
    //     line.b.y = std::stod(temps[4]);
    //     line.b.theta = std::stod(temps[5]);
    //     line.go_list.clear();
    //     for (auto i = 6; i < temps.size(); ++i) {
    //       line.go_list.emplace_back(std::stoi(temps[i]));
    //     }
    //     lines.emplace_back(line);
    //   }

    //   return !lines.empty();
    // }


    // Read the route
    // auto SimpleAstarTest::read_file(std::string& file_path, vector<vector<Eigen::Vector3d>>& routes) -> bool {
    //   std::ifstream in(file_path.c_str());
    //   if (!in.is_open()) {
    //     ROS_ERROR("Open file %s failed!", file_path.c_str());
    //     return false;
    //   }
      
    //   std::vector<Eigen::Vector3d> route;
      
    //   std::string contend, temp;
    //   std::vector<std::string> temps;
    //   while (getline(in, contend)) {
    //     if (contend == "EOP" && !route.empty()) {
    //       routes.emplace_back(route);
    //       route.clear();
    //       continue;
    //     }

    //     temps.clear();
    //     temp.clear();
    //     for (auto const& c : contend) {
    //       if (c != ' ') {
    //         temp += c;
    //       } else {
    //         temps.emplace_back(temp);
    //         temp.clear();
    //       }
    //     }
    //     if (!temp.empty()) temps.emplace_back(temp);
    //     if (temps.size() != 3 && temps.size()!= 1) continue;

    //     // Use a table to store all distance values
    //     if(temps.size() == 1) distance_table.emplace_back(std::stod(temps[0]));
    //     else{
    //     Eigen::Vector3d p;
    //     p[0] = std::stod(temps[0]);
    //     p[1] = std::stod(temps[1]);
    //     p[2] = std::stod(temps[2]);
    //     route.emplace_back(p);
    //     }
    //   }

    //   return !routes.empty();
    // }



  
}