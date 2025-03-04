#include "hybrid_astar_dubins_opt.h"


namespace chibot::topologyplanning {

  HybridAstarDubinsOpt::HybridAstarDubinsOpt():
    line_ready_(false),
    has_start(false),
    has_goal(false){}

  void HybridAstarDubinsOpt::init() {
    map_sub_ = nh.subscribe("/map", 1, &HybridAstarDubinsOpt::mapReceived, this);
    start_sub = nh.subscribe("/initialpose", 1, &HybridAstarDubinsOpt::startCallback, this);
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &HybridAstarDubinsOpt::goalCallback, this);
    task_srv_ = nh.advertiseService(TASK_SRV, &HybridAstarDubinsOpt::task_service, this); //Server setup.
    nh.getParam("/hybrid_astar_dubins_opt/Inspection_route_num", route_num);
  }

  void HybridAstarDubinsOpt::run() {
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        // Task state machine, state transitions occur in various events.
        switch (cur_state_) {
            case StateValue::Idle:
                // Plan and save the semi-structured topological network.
                if (line_ready_) {
                    half_struct_planner_->set_traffic_route(lines);
                    ROS_WARN("Path Ready!");
                    half_struct_planner_->set_traffic_route_topology(lines);
                    ROS_WARN("Topological Path Ready!");
                    lines.shrink_to_fit();
                    lines.clear();
                    line_ready_ = false;
                }

                if (path_ready_) {
                    // Visualize the semi-structured topological network.
                    half_struct_planner_->visSGPath(paths);
                    ROS_WARN("visSGPath!");
                    paths.clear();
                    paths.shrink_to_fit();
                    half_struct_planner_->vistopologyPath(paths_);
                    ROS_WARN("vistopologyPath!");
                    paths_.clear();
                    paths_.shrink_to_fit();
                    // Display waypoint indices, set the distance matrix.
                    half_struct_planner_->calculate_pre_graph(lines, distance_table);
                    ROS_WARN("Finish!");
                    lines.clear();
                    lines.shrink_to_fit();
                    distance_table.clear();
                    distance_table.shrink_to_fit();
                    cur_state_ = StateValue::Run;
                    path_ready_ = false;
                }
                break;

            case StateValue::Pause:
                break;

            case StateValue::Record_Traffic_route:
                break;

            case StateValue::Run:
                if (has_goal & has_start) {
                    ROS_WARN("Start!");
                    // Retrieve the route from the semi-structured topological network based on the starting and ending poses.
                    auto poses = half_struct_planner_->get_path(start, goal);
                    // Poses is a set of poses.
                    if (poses.empty()) {
                        ROS_INFO("Traffic path not found!");
                        has_goal = false;
                        has_start = false;
                        return;
                    }
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

  void HybridAstarDubinsOpt::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!msg) {
        ROS_WARN("Received invalid start message");
        return;
    }

    Vec3d start_pose;
    start_pose[0] = msg->pose.pose.position.x;
    start_pose[1] = msg->pose.pose.position.y;
    start_pose[2] = tf2::getYaw(msg->pose.pose.orientation);

    // Describe the starting pose.
    if (cur_state_ == StateValue::Run) {
        start.header = msg->header;
        start.pose.position = msg->pose.pose.position;
        start.pose.orientation = msg->pose.pose.orientation;
        has_start = true;
        vis.publishStartPoses(start_pose);
    }
    // Receive the starting pose, set the topological starting point set.
    else if (cur_state_ == StateValue::Record_Traffic_route) {
        start_pose_set.emplace_back(start_pose);
        vis.publishStartPoses(start_pose);
    }
  }

  void HybridAstarDubinsOpt::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        ROS_WARN("Received invalid goal message");
        return;
    }
    
    Vec3d goal_pose;
    goal_pose[0] = msg->pose.position.x;
    goal_pose[1] = msg->pose.position.y;
    goal_pose[2] = tf2::getYaw(msg->pose.orientation);
    
    // Receive the ending pose, a set.
    if (cur_state_ == StateValue::Record_Traffic_route) {
        goal_pose_set.emplace_back(goal_pose);
        vis.publishGoalPoses(goal_pose);
    }
    // Describe the goal pose.
    else if (cur_state_ == StateValue::Run) {
        goal = *msg;
        has_goal = true;
        vis.publishGoalPoses(goal_pose);
    }
  }

  // Receive map information.
  void HybridAstarDubinsOpt::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg) {
    boost::mutex::scoped_lock lock(map_mutex_);
    // Initialize Semi-structured planning.
    half_struct_planner_ = std::make_shared<HalfStructPlanner>(msg, nh);
    half_struct_planner_->init(nh);
  }

  // Task switching server.
  auto HybridAstarDubinsOpt::task_service(tso_hastar_net::chibot_task::Request& req, tso_hastar_net::chibot_task::Response& resp) -> bool {
    std::string root_path;
    // Parameter service, value taken from the launch file.
    ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
    switch (req.type) {
      case tso_hastar_net::chibot_task::Request::RECORD_TRAFFIC_ROUTE: {
          switch (req.command) {
              case tso_hastar_net::chibot_task::Request::START: {
                  if (cur_state_ != StateValue::Idle) {
                      resp.message = "Not in IDLE status, ignore record command.";
                      return true;
                  }
                  std::stringstream ss;
                  ss << "inspection_traffic_route_" << std::to_string(++route_num);
                  auto name = ss.str();
                  auto dir = root_path + "/topology_map/";
                  map_path_ = dir + name; // Full path.

                  // Loop until the inspection_traffic_route_x file does not exist, meaning it is a new file, then exit the loop.
                  while (check_file_exist(map_path_)) {
                    ROS_WARN("File already exists, will generate new inspection_traffic_route file.");
                    std::stringstream ss;
                    ss << "inspection_traffic_route_" << std::to_string(++route_num);
                    name = ss.str();
                    map_path_ = dir + name;
                  }

                  start_pose_set.clear();
                  goal_pose_set.clear();
                  cur_state_ = StateValue::Record_Traffic_route;
                  resp.message = "Start recording traffic route, save to " + map_path_ + " .";
                  return true;
              }
              case tso_hastar_net::chibot_task::Request::KEEP_TRAFFIC_ROUTE:
              case tso_hastar_net::chibot_task::Request::DISCARD: {
                  if (cur_state_ != StateValue::Record_Traffic_route) {
                      resp.message = "Not in Record Traffic Route status, ignore command.";
                      return true;
                  }
                  cur_state_ = StateValue::Idle;
                  // Discard the current recording.
                  if (req.command == tso_hastar_net::chibot_task::Request::DISCARD) {
                      route_num--;
                      resp.message = "Do not save traffic route, discard recorded data.";
                  } else {
                      // Save the start and end points of the semi-structured road, manually write the topological relationships into the file: end point --> start point of another side.
                      if (!write_traffic_route(map_path_, start_pose_set, goal_pose_set)) {
                          resp.message = "Write file failed.";
                      } else {
                          resp.message = "Save traffic route successful.";
                      }
                  }
                  vis.vis_clear(); // Clear visualization.
                  start_pose_set.clear();
                  goal_pose_set.clear();
                  return true;
              }
              default:
                  resp.message = "Illegal service command.";
                  break;
          }
          return true;
      }

      case tso_hastar_net::chibot_task::Request::LOAD_TRAFFIC_ROUTE: {
          if (cur_state_ != StateValue::Idle) {
              resp.message = "Not in IDLE status, ignore load_traffic_route command.";
              return true;
          }
          auto name = req.map_name;
          if (name == "") {
              resp.message = "Not set traffic route file.";
              return true;
          }
          auto dir = root_path + "/topology_map/";
          map_path_ = dir + name;
          if (!check_file_exist(map_path_)) {
              resp.message = "File does not exist, need record new traffic route.";
              return true;
          }
          if (!read_traffic_route(map_path_, lines)) {
              resp.message = "read file failed.";
          } else {
              resp.message = "read file successful, got " + std::to_string(lines.size()) + " lines";
          }
          line_ready_ = true;
          return true;
      }

      case tso_hastar_net::chibot_task::Request::LOAD_PATH: {
        if (cur_state_ != StateValue::Idle) {
            resp.message = "Not in IDLE status, ignore load_traffic_route command.";
            return true;
        }
        auto name = req.path_name;
        if (name == "") {
            resp.message = "Not set path file.";
            return true;
        }
        auto dir = req.dir;
        if (dir == "") {
            resp.message = "Not set topology path file.";
            return true;
        }
        file_path_ = root_path + "/path/" + name;
        if (!check_file_exist(file_path_)) {
            resp.message = "File does not exist, need record new traffic route.";
            return true;
        }
        if (!read_file(file_path_, paths)) {
            resp.message = "read path file failed.";
            return true;
        } else {
            file_path_ = root_path + "/topology_path/" + dir;
            if (!read_file(file_path_, paths_)) {
                resp.message = "read topology path file failed.";
                return true;
            }
            resp.message = "read path file successful.";

            name = req.map_name;
            if (name == "") {
                resp.message = "Not set topology_map file.";
                return true;
            }
            map_path_ = root_path + "/topology_map/" + name;
            read_traffic_route(map_path_, lines);
            path_ready_ = true;
        }
        return true;
      }

      default: {
        ROS_ERROR("Illegal service type %d", req.type);
        break;
      }
    }
      return true;
  }

  auto HybridAstarDubinsOpt::write_traffic_route(std::string& file_path, std::vector<Vec3d> const& line_start, std::vector<Vec3d> const& line_goal) -> bool {
    if ((line_start.size() + line_goal.size()) % 2 != 0) {
        ROS_ERROR("line points are incomplete, currently not supported.");
        return false;
    }
    std::ofstream out(file_path.c_str(), std::ios_base::out | std::ios_base::app);
    if (!out.is_open()) {
        ROS_ERROR("Open file %s failed!", file_path.c_str());
        return false;
    }
    for (auto i = 0; i < line_start.size(); i++) {
        out << std::to_string(line_start[i][0]) << " " << std::to_string(line_start[i][1]) << " " << std::to_string(line_start[i][2])
            << " ===> "
            << std::to_string(line_goal[i][0]) << " " << std::to_string(line_goal[i][1]) << " " << std::to_string(line_goal[i][2])
            << "\n";
    }
    out.close();
    return true;
  }

  auto HybridAstarDubinsOpt::check_file_exist(std::string& file_path) -> bool {
    std::ifstream exist(file_path.c_str());
    return !!exist;
  }

  auto HybridAstarDubinsOpt::read_traffic_route(std::string& file_path, lines_type& lines) -> bool {
    std::ifstream in(file_path.c_str());
    if (!in.is_open()) {
        ROS_ERROR("Open file %s failed!", file_path.c_str());
        return false;
    }
    line_t line;
    std::string content, temp;
    std::vector<std::string> temps;
    while (getline(in, content)) {
        temps.clear();
        temp.clear();
        for (auto const& c : content) {
            if (c != ' ' && c != '=' && c != '>') {
                temp += c;
            } else if (!temp.empty()) {
                temps.emplace_back(temp);
                temp.clear();
            }
        }
        if (!temp.empty()) temps.emplace_back(temp);
        if (temps.size() < 6) continue;
        line.a.x = std::stod(temps[0]);
        line.a.y = std::stod(temps[1]);
        line.a.theta = std::stod(temps[2]);
        line.b.x = std::stod(temps[3]);
        line.b.y = std::stod(temps[4]);
        line.b.theta = std::stod(temps[5]);
        line.go_list.clear();
        for (auto i = 6; i < temps.size(); ++i) {
            line.go_list.emplace_back(std::stoi(temps[i]));
        }
        lines.emplace_back(line);
    }
    return !lines.empty();
  }

  auto HybridAstarDubinsOpt::read_file(std::string& file_path, vector<vector<Eigen::Vector3d>>& routes) -> bool {
    std::ifstream in(file_path.c_str());
    if (!in.is_open()) {
        ROS_ERROR("Open file %s failed!", file_path.c_str());
        return false;
    }
    std::vector<Eigen::Vector3d> route;
    std::string content, temp;
    std::vector<std::string> temps;
    while (getline(in, content)) {
        if (content == "EOP" && !route.empty()) {
            routes.emplace_back(route);
            route.clear();
            continue;
        }
        temps.clear();
        temp.clear();
        for (auto const& c : content) {
            if (c != ' ') {
                temp += c;
            } else {
                temps.emplace_back(temp);
                temp.clear();
            }
        }
        if (!temp.empty()) temps.emplace_back(temp);
        if (temps.size() != 3 && temps.size() != 1) continue;
        // Use a table to store all distance values.
        if (temps.size() == 1) distance_table.emplace_back(std::stod(temps[0]));
        else {
            Eigen::Vector3d p;
            p[0] = std::stod(temps[0]);
            p[1] = std::stod(temps[1]);
            p[2] = std::stod(temps[2]);
            route.emplace_back(p);
        }
    }
    return !routes.empty();
  }


}
