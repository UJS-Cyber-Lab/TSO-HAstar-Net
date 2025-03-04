#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <fstream>
#include <chrono>
#include <geometry_msgs/Twist.h>

#include "hybrid_astar_searcher/visualize.h"
#include "tso_hastar_net/chibot_task.h"
#include "half_struct_planner.h"


namespace chibot::topologyplanning {
    constexpr static const char* NODE_NAME = "tso_hastar_net";
    constexpr static const char* TASK_SRV = "inspection_task";
    using namespace std;
    using namespace Eigen;
    
    enum class StateValue : uint8_t {
        Idle = 0,
        Record,
        Record_Traffic_route,
        Run,
        Pause
    };

    class HybridAstarDubinsOpt {
        public:
            HybridAstarDubinsOpt();
            ~HybridAstarDubinsOpt() = default;
            void init();
            void run();

        private:
            void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
            void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
            void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
            auto read_traffic_route(std::string& file_path, lines_type& lines) -> bool;
            auto check_file_exist(std::string& file_path) -> bool;
            auto write_traffic_route(std::string& file_path, std::vector<Vec3d> const& line_start, std::vector<Vec3d> const& line_goal) -> bool;
            auto task_service(tso_hastar_net::chibot_task::Request& req, tso_hastar_net::chibot_task::Response& resp) -> bool;
            auto read_file(std::string& file_path, vector<vector<Eigen::Vector3d>>& routes) -> bool;
        
        private:
            ros::NodeHandle nh;
            ros::Subscriber map_sub_;
            ros::Subscriber start_sub;
            ros::Subscriber goal_sub;
            ros::ServiceServer task_srv_;

            std::shared_ptr<HalfStructPlanner> half_struct_planner_;
            bool has_start, has_goal;
            geometry_msgs::PoseStamped start, goal;
            std::vector<Vec3d> start_pose_set;
            std::vector<Vec3d> goal_pose_set;
            boost::mutex map_mutex_;
            StateValue cur_state_;
            int route_num; //inspection_traffic_route_x, x=route_num+1
            Visualize vis;
            std::string map_path_;
            std::string file_path_;
            lines_type lines;
            std::atomic_bool line_ready_, path_ready_;
            vector<vector<Eigen::Vector3d>> paths;
            vector<vector<Eigen::Vector3d>> paths_;
            vector<double> distance_table;

    };
}


