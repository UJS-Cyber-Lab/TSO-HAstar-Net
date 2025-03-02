#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
// nav_msgs/GetMap是map_server提供的服务的消息类型。

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <fstream>
#include <chrono>


#include "hybrid_astar_searcher/visualize.h"
#include "half_struct_planner.h"
#include "hybrid_astar_searcher/hybrid_astar.h"


namespace chibot::topologyplanning {
    // constexpr static const char* NODE_NAME = "hybrid_astar_dubins_opt";
    // constexpr static const char* TASK_SRV = "inspection_task";
    // constexpr static const char* RVIZ_POINT = "/clicked_point";
    // namespace ob = ompl::base;
    // namespace og = ompl::geometric;
    // typedef ompl::base::SE2StateSpace::StateType State;
    using namespace planning;
    using namespace std;

    enum class StateValue : uint8_t {
        Idle = 0,
        Record,
        // 记录示教路径
        Record_Traffic_route,
        // 记录路网
        Run,
        Pause
    };

class SimpleAstarTest {
    public:
        SimpleAstarTest();
        ~SimpleAstarTest() = default;
        void init();
        void run();

    private:
        // void Gridmap_Callback(grid_map_msgs::GridMap msg);
        void startCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
        // void goalCallback(const geometry_msgs::PoseStamped msg);
        void goalCallback(geometry_msgs::PoseStamped::ConstPtr const& msg);
        void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
        // void point_cb(geometry_msgs::PointStampedConstPtr const& msg);
        // auto read_traffic_route(std::string& file_path, lines_type& lines) -> bool;
        // auto check_file_exist(std::string& file_path) -> bool;
        // auto write_traffic_route(std::string& file_path, std::vector<Vec3d> const& line_start, std::vector<Vec3d> const& line_goal) -> bool;
        // auto read_file(std::string& file_path, vector<vector<Eigen::Vector3d>>& routes) -> bool;

    private:
        std::shared_ptr<HalfStructPlanner> half_struct_planner_;
        
        ros::Subscriber map_sub_;
        ros::Subscriber start_sub;
        ros::Subscriber goal_sub;
        ros::Subscriber point_sub_;
        ros::ServiceServer task_srv_;

        /*---------optimize related-------------*/
        ros::Publisher  optimized_traj_pub;

        nav_msgs::Path path;
        geometry_msgs::PoseStamped pose;


        // Eigen::Vector3d start, goal;
        bool has_start, has_goal;

        // Vec3d start_pose, goal_pose;
        geometry_msgs::PoseStamped start, goal;

        // std::vector<Vec3d> start_pose_set;
        // std::vector<Vec3d> goal_pose_set;

        nav_msgs::OccupancyGrid grid_{};
        std::atomic_bool grid_ready_;

        boost::mutex map_mutex_;

        StateValue cur_state_;
        // std::vector<geometry_msgs::PointStamped> record_points_;//录制路点
        // size_t route_num;//map文件的编号 inspection_traffic_route_x

        // std::string map_path_;
        // std::string file_path_;
        Visualize vis;

        // lines_type lines;
        // std::atomic_bool line_ready_, path_ready_;
        // vector<vector<Eigen::Vector3d>> paths;
        // vector<vector<Eigen::Vector3d>> paths_;
        // vector<double> distance_table;

        std::unique_ptr<HybridAstar> hybrid_astar_ptr;
};
}


