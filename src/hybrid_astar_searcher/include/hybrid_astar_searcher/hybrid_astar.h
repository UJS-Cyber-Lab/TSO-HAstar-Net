#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <chrono>


#include <nav_msgs/GetMap.h>
// #include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include "hybrid_astar_searcher/type_defs.h" 
#include "hybrid_astar_searcher/calculate_heuristic.h" 
#include "hybrid_astar_searcher/visualize.h"
// #include "hybrid_astar_searcher/ReedsSheppPath.h"
#include "hybrid_astar_searcher/node3d.h"
#include "hybrid_astar_searcher/dubins.h"
#include "hybrid_astar_searcher/collisiondetection.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace planning {

    const float length_ = 0.72;
    const float width_ = 0.58;
    const float min_turn_radius_ = 0.942;
    //tan=L/R
    const float max_steer_angle_ = 20.0;
    const float max_steer_angle_rate_ = 8.552;
    const int steer_ratio_ = 16;
    const float wheel_base_ = 0.48;
    const float front_edge_to_center_ = 0.6;  
    const float back_edge_to_center_ = 0.12; 
    const float left_edge_to_center_ = 0.29;
    const float right_edge_to_center_ = 0.29;
    const float inflation_value = 0.1;
    const int next_node_num_ = 9;
    const float step_size_ = 0.5;

    const float traj_forward_penalty_ = 1.0;
    const float traj_back_penalty_ = 5.0;
    const float traj_gear_switch_penalty_ = 10.0;
    const float traj_steer_penalty_ =0.5;
    const float traj_steer_change_penalty_ = 1.2;

    // const float dubins_radius_ = 0.3;

    struct HybridAstarResult {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> theta;
        double total_length=0.0;
    };

    class HybridAstar
    {
        public:
            HybridAstar(const nav_msgs::OccupancyGridConstPtr& grid, ros::NodeHandle &nh);
            ~HybridAstar();
            bool plan(Vec3d start_pose, Vec3d goal_pose, HybridAstarResult &result);
            Vec3i getIndexFromPose(Vec3d pose);
            void mod2Pi(double &angle);
            bool AnalyticExpansion(std::shared_ptr<Node3d> cur_node);
            std::vector<Vec2d> calculateCarBoundingBox(Vec3d pose);
            bool isLinecollision(double x0, double y0, double x1, double y1);
            bool validityCheck(std::shared_ptr<Node3d> node);
            bool isInMap(double x, double y);
            std::shared_ptr<Node3d> nextNodeGenerator(std::shared_ptr<Node3d> cur_node, int next_node_idx); 
            double TrajCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node);
            void calculateNodeCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node);
            bool getHybridAstarResult(HybridAstarResult &result);

        private:
            //map params
            double x_min_, x_max_, y_min_, y_max_, xy_resolution_, theta_resolution_;
            int map_size_x_, map_size_y_;
            nav_msgs::OccupancyGridConstPtr grid_;
            Visualize vis;
            std::shared_ptr<Node3d> start_node_;
            std::shared_ptr<Node3d> goal_node_;
            std::shared_ptr<Node3d> final_node_;
            struct cmp {
                bool operator() (const std::pair<int, double>& l, 
                                const std::pair<int, double>& r) {
                    return l.second > r.second;                
                }
            };
            std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, cmp> open_pq_;
            std::unordered_map<int, std::shared_ptr<Node3d>> open_set_;
            std::unordered_map<int, std::shared_ptr<Node3d>> close_set_;
            std::unique_ptr<GridSearch> grid_search_ptr_;
            // std::shared_ptr<ReedShepp> reed_shepp_generator_;
            std::shared_ptr<DubinsStateSpace> dubins_generator_;
            double db_length;
            CollisionDetection configurationSpace;
            double dubins_radius_;
    };
} //namespace planning

