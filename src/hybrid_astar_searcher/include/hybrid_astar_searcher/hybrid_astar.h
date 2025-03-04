#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <chrono>

// #include <grid_map_core/grid_map_core.hpp>
// #include <grid_map_msgs/GridMap.h>
#include <nav_msgs/GetMap.h> //OccupancyGrid用

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

    // max_steer_angle --- vehicle max steer angle  
    // max_steer_angle_rate -- vehicle max steer rate; how fast can the steering wheel turn
    // min_steer_angle_rate --- vehicle min steer rate;
    // steer_ratio --- ratio between the turn of steering wheel and the turn of wheels
    //车长
    // double length_ = 4.933;
    const float length_ = 0.72;
    //车宽
    // double width_ = 2.11;
    const float width_ = 0.58;
    //最小转弯半径 R，L固定，tan大，R最小
    // double min_turn_radius_ = 5.0538;
    const float min_turn_radius_ = 0.942;//设置了，但其实没用到
    //最大前轮转角：tan=L/R
    // const float max_steer_angle_ = 6.0; /////////////这个值选小一点可以使路径曲率小一点
    // 不能是90度，因为tan
    const float max_steer_angle_ = 20.0; /////////////这个值选小一点可以使路径曲率小一点
    const float max_steer_angle_rate_ = 8.552;
    // double max_steer_angle_rate_ = 8.552;
    const int steer_ratio_ = 16;
    // double steer_ratio_ = 16;
    // 前轮和后轮的距离L，我们没这个东西
    // double wheel_base_ = 2.845;
    const float wheel_base_ = 0.48;
    //中心是后轴中心，我们假设我们的差速车是阿克曼一样的后驱的
    //中心距前边沿距离
    // double front_edge_to_center_ = 3.89;  
    const float front_edge_to_center_ = 0.6;  
    //中心距后边沿距离
    // double back_edge_to_center_ = 1.043; 
    const float back_edge_to_center_ = 0.12; 
    //中心距左边沿距离
    // double left_edge_to_center_ = 1.055;
    const float left_edge_to_center_ = 0.29;
    // double right_edge_to_center_ = 1.055;
    const float right_edge_to_center_ = 0.29;
    //碰撞检测膨胀值
    const float inflation_value = 0.1;

    //6个朝向，仅保留一个后退的，其他5个朝向都是前进的
    // const int next_node_num_ = 6;
    const int next_node_num_ = 9;
    //每个朝向一步走多少m
    const float step_size_ = 0.5;//可以动态改变搜索圆弧的长度？  我们暂设置为定长，因为后续要作为控制点输入到B样条轨迹路线上

    //penalty 惩罚系数
    //取1.0对应真实距离
    const float traj_forward_penalty_ = 1.0;
    const float traj_back_penalty_ = 5.0;
    const float traj_gear_switch_penalty_ = 10.0;//换档
    //改善突然的折角，惩罚值要选好
    // const float traj_steer_penalty_ = 0.1;//大转向角惩罚
    const float traj_steer_penalty_ =0.5;//大转向角惩罚
    // const float traj_steer_change_penalty_ = 0.15; //切换转向角惩罚
    const float traj_steer_change_penalty_ = 1.2; //切换转向角惩罚

    const float dubins_radius_ = 0.3; //dubins曲率半径

    /**
     * @brief 存储混合A星结果的类型
     * 
     */
    struct HybridAstarResult {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> theta;
        double total_length=0.0;
    };

    class HybridAstar
    {
        public:
            // HybridAstar(grid_map::GridMap map);
            HybridAstar(const nav_msgs::OccupancyGridConstPtr& grid);
            ~HybridAstar();

            bool plan(Vec3d start_pose, Vec3d goal_pose, HybridAstarResult &result);
            Vec3i getIndexFromPose(Vec3d pose);
            void mod2Pi(double &angle);

            bool AnalyticExpansion(std::shared_ptr<Node3d> cur_node);
            std::vector<Vec2d> calculateCarBoundingBox(Vec3d pose);
            bool isLinecollision(double x0, double y0, double x1, double y1);
            bool validityCheck(std::shared_ptr<Node3d> node);
            bool isInMap(double x, double y);
            bool isStateValid2(const ob::SpaceInformation *si, const ob::State *state);
            std::shared_ptr<Node3d> nextNodeGenerator(std::shared_ptr<Node3d> cur_node, int next_node_idx); 

            double TrajCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node);
            void calculateNodeCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node);

            bool getHybridAstarResult(HybridAstarResult &result);
            void Astarplan(Vec2d start_pose, Vec2d goal_pose);

        private:
            //map params
            double x_min_, x_max_, y_min_, y_max_, xy_resolution_, theta_resolution_;
            int map_size_x_, map_size_y_;
            // grid_map::GridMap map_;
            // nav_msgs::OccupancyGrid::Ptr grid_;
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
            // open_pq_队列根据节点cost由小到达的顺序排列
            std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, cmp> open_pq_;
            // open_set_记录了所有待遍历和遍历过的节点
            std::unordered_map<int, std::shared_ptr<Node3d>> open_set_;
            // closet_set_记录了所有已经遍历过的节点
            std::unordered_map<int, std::shared_ptr<Node3d>> close_set_;

            std::unique_ptr<GridSearch> grid_search_ptr_;
            // std::shared_ptr<ReedShepp> reed_shepp_generator_;
            std::shared_ptr<DubinsStateSpace> dubins_generator_;
            double db_length;

            CollisionDetection configurationSpace;//碰撞检测类实体，用以检测某个配置是否会发生碰撞

    };
} //namespace planning
