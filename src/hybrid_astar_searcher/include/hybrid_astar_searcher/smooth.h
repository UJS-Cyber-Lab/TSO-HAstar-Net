#pragma once

#include <cmath>
#include <vector>
#include <iostream>
#include "hybrid_astar_searcher/dynamicvoronoi.h"
#include "hybrid_astar_searcher/hybrid_astar.h"
#include "hybrid_astar_searcher/type_defs.h"
#include "hybrid_astar_searcher/lbfgs.hpp"
#include <cfloat>
#include <iomanip>


namespace planning
{
    class Smoother
    {
        private:
            //阈值
            // double max_kappa_ = 1.0 / 20;//表示路径的最大曲率。曲率是描述路径弯曲程度的量，该值表示路径允许的最大曲率，即路径在任意一点的曲率不应超过该值
            double max_kappa_ = 1.0 / 1.5;//表示路径的最大曲率。曲率是描述路径弯曲程度的量，该值表示路径允许的最大曲率，即路径在任意一点的曲率不应超过该值
            double max_clearance_ = 0.5;//表示路径的最大离障碍物距离。该值表示路径与障碍物之间应保持的最小距离，以确保路径不会与障碍物发生碰撞
            double max_voronoi_ = max_clearance_;//max_voronoi_ 被设置为max_clearance_，表示Voronoi图的最大值，用于计算路径的代价。Voronoi图是一种图形化的表示方法，用于描述空间中各点到最近障碍物的距离

            // 曲率代价(梯度下降版本)
            double alpha_ = 1;//表示路径曲率的权重系数。该系数用于平衡路径曲率对代价的影响
            // 障碍物代价
            double w_obs_ = 1;//表示障碍物的权重系数。该系数用于平衡路径与障碍物之间的间隔对代价的影响
            // 让路径靠近Voronoi图骨架
            double w_vor_ = 0;//表示Voronoi图的权重系数。该系数用于平衡路径与Voronoi图之间的关系对代价的影响
            // 曲率代价
            // double w_cur_ = 300;//表示路径曲率的变化率的权重系数。该系数用于平衡路径曲率变化率对代价的影响
            double w_cur_ = 10;//表示路径曲率的变化率的权重系数。该系数用于平衡路径曲率变化率对代价的影响
            // 平滑代价
            // double w_smo_ = 1;//表示路径平滑度的权重系数。该系数用于平衡路径平滑度对代价的影响
            double w_smo_ = 10;//表示路径平滑度的权重系数。该系数用于平衡路径平滑度对代价的影响

            DynamicVoronoi voronoi_;
            // voronoi_宽高
            int width_;
            int height_;
            // nav_msgs::OccupancyGrid grid_map_;
            nav_msgs::OccupancyGridConstPtr  grid_map_;

            std::vector<Vec3d> path_;
            std::vector<Vec3d> smooth_path_;

            lbfgs::lbfgs_parameter_t lbfgs_params;
            static double costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g);
            double x_min_, x_max_, y_min_, y_max_, resolution_;

        public:
            Smoother(/* args */);
            ~Smoother();
            //路径优化：l(Limited-memory)-bfgs平滑
            double optimize(DynamicVoronoi &voronoi, std::vector<Vec3d> &path, const nav_msgs::OccupancyGridConstPtr& grid_map);
            //  梯度下降版本,路径优化
            void smoothPath(DynamicVoronoi &voronoi, std::vector<Vec3d> &path, const nav_msgs::OccupancyGridConstPtr& grid_map);
            // 获取平滑路径
            void getSmoothPath(std::vector<Vec3d> &smooth_path) {smooth_path = smooth_path_;}

            Vec2d calObstacleTerm(Vec2d x);
            Vec2d calSmoothTerm(Vec2d x_p2, Vec2d x_p, Vec2d x_c, Vec2d x_n, Vec2d x_n2);
            bool isInMap(Vec2d x);

    };
} // namespace planning






