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
            double max_kappa_ = 1.0 / 1.5;
            double max_clearance_ = 0.5;
            double max_voronoi_ = max_clearance_;

            double alpha_ = 1;
            double w_obs_ = 1;
            double w_vor_ = 0;
            double w_cur_ = 10;
            double w_smo_ = 10;

            DynamicVoronoi voronoi_;
            int width_;
            int height_;
            nav_msgs::OccupancyGridConstPtr  grid_map_;
            std::vector<Vec3d> path_;
            std::vector<Vec3d> smooth_path_;
            lbfgs::lbfgs_parameter_t lbfgs_params;
            static double costFunction(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g);
            double x_min_, x_max_, y_min_, y_max_, resolution_;

        public:
            Smoother(/* args */);
            ~Smoother();
            //l(Limited-memory)-bfgs smooth.
            double optimize(DynamicVoronoi &voronoi, std::vector<Vec3d> &path, const nav_msgs::OccupancyGridConstPtr& grid_map);
            // Gradient descent version.
            void smoothPath(DynamicVoronoi &voronoi, std::vector<Vec3d> &path, const nav_msgs::OccupancyGridConstPtr& grid_map);
            void getSmoothPath(std::vector<Vec3d> &smooth_path) {smooth_path = smooth_path_;}
            Vec2d calObstacleTerm(Vec2d x);
            Vec2d calSmoothTerm(Vec2d x_p2, Vec2d x_p, Vec2d x_c, Vec2d x_n, Vec2d x_n2);
            bool isInMap(Vec2d x);

    };

} // namespace planning
