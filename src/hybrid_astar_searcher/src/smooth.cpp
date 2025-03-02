#include "hybrid_astar_searcher/smooth.h"

namespace planning
{
    Smoother::Smoother(/* args */){}
    Smoother::~Smoother(){}

    // cost objective function
    double Smoother::costFunction(void *ptr, 
                                  const Eigen::VectorXd &x, 
                                  Eigen::VectorXd &g)  {
        auto instance = reinterpret_cast<Smoother *>(ptr);
        std::vector<Vec3d> smooth_path = instance->smooth_path_;
        // Path to be optimized
        const int points_num = smooth_path.size() - 4;
        Eigen::Matrix2Xd opt_points;
        // A matrix with 2 rows and x columns
        opt_points.resize(2, smooth_path.size());
        // The first path point
        opt_points(0,0) = smooth_path[0](0);
        opt_points(1,0) = smooth_path[0](1);
        // The second path point
        opt_points(0,1) = smooth_path[1](0);
        opt_points(1,1) = smooth_path[1](1);
        
        // Transpose the first `points_num` elements of `x` and assign them to the part of `opt_points` starting from the third column
        // Operate on a sub-block of the `opt_points` matrix. Specifically, it represents selecting a sub-matrix of 1 row and `points_num` columns starting from row 0 and column 2 of the `opt_points` matrix.
        opt_points.block(0,2,1,points_num) = x.head(points_num).transpose();
        // Transpose the last `points_num` elements of `x` and assign them to the part of `opt_points` starting from the fourth column
        opt_points.block(1,2,1,points_num) = x.tail(points_num).transpose();
        // Assign the x and y coordinates of the third-to-last smoothed path point to the third-to-last column of `opt_points`
        // opt_points.col(smooth_path.size()-3)(0) = smooth_path[smooth_path.size()-3](0);
        // opt_points.col(smooth_path.size()-3)(1) = smooth_path[smooth_path.size()-3](1);
        // Assign the x and y coordinates of the second-to-last smoothed path point to the second-to-last column of `opt_points`
        opt_points.col(smooth_path.size()-2)(0) = smooth_path[smooth_path.size()-2](0);
        opt_points.col(smooth_path.size()-2)(1) = smooth_path[smooth_path.size()-2](1);
        // Assign the x and y coordinates of the last smoothed path point to the last column of `opt_points`
        opt_points.col(smooth_path.size()-1)(0) = smooth_path[smooth_path.size()-1](0);
        opt_points.col(smooth_path.size()-1)(1) = smooth_path[smooth_path.size()-1](1);
        
        Eigen::Matrix2Xd grad;
        grad.resize(2, points_num);
        // Gradient, excluding the first two points and the last three points
        grad.setZero();
        double cost = 0.0;
        // Cost
        
        double max_clearance = instance->max_clearance_;
        // Collision penalty will be applied within this value
        auto grid_resolution = instance->grid_map_->info.resolution;
        // Map resolution
        auto xmin = instance->grid_map_->info.origin.position.x;
        // Minimum x-coordinate of the map
        auto ymin = instance->grid_map_->info.origin.position.y;
        
        // Collision term
        // Suppose we have a point x_i, and the distance to the nearest obstacle is dist2obs, with the maximum allowed distance being max_clearance.
        // We want to move point x_i to a more suitable position while avoiding collisions. To achieve this, we need to calculate the impact of point x_i on the collision cost, i.e., the gradient.
        // std::cout << "calculate collision cost" << std::endl;
        // collision cost
        double collision_cost = 0.0;
        Eigen::Matrix2Xd collision_grad; // Stores the gradient of the collision term
        collision_grad.resize(2, points_num);
        collision_grad.setZero();
        
        // Traverse the columns of the `opt_points` matrix, starting from the 2nd column and ending at the 4th-to-last column
        for (int i = 2; i < opt_points.cols()-2; ++i) {
            Vec2i x_i;
            // Calculate a 2D vector x_i based on the values in the `opt_points` matrix
            x_i(0) = static_cast<int>((opt_points(0, i) - (xmin)) / grid_resolution);
            // Grid index
            x_i(1) = static_cast<int>((opt_points(1, i) - (ymin)) / grid_resolution);
        
            // Calculate the distance to the nearest obstacle dist2obs and the vector from the obstacle to x_i, vec_o2x
            // dist to the closest obstacle    unit m  grid_resolution resolution
            double dist2obs = grid_resolution * instance->voronoi_.getDistance(x_i(0), x_i(1));
            // std::cout << "dist2obs:" << dist2obs << std::endl;
        
            // Calculate the vector from the obstacle to point x_i, vec_o2x
            Vec2d vec_o2x(x_i(0) - instance->voronoi_.data[x_i(0)][x_i(1)].obstX,
                    x_i(1) - instance->voronoi_.data[x_i(0)][x_i(1)].obstY);
            // std::cout << "vec_o2x:" << vec_o2x << std::endl;
            
            // If dist2obs - max_clearance < 0, it indicates a collision, then update the collision cost collision_cost and the collision gradient collision_grad
            if (dist2obs - max_clearance < 0) {
                // Relationship between collision cost and distance: The collision cost is usually proportional to the distance to the nearest obstacle. The closer the distance, the higher the collision cost.
                // Gradient direction: If the point is closer to the obstacle, moving the point in the direction towards the obstacle will increase the collision cost. Therefore, to avoid collisions, we want to move the point away from the obstacle.
                collision_cost += instance->w_obs_ * pow((dist2obs - max_clearance), 2);
                // Cost
                Vec2d gradient;
                // Calculate the gradient and assign it to the corresponding position in collision_grad
                // vec_o2x is the vector from the obstacle to point x_i. Since we want to move the point away from the obstacle, the direction of this vector is used as the gradient direction.
                // The calculated gradient can be used to update the point's position using optimization algorithms (e.g., gradient descent) to minimize the collision cost.
                gradient = instance->w_obs_ * 2 * (dist2obs - max_clearance) / dist2obs * vec_o2x;
                // Gradient
                collision_grad(0, i-2) = gradient(0);
                collision_grad(1, i-2) = gradient(1);
            // Otherwise, if dist2obs - max_clearance >= 0, it indicates no collision, then set collision_cost and collision_grad to 0
            } else {
                collision_cost += 0;
                collision_grad(0, i-2) = 0;
                collision_grad(1, i-2) = 0;
            }
        }
        // After the loop, add the collision cost collision_cost to the total cost cost, and add the collision gradient collision_grad to the total gradient grad
        cost += collision_cost;
        grad += collision_grad;


        // 平滑项：平滑项的目的是使路径更加平滑，避免出现过于尖锐的拐角。
        // std::cout << "calculate smooth cost" << std::endl;
        // smooth cost
        double smooth_cost = 0.0;
        Eigen::Matrix2Xd smooth_grad;
        smooth_grad.resize(2, points_num);
        smooth_grad.setZero();
        //std::cout << opt_points.cols()-1 << std::endl;
        for (int i = 2; i < opt_points.cols()-2; ++i)  {
            Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));
            // 相邻的前两个点
            Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
            Vec2d x_c(opt_points(0, i), opt_points(1, i));
            // 当前点
            Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));
            Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));
            // 相邻的后两个点

            // 平滑项的误差err
            Vec2d err = x_p + x_n - 2* x_c;
            // 当前点与相邻两个点的差值

            smooth_cost += instance->w_smo_ * err.transpose() * err;
            // 平方
            // std::cout << smooth_cost << std::endl;
            
            // 计算平滑项的梯度
            // smooth_grad.col(i-1) = ((-4) * x_p + 8 * x_c - 4 * x_n);
            smooth_grad.col(i-2) = instance->w_smo_ * 2 * (x_p2 - 4 * x_p + 6 * x_c - 4 * x_n + x_n2);
            // 五点差分公式，比较准确地估计二阶导数（即路径的弯曲程度）
        }
        // std::cout << "smooth_grad" << smooth_grad << std::endl;
        cost += smooth_cost;
        grad +=  smooth_grad;
        // std::cout << "grad" << grad << std::endl;


        // 曲率项
        // 曲率代价项是用来惩罚路径上车辆转弯过大或曲率变化过快的部分
        // curvature cost
        double curvature_cost = 0.0;
        Eigen::Matrix2Xd curvature_grad;
        curvature_grad.resize(2, points_num);
        curvature_grad.setZero();
        // std::cout << opt_points.cols()-1 << std::endl;
        // 循环遍历路径上的点：
        for (int i = 2; i < opt_points.cols()-2; ++i)  {
            // 对于每个点，我们需要获取其前两个点（x_p2和x_p），当前点（x_c），以及后两个点（x_n和x_n2）的位置。
            Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));
            // 前两个点
            Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
            Vec2d x_c(opt_points(0, i), opt_points(1, i));
            // 当前点
            Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));
            // 后两个点
            Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));

            // 四段线
            // 计算四段线段的向量差,这些向量表示路径的曲线方向
            Vec2d delta_x_p = x_p - x_p2;
            Vec2d delta_x_c = x_c - x_p;
            Vec2d delta_x_n = x_n - x_c;
            Vec2d delta_x_n2 = x_n2 - x_n;

            // 计算角度差
            // 使用向量的内积和范数计算相邻向量之间的夹角。通过将内积除以向量的范数乘积，我们获得了两个向量之间的夹角的余弦值。然后，使用反余弦函数将余弦值转换为实际角度（delta_phi_p，delta_phi_c和delta_phi_n）。
            // norm()是一种计算向量的范数（或长度）的函数，欧几里得范数，在二维空间中，向量(x,y)的范数等于sqrt(x^2 + y^2)
            if (delta_x_p.norm() > 0 && delta_x_c.norm() > 0 && delta_x_n.norm() > 0 && delta_x_n2.norm() > 0) {
                // 取[-1,1]防止出现nan
                double delta_phi_p = std::acos(std::min(std::max(delta_x_p.dot(delta_x_c) / delta_x_p.norm() / delta_x_c.norm(), -1.0), 1.0));
                double delta_phi_c = std::acos(std::min(std::max(delta_x_c.dot(delta_x_n) / delta_x_c.norm() / delta_x_n.norm(), -1.0), 1.0));
                double delta_phi_n = std::acos(std::min(std::max(delta_x_n.dot(delta_x_n2) / delta_x_n.norm() / delta_x_n2.norm(), -1.0), 1.0));
                // std::cout << delta_x_p.dot(delta_x_c) / delta_x_p.norm() / delta_x_c.norm() << std::endl;
                // std::cout << "delta_phi_p:" << delta_phi_p << std::endl;

                // 计算曲率（kappa）
                // 曲率是路径曲线在某一点处的曲率半径的倒数。我们通过将角度差除以对应向量的长度来计算曲率。得到了三个曲率值，即kappa_p，kappa_c和kappa_n
                // 通过将角度差除以向量长度，我们得到了路径在前一个点处的弧长与弧长对应的角度之间的比例。这可以被视为曲率的近似。
                double kappa_p = delta_phi_p / delta_x_p.norm();//角度差和向量长度的比值来近似曲率
                double kappa_c = delta_phi_c / delta_x_c.norm();
                double kappa_n = delta_phi_n / delta_x_n.norm();

                // 计算代价和梯度
                // 如果当前点的曲率（kappa_c）超过设定的最大曲率（instance->max_kappa_），则需要计算代价和梯度。否则，代价为0，梯度为零向量。
                if (kappa_c > instance->max_kappa_ && kappa_p > 0 && kappa_n > 0) {
                    // &函数：
                    // compute_d_delta_phi函数用于计算角度变化率
                    auto compute_d_delta_phi = [](const double delta_phi) {
                        // 我们首先使用delta_phi计算了曲线与切线之间的角度差。然后，我们使用std::cos()函数计算了该角度的余弦值。最后，我们使用余弦值来计算曲率，即将-1除以根号下（1- cos(delta_phi)的平方）。这将给出曲率半径的倒数，从而得到曲率。
                        return -1.0 / std::sqrt(1.0 - std::pow(std::cos(delta_phi),2));
                    };
                    // compute_orthogonal_complement函数用于计算两个向量的正交补
                    auto compute_orthogonal_complement = [](Vec2d x0, Vec2d x1) {
                        return x0 - x1 * x0.dot(x1) / std::pow(x1.norm(), 2);
                    };

                    // 利用向量运算，我们计算了曲率的导数（d_kappa_p，d_kappa_c和d_kappa_n）。这些导数将用于计算代价项的梯度。
                    // d_delta_phi_p是角度变化率delta_phi_p对时间t的导数
                    double d_delta_phi_p = compute_d_delta_phi(delta_phi_p);
                    Vec2d d_cos_delta_phi_p = compute_orthogonal_complement(delta_x_p, delta_x_c) 
                                            /  delta_x_p.norm() / delta_x_c.norm();
                    // d_kappa_p是曲率kappa_p对时间t的导数
                    Vec2d d_kappa_p = 1.0 / delta_x_p.norm() * d_delta_phi_p *  d_cos_delta_phi_p;
                    Vec2d k_p = 2.0 * (kappa_p - instance->max_kappa_) * d_kappa_p;
                    // std::cout <<  std::pow(std::cos(delta_phi_p),2) << std::endl;
                    // std::cout << "d_delta_phi_p:" << d_delta_phi_p << std::endl;
                    // std::cout << "d_cos_delta_phi_p:" << d_cos_delta_phi_p << std::endl;
                    // std::cout << "d_kappa_p:" << d_kappa_p << std::endl;
                
                    // std::cout << "kp:" << k_p << std::endl;


                    double d_delta_phi_c = compute_d_delta_phi(delta_phi_c);
                    Vec2d d_cos_delta_phi_c = compute_orthogonal_complement(delta_x_n, delta_x_c) 
                                            /  delta_x_c.norm() / delta_x_n.norm()
                                            -compute_orthogonal_complement(delta_x_c, delta_x_n)
                                            / delta_x_c.norm() / delta_x_n.norm();

                    Vec2d d_kappa_c = 1.0 / delta_x_c.norm() * d_delta_phi_c *  d_cos_delta_phi_c 
                                    -delta_phi_c / std::pow(delta_x_c.norm(), 3) * delta_x_c;
                    Vec2d k_c = 2.0 * (kappa_c - instance->max_kappa_) * d_kappa_c;

                    // std::cout << "d_cos_delta_phi_c:" << d_cos_delta_phi_c << std::endl;
                    // std::cout << "k_c:" << k_c << std::endl;

                    double d_delta_phi_n = compute_d_delta_phi(delta_phi_n);
                    Vec2d d_cos_delta_phi_n = -compute_orthogonal_complement(delta_x_n2, delta_x_n) 
                                            /  delta_x_n.norm() / delta_x_n2.norm();
                    Vec2d d_kappa_n = 1.0 / delta_x_n.norm() * d_delta_phi_n *  d_cos_delta_phi_n 
                                    +delta_phi_n / std::pow(delta_x_n.norm(), 3) * delta_x_n;
                    Vec2d k_n = 2.0 * (kappa_n - instance->max_kappa_) * d_kappa_n;
                    // std::cout << "d_cos_delta_phi_n:" << d_cos_delta_phi_n << std::endl;
                    // std::cout << "kn:" << k_n << std::endl;

                    // 最后，我们计算了曲率代价项（curvature_cost）。曲率代价项是超过最大曲率的点的曲率与最大曲率之间的差值的平方乘以权重（instance->w_cur_）的总和。
                    curvature_cost += instance->w_cur_ * std::pow(kappa_c - instance->max_kappa_, 2);
                    // 代价
                    // 曲率代价项的梯度（curvature_grad）是超过最大曲率的点的曲率导数的加权平均
                    curvature_grad.col(i-2) = instance->w_cur_ * (  0.25*k_p +  0.5*k_c + 0.25*k_n );
                    // 梯度

                } else {
                    curvature_cost += 0;
                    curvature_grad.col(i-2) = Vec2d(0, 0);
                }
            }
        }
        // std::cout << "curvature_grad" << curvature_grad << std::endl;
        cost += curvature_cost;
        grad += curvature_grad;


        // voronoi项，为了让路线尽可能在voronoi图上延伸
        // voronoi cost
        // double voronoi_cost = 0.0;
        // Eigen::Matrix2Xd voronoi_grad;
        // voronoi_grad.resize(2, points_num);
        // voronoi_grad.setZero();
        // //std::cout << opt_points.cols()-1 << std::endl;
        // for (int i = 2; i < opt_points.cols()-2; ++i)  {
        //     Vec2i x_i;
        //     x_i(0) = static_cast<int>((opt_points(0, i) - (xmin)) / grid_resolution);
        //     x_i(1) = static_cast<int>((opt_points(1, i) - (ymin)) / grid_resolution);

        //     // 机器人到最近障碍物的距离
        //     //dist to the closet obstacle    单位：unit m
        //     double dist2obs = grid_resolution * instance->voronoi_.getDistance(x_i(0), x_i(1));
        //     //std::cout << "dist2obs:" << dist2obs << std::endl;

        //     Vec2d vec_o2x(x_i(0) - instance->voronoi_.data[x_i(0)][x_i(1)].obstX,
        //                   x_i(1) - instance->voronoi_.data[x_i(0)][x_i(1)].obstY);
        
        //     int x_v = x_i(0), y_v = x_i(1);//路点的栅格索引
        //     std::vector<Vec2i> voronoi_points;
        //     for (int i = x_v - 30; i < (x_v + 30); ++i) {
        //         for (int j = y_v - 30; j < (y_v + 30); ++j) {
        //             // 获取其邻近区域内的Voronoi图上的点
        //             if (instance->voronoi_.isVoronoi(x_v, y_v)) {
        //                 voronoi_points.push_back({x_v, y_v});
        //             }
        //         }
        //     }
        //     // 计算当前点到最近边界的距离
        //     double dist2edge;
        //     Vec2d vec_e2x;
        //     // 表示当前点周围没有在Voronoi图上的点
        //     if (voronoi_points.empty()) {
        //         dist2edge = 3;//将dist2edge设置为一个较大的值3
        //         vec_e2x  = -vec_o2x;
        //     } else {
        //         int min_idx = 0;
        //         double min_dist = 10;
        //         for (int i = 0; i < voronoi_points.size(); ++i) {
        //             // 根据当前点与邻近Voronoi点的距离，找到距离最小的Voronoi点
        //             double dist = 0.1 * (x_i - voronoi_points[i]).norm();
        //             if (dist < min_dist) {
        //                 min_dist = dist;
        //                 min_idx = i;
        //             }
                    
        //         }
        //         dist2edge = min_dist;
        //         // 计算从该Voronoi点到当前点的向量
        //         vec_e2x(x_i(0) - voronoi_points[min_idx](0),
        //                 x_i(1) - voronoi_points[min_idx](1));
        //     }

        //     double alpha = instance->alpha_;

        //     if (dist2obs - max_clearance < 0) {
        //         // std::cout << "求gradient:" << std::endl;
        //         voronoi_cost += instance->w_vor_ * alpha /(alpha + dist2obs)
        //                         * dist2edge / (dist2edge + dist2obs)
        //                         * pow(dist2obs - max_clearance, 2) / pow(max_clearance, 2);
                
        //         // std::cout << "求gradient:" << std::endl;
        //         Vec2d gradient;
        //         gradient = instance->w_vor_ * 
        //                    (alpha /(alpha + dist2obs)
        //                    * dist2edge / (dist2edge + dist2obs)
        //                    * (dist2obs - max_clearance) / pow(max_clearance, 2)
        //                    * ((max_clearance - dist2obs)/(alpha + dist2obs)
        //                      -(dist2obs - max_clearance) / (dist2obs + dist2edge) + 2)
        //                    * vec_o2x / dist2obs
                        
        //                    + 
                        
        //                     alpha /(alpha + dist2obs) 
        //                    * dist2obs / pow(dist2edge + dist2obs, 2)
        //                    * pow(dist2obs - max_clearance, 2) / pow(max_clearance, 2)
        //                    * vec_e2x / dist2edge
        //                    );
                            
        //         voronoi_grad(0, i-2) = gradient(0);
        //         voronoi_grad(1, i-2) = gradient(1);
        //     } else {
        //         voronoi_cost += 0;
        //         voronoi_grad(0, i-2) = 0;
        //         voronoi_grad(1, i-2) = 0;
        //     }
        // }
        // //std::cout << "smooth_grad" << smooth_grad << std::endl;
        // cost += voronoi_cost;
        // grad +=  voronoi_grad;



        g.setZero();
        // 将grad矩阵的第一行转置后赋值给了g矩阵的前points_num行
        g.head(points_num) = grad.row(0).transpose();
        // 将grad矩阵的第二行转置后赋值给了g矩阵的后points_num行
        g.tail(points_num) = grad.row(1).transpose();



        // std::cout << std::setprecision(10)
        // std::cout << "------------------------" << "\n";
        // std::cout << "Function Value: " << cost << "\n";
        // std::cout << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << "\n";
        // std::cout << "------------------------" << "\n";

        return cost;

    }


    // Path optimization: l(Limited-memory)-bfgs smoothing
    double Smoother::optimize(DynamicVoronoi &voronoi, std::vector<Vec3d> &path, const nav_msgs::OccupancyGridConstPtr& grid_map) {
        smooth_path_ = path;
        // Path to be optimized
        voronoi_ = voronoi;
        // Dynamic Voronoi diagram
        grid_map_ = grid_map;
        // Grid map

        int points_num = smooth_path_.size() - 4;
        // Remove the first two points and the last three points (we added the goal point to the path), which are not optimized
        Eigen::VectorXd x(2 * points_num);
        // Represents the x and y coordinates of the path points
        for (int i = 2; i < smooth_path_.size()-2; ++i) {
            // Start from the third point and end at the fourth-to-last point
            // The first `points_num` elements store the x coordinates of the path points, and the last `points_num` elements store the y coordinates of the path points
            x(i-2) = smooth_path_[i](0);
            // Fill x
            x(i-2 + points_num) = smooth_path_[i](1);
            // Fill y
        }

        //std::cout << "About to optimize" << std::endl;
        double minCost = 0.0;
        lbfgs_params.mem_size = 256;
        // This may refer to the amount of historical information used by the LBFGS algorithm, which is the "Limited-memory" part of the LBFGS name. This value is set to 256, indicating that the algorithm will use information from the most recent 256 iterations.
        // lbfgs_params.past = 3;
        // This parameter may be used to check for convergence. If the improvements in the most recent `past` iterations are very small, the algorithm may stop and declare convergence. This value is set to 3, meaning the algorithm will look at the most recent 3 iterations.
        lbfgs_params.past = 0;
        // lbfgs_params.min_step = 1.0e-32;
        // This is the lower bound for the step size in the algorithm's line search. If the step size becomes too small, the algorithm may stop prematurely. This value is set to 1.0e-32, a very small positive number, allowing the algorithm to continue even with very small step sizes.
        // lbfgs_params.max_linesearch =256;
        lbfgs_params.max_linesearch =64;
        // lbfgs_params.g_epsilon = 0.0;
        // In some implementations, this is the threshold for the gradient. When the norm of the gradient falls below this value, the algorithm may stop and declare convergence. This value is set to 0.0, possibly indicating that this condition is disabled.
        lbfgs_params.delta = 1.0e-5;
        // This may be the tolerance value used for convergence testing. If the improvements in several consecutive iterations are smaller than this value, the algorithm may stop. This value is set to 1.0e-5.

        int ret = lbfgs::lbfgs_optimize(x,
                                        minCost,
                                        &Smoother::costFunction,
                                        // Cost objective function
                                        nullptr,
                                        nullptr,
                                        this,
                                        lbfgs_params);
        // std::cout << "ret:" << ret << std::endl;
        
        // std::cout << "minCost" << minCost << std::endl;
        // Calculate the heading angle of each point on the path. Indices 0-1 are the original points.
        for (int i = 2; i < smooth_path_.size() -2; ++i) {
            smooth_path_[i](0) = x(i-2);
            // x
            smooth_path_[i](1) = x(i-2 + points_num);
            // y
            smooth_path_[i-1](2) = std::atan2(smooth_path_[i](1) - smooth_path_[i-1](1),
                                        smooth_path_[i](0) - smooth_path_[i-1](0));
                                        // yaw
        }
        // The yaw of the last optimized point
        smooth_path_[smooth_path_.size() - 3](2) = std::atan2(smooth_path_[smooth_path_.size() -2](1) - smooth_path_[smooth_path_.size() -3](1),
                                        smooth_path_[smooth_path_.size() -2](0) - smooth_path_[smooth_path_.size() -3](0));
        if (ret >= 0) {
            
            // std::cout << "smooth_path_[i-1](2)" << smooth_path_[i-1](2) << std::endl;
            
            std::cout << "Optimization success" << std::endl;
        } else {
            minCost = INFINITY;
            std::cout << "Optimization Failed: "
                        << lbfgs::lbfgs_strerror(ret)
                        << std::endl;
        }
        return minCost;
    }


    // Gradient Descent Version
    void Smoother::smoothPath(DynamicVoronoi &voronoi, std::vector<Vec3d> &path, const nav_msgs::OccupancyGridConstPtr& grid_map) {
        path_ = path;
        smooth_path_ = path;
        voronoi_ = voronoi;
        grid_map_ = grid_map;
        // Grid map
        resolution_ = grid_map_->info.resolution;
        x_min_ = grid_map_->info.origin.position.x;
        // The value of the map origin (first grid point) in the frame_id coordinate system ("map")
        x_max_ = x_min_ + grid_map_->info.width * resolution_;
        // grid.info.width: width: the number of grids in the x direction
        y_min_ = grid_map_->info.origin.position.y;
        y_max_ = y_min_ + grid_map_->info.height * resolution_;
        // Voronoi width and height
        width_ = voronoi_.getSizeX();
        height_ = voronoi_.getSizeY();
        
        int iter = 0;
        int max_iter = 1000;

        double weight_sum = w_obs_ + w_cur_ + w_smo_ + w_vor_;

        while (iter < max_iter) {
            
            for (int i = 2; i < path_.size() - 2; ++i) {
                Vec2d x_p2(smooth_path_[i-2](0), smooth_path_[i-2](1));
                Vec2d x_p(smooth_path_[i-1](0), smooth_path_[i-1](1));
                Vec2d x_c(smooth_path_[i](0), smooth_path_[i](1));
                Vec2d x_n(smooth_path_[i+1](0), smooth_path_[i+1](1));
                Vec2d x_n2(smooth_path_[i+2](0), smooth_path_[i+2](1));

                Vec2d correction = Vec2d::Zero();

                correction = correction + calObstacleTerm(x_c);
                
                if (!isInMap(x_c - alpha_ * correction / weight_sum)) continue;

                
                correction = correction + calSmoothTerm(x_p2, x_p, x_c, x_n, x_n2);
                
                
                if (!isInMap(x_c - alpha_ * correction / weight_sum)) continue;


                x_c = x_c - alpha_ * correction / weight_sum;
                
                smooth_path_[i](0) = x_c(0);
                smooth_path_[i](1) = x_c(1);

                Vec2d delta_x = x_c - x_p;
                if (i > 1) {
                    smooth_path_[i-1](2) = std::atan2(delta_x(1), delta_x(0));
                }
            }
            ++iter;
        }
        std::cout << iter << std::endl;
    }

    // Calculate the obstacle term
    Vec2d Smoother::calObstacleTerm(Vec2d x) {
        Vec2d gradient;

        Vec2i x_i;
        // x_i(0) = static_cast<int>((x(0) - (-25)) / 0.1);
        x_i(0) = static_cast<int>((x(0) - x_min_) / resolution_);
        // x_i(1) = static_cast<int>((x(1) - (-25)) / 0.1);
        x_i(1) = static_cast<int>((x(1) - y_min_) / resolution_);

        // Distance to the closest obstacle    unit m
        double dist2obs = 0.1 * voronoi_.getDistance(x_i(0), x_i(1));

        Vec2d vec_o2x(x_i(0) - voronoi_.data[x_i(0)][x_i(1)].obstX,
                    x_i(1) - voronoi_.data[x_i(0)][x_i(1)].obstY);
        
        if (dist2obs  < max_clearance_) {
            gradient = w_obs_ * 2 * (dist2obs - max_clearance_) / dist2obs * vec_o2x;
        } else {
            gradient = Vec2d::Zero();
        }
        return gradient;
    }

    // Calculate the smoothness term
    Vec2d Smoother::calSmoothTerm(Vec2d x_p2, Vec2d x_p, Vec2d x_c, Vec2d x_n, Vec2d x_n2) {
        Vec2d gradient; // Gradient
        gradient = w_smo_ * (x_p2 - 4 * x_p + 6 * x_c - 4 * x_n + x_n2);
        return gradient;
    }

    // Check if a point is within the map
    inline bool Smoother::isInMap(Vec2d x) {
        // if (x(0) < -25 || x(1) < -25 || x(0) >= 25 || x(1) >= 25) {
        if (x(0) < x_min_ || x(1) < y_min_ || x(0) >= x_max_ || x(1) >= y_max_) {
            return false;
        }
        return true;
    }


} // namespace planning


