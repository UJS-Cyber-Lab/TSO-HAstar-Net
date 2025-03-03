#pragma once

#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <memory>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_set>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetMap.h> //OccupancyGrid用
#include <visualization_msgs/MarkerArray.h>

#include "hybrid_astar_searcher/type_defs.h" 
#include "hybrid_astar_searcher/path_adjust.h"
#include "hybrid_astar_searcher/visualize.h"
#include "hybrid_astar_searcher/collisiondetection.h"


namespace planning {

// 存入8个朝向及其对应的角度（弧度制）
// {x,y}-->yaw
static const std::map<std::pair<int, int>, float> directionAngles = {
    {{-1,-1}, -3 * M_PI_4},
    {{-1,0}, M_PI},
    {{-1,1}, 3 * M_PI_4},
    {{0,-1}, -M_PI_2},
    {{0,1}, M_PI_2},
    {{1,-1}, -M_PI_4},
    {{1,0}, 0.0},
    {{1,1}, M_PI_4}
};

//Point2D结构体 int x,y
struct Point2D {
  int x, y;
};

struct GridPoint {
  int x, y;
  float distance_to_goal;  // 这是从这个点到目标点的路径距离
};


/**
 * @brief 用于A*搜索和构建离线的查询障碍物启发函数距离的表
 * 
 */
class Node2d
{
private:
    Vec2i pos_map_;
    double g_, h_, f_, dir_;
    int index_;
    std::shared_ptr<Node2d> parent_;

public:
    Node2d(Vec2i pos_map, const int map_size_x) {
        pos_map_ = pos_map;//行列索引
        g_ = 0.0;
        h_ = 0.0;
        f_ = 0.0;
        index_ = pos_map_(1) * map_size_x + pos_map_(0);//由行列索引得栅格索引
        parent_ = nullptr;
        dir_ = 0.; //存放当前节点相对于上一个节点的朝向角度，只有8个朝向
    }
    void setG(const double g) {g_ = g; f_ = g_ + 1*h_;}
    void setH(const double h) {h_ = h; f_ = g_ + 1*h_;}
    void setF(const double f) {f_ = f;}
    void setDIR(const float dir) {dir_ = dir;}
    void setParent(std::shared_ptr<Node2d> node) {parent_ = node;}

    int getPosXInMap() {return pos_map_(0);}//行列索引
    int getPosYInMap() {return pos_map_(1);}
    double getG() {return g_;}
    double getH() {return h_;}
    double getF() {return f_;}
    float getDIR() {return dir_;}
    int getIndex() {return index_;}
    std::shared_ptr<Node2d> getParentNode() {return parent_;}
};


class GridSearch {
private:
    double x_min_, x_max_, y_min_, y_max_, resolution_;
    int map_size_x_, map_size_y_;
    // grid_map::GridMap gmap_;
    // nav_msgs::OccupancyGrid::ConstPtr map_;
    nav_msgs::OccupancyGridConstPtr map_;
    std::shared_ptr<Node2d> start_node_;
    std::shared_ptr<Node2d> goal_node_;
    std::shared_ptr<Node2d> final_node_;
    // double heu_astar_;
    int explored_node_num_;
    int num;

    struct cmp {
        bool operator() (const std::pair<int, double>& l, 
                         const std::pair<int, double>& r) {
            return l.second > r.second;                
        }
    };

    // DP-Map
    std::unordered_map<int, std::shared_ptr<Node2d>> dp_map_; 
    // KD-tree
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    // 点云
    pcl::PointCloud<pcl::PointXY>::Ptr cloud;
    Visualize vis;
    ros::Publisher path_point_vis_pub;
    std::vector<GridPoint> astar_points;
    // 碰撞检测类
    std::unique_ptr<CollisionDetection> configurationSpace_ptr_;
    
public:
    // GridSearch(grid_map::GridMap map);
    GridSearch(const nav_msgs::OccupancyGridConstPtr& map, CollisionDetection& configurationSpace);
    // bool generateDpMap(Vec2d goal_pos);
    bool generateDpMap(Vec2d goal_pos, Vec2d start_pos);
    bool generateLocalDpMap(Vec2d goal_pos, Vec2d start_pos);
    double lookupInDpMap(Vec2d start_pos);
    void InitLookUpTable(Vec3d& goal_pos, Vec3d& start_pos);
    double CheckLookUpTable(Vec2d start_pos);
    std::vector<Point2D> PathShorten(std::vector<Point2D> const& path);
    std::vector<Point2D> calculateHeuByAstar(Vec3d& start_pos, Vec3d& goal_pos);
    double calculateHeuByAstar(Vec2d start_pos, Vec2d goal_pos);
    void mod2Pi(double &angle);
    void clear();

private:
    std::vector<std::shared_ptr<Node2d>> getNeighborNodes(std::shared_ptr<Node2d> cur_node);
    double EuclidDistance(const double x1, const double y1,
                          const double x2, const double y2);
    bool isInGridMap(std::shared_ptr<Node2d> node);
    bool isInLimitedMap(std::shared_ptr<Node2d> node, VecXd lzone);
    bool isOnObstacle(std::shared_ptr<Node2d> node);
    Vec2i getIndexFromPosition(Vec2d position);
    Vec2d getPositionFromIndex(Vec2i index);
    bool collision(const double x, const double y);
    bool LineHit(const Point2D pose1, const Point2D pose2);
    inline double Distance(const Point2D& a, const Point2D& b);
    void getPointsOnPolyline(const std::vector<Point2D>& polyline, float interval, std::vector<GridPoint>& points);
    std::vector<Vec2d> getAstartPath();
    std::vector<Point2D> PathInterpolation(std::vector<Point2D> const& path);

};


} //namespace planning



