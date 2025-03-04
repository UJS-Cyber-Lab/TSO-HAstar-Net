#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <unordered_map>

#include "hybrid_astar_searcher/hybrid_astar.h"
#include "hybrid_astar_searcher/dynamicvoronoi.h"
#include "hybrid_astar_searcher/smooth.h"
#include "hybrid_astar_searcher/visualize.h"


namespace chibot::topologyplanning {
  using namespace planning;
  using std::vector;

  #define RED     "\033[31m"           /* Red */
  #define GREEN   "\033[32m"           /* Green */
  #define WHITE   "\033[37m"           /* White */
  #define SET_FONT_ON  "\033[6;2s"     
  #define SET_FONT_1   "\033[1t"       
  #define SET_FONT_0   "\033[0t"       

  typedef struct point {
    double x;
    double y;
    double theta;
    bool operator==(point const& rhs) const {
      return std::abs(x - rhs.x) < 1e-6 && std::abs(y - rhs.y) < 1e-6;
    }
  } point_t;

  typedef struct line { // edge
    point_t a; //Start
    point_t b; //Goal
    std::vector<int> go_list {}; //Interconnectedness(labelled by path(edge) numbers(0,1,2...))
  } line_t;
  using lines_type = std::vector<line_t>;

  struct setnode {
    point_t nearest_point;
    int line;
    double nearest_dist;
    int nearest_index;
    bool operator<(setnode const& rhs) const {
      return nearest_dist <= rhs.nearest_dist;
    }
  };

  struct PairHash {
      template <typename T, typename U>
      std::size_t operator()(const std::pair<T, U>& x) const {
          return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
      }
  };

  class HalfStructPlanner {
    public:
      HalfStructPlanner(const nav_msgs::OccupancyGridConstPtr& grid, ros::NodeHandle &nh);
      ~HalfStructPlanner();
      void init(ros::NodeHandle &nh);
      void set_traffic_route(lines_type const& lines);
      void set_traffic_route_topology(lines_type const& lines);
      auto get_path(geometry_msgs::PoseStamped const& start,
                    geometry_msgs::PoseStamped const& goal) -> std::vector<geometry_msgs::PoseStamped>; 
      void visSGPath(const std::vector<vector<Eigen::Vector3d>>& paths);
      void vistopologyPath(const std::vector<vector<Eigen::Vector3d>>& paths);
      void calculate_pre_graph(lines_type const& lines, vector<double> const& distance_table);
      void HybridAstarplan(Vec3d start_pose, Vec3d goal_pose);

    private:
      auto is_traffic_plan() -> bool;
      void visPath(std::vector<vector<Eigen::Vector3d>> paths);
      void calculate_graph();
      auto nearest_point_of_segment(point_t const& p, vector<Eigen::Vector3d> const * paths, bool flag) -> setnode;
      auto distance(point_t const& a, point_t const& b) -> double;
      auto distance(geometry_msgs::PoseStamped const& a, geometry_msgs::PoseStamped const& b) -> double {
        auto pa = point_t {a.pose.position.x, a.pose.position.y};
        auto pb = point_t {b.pose.position.x, b.pose.position.y};
        return distance(pa, pb);
      }
      void print_graph();
      bool line_safe(point_t const& a, point_t const& b);
      auto line_safe(geometry_msgs::PoseStamped const& a, geometry_msgs::PoseStamped const& b) -> bool {
        auto pa = point_t {a.pose.position.x, a.pose.position.y};
        auto pb = point_t {b.pose.position.x, b.pose.position.y};
        return line_safe(pa, pb);
      }
      auto dijkstra() -> std::vector<int>;
      void visOptPath(std::vector<vector<Eigen::Vector3d>> paths);
      void show_graph();
      void show_graph_(std::vector<int> node_list);
      auto write_file(std::string& file_path, std::vector<Eigen::Vector3d> const& path, double length) -> bool;
      void visfullPath(const std::vector<Eigen::Vector3d>& poses);

    private:
      lines_type traffic_routes_;
      double** graph_;
      double** graph_bk_;
      int node_num_;
      int connection_point_num;
      double connection_points_range;
      double step_size;

      std::vector<geometry_msgs::PoseStamped> nodes_;
      std::vector<vector<Eigen::Vector3d>> SG_paths;
      std::vector<vector<Eigen::Vector3d>> topology_paths;
      std::vector<vector<Eigen::Vector3d>> ContactS_paths; //S(G)->CS(CG)
      std::vector<vector<Eigen::Vector3d>> ContactG_paths;
      std::vector<vector<Eigen::Vector3d>> ContactS_paths_; //CS(CG)->Path endpoint
      std::vector<vector<Eigen::Vector3d>> ContactG_paths_;
      std::vector<vector<Eigen::Vector3d>> ContactSG_paths; //CS->CG     
      ros::Publisher vis_pub_;
      ros::Publisher string_pub_;
      ros::Publisher res_pub_;
      ros::Publisher path_vis_pub, marker_pub, optimized_path_vis_pub, path_point_vis_pub, SG_path_vis_pub, topology_path_vis_pub, full_path_vis_pub;
      ros::Publisher voronoi_pub;
      std::unordered_map<std::pair<int, int>, const std::vector<Eigen::Vector3d>*, PairHash> mappingTable;

      std::unique_ptr<HybridAstar> hybrid_astar_ptr;
      std::unique_ptr<Smoother> smoother_ptr;
      HybridAstarResult result;
      visualization_msgs::Marker voronoi;
      DynamicVoronoi voronoiDiagram;
      nav_msgs::OccupancyGridConstPtr grid_;
      Visualize vis;

  };

}
