#pragma once


#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/GetMap.h>
// nav_msgs/GetMap是map_server提供的服务的消息类型。

// #include "hybrid_astar_searcher/calculate_heuristic.h"
#include "hybrid_astar_searcher/hybrid_astar.h"
#include "hybrid_astar_searcher/dynamicvoronoi.h"
#include "hybrid_astar_searcher/smooth.h"
#include "hybrid_astar_searcher/visualize.h"

#include <tf2/utils.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <unordered_map>


namespace chibot::topologyplanning {
  using namespace planning;
  using std::vector;

  /*这是一个用于在终端显示红色文本的宏定义。在终端中，通常使用转义字符来控制文本的颜色和样式。每种颜色都对应一个特定的转义序列。
    在这个宏定义中，"\033[31m"表示将终端输出的文本颜色设置为红色，后续的文本将以红色显示。
    要使用这个宏定义，只需在需要显示红色文本的地方插入RED即可
    例如：cout << RED << "这是红色的文本" << endl;
    这样就会在终端上显示红色的文本。请注意，在使用完红色文本后，需要使用另一个转义序列将文本颜色恢复为默认值，否则后续的文本仍然会以红色显示。通常，可以使用"\033[0m"来将文本颜色恢复为默认值。例如：
    cout << RED << "这是红色的文本" << "\033[0m" << endl;
    这样就可以将红色文本显示结束后，恢复为默认颜色。*/
  #define RED     "\033[31m"           /* Red */
  #define GREEN   "\033[32m"           /* Green */
  #define WHITE   "\033[37m"           /* White */
  #define SET_FONT_ON  "\033[6;2s"     /* 开始设置字体 */
  #define SET_FONT_1   "\033[1t"       /* 字体大小为3 */
  #define SET_FONT_0   "\033[0t"       /* 将字体大小设置为默认值 */


  typedef struct point {
    // 点的x,y坐标
    double x;
    double y;
    double theta;
    /*这是一个重载了等于运算符（==）的成员函数。该函数用于比较当前对象与另一个point对象rhs是否相等。
      在该函数的实现中，首先使用std::abs()函数计算当前对象的x坐标与rhs对象的x坐标的差值的绝对值，并与1e-6（即0.000001）进行比较。然后再计算当前对象的y坐标与rhs对象的y坐标的差值的绝对值，并与1e-6进行比较。
      如果两个差值均小于1e-6，则认为两个point对象的x坐标和y坐标非常接近，即认为它们是相等的。此时，operator==()函数返回true；否则，返回false。
      需要注意的是，通过使用浮点数的绝对值来进行比较存在一定的误差问题。由于浮点数的精度限制，可能会导致非常接近但不完全相等的浮点数被判定为不相等。在实际应用中，可能需要根据具体情况调整比较的精度或使用其他方法来处理浮点数的比较。*/
    bool operator==(point const& rhs) const {
      return std::abs(x - rhs.x) < 1e-6 && std::abs(y - rhs.y) < 1e-6;
    }
  } point_t;

  typedef struct line {
    // 线的首尾坐标，路网起终点
    point_t a;
    point_t b;
    // 存放的是当前路网能够到达哪些路网，其起点索引
    std::vector<int> go_list {};
    // 定义了一个空的std::vector<int>对象go_list
  } line_t;

  /*这是一个类型别名定义语句，用于为std::vector<line_t>类型定义一个别名lines_type。
    std::vector<line_t>表示一个存储line_t类型元素的动态数组，即包含多个line_t元素的向量。
    using是C++11中引入的一个关键字，用于定义类型别名。这里使用using关键字将std::vector<line_t>类型定义为lines_type类型的别名。
    通过定义类型别名，可以方便地在代码中引用复杂或冗长的类型，提高代码的可读性和可维护性。例如，在使用lines_type类型时，可以直接使用lines_type作为类型名，而无需写出完整的std::vector<line_t>类型。*/
  using lines_type = std::vector<line_t>;

  typedef struct graph {
    double cost = std::numeric_limits<double>::max();
    std::vector<geometry_msgs::PoseStamped> edge{};
  } graph_t; 
  // 没有用这个结构体实现，因为边都是直线，因此简单替换成长度了，如果支持复杂通道，需要把路径作为边存下来

  struct setnode {
    point_t nearest_point;
    // point_t nearest_point_b;
    int line;
    double nearest_dist;
    int nearest_index;
    // int nearest_index_b;
    // int nearest_index;
  /*这段代码定义了一个小于运算符重载函数，用于比较setnode类型的对象之间的大小关系。
    operator<是C++中的一个二元运算符，用于比较两个对象的大小关系。当我们对自定义类型进行排序或存储到有序容器中时，需要定义小于运算符，以便容器能够正确地对元素进行排序。
    在这个特定的代码段中，operator<被定义为setnode类的成员函数，参数为另一个setnode类型的对象rhs。因为它是一个成员函数，所以它可以访问setnode对象的私有成员变量。
    函数体内部，比较当前对象的nearest_dist属性与rhs对象的nearest_dist属性。如果当前对象的nearest_dist属性小于rhs对象的nearest_dist属性，则返回true，表示当前对象小于rhs对象；否则返回false，表示当前对象大于等于rhs对象。*/
    bool operator<(setnode const& rhs) const {
      return nearest_dist <= rhs.nearest_dist;
      // 很好的高效率的排序方法
    }
  };

  // 哈希表
  struct PairHash {
      template <typename T, typename U>
      std::size_t operator()(const std::pair<T, U>& x) const {
          return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
      }
  };

  class HalfStructPlanner {
    public:
      // HalfStructPlanner(const nav_msgs::OccupancyGridConstPtr& grid);
      HalfStructPlanner(const nav_msgs::OccupancyGridConstPtr& grid);
      ~HalfStructPlanner();
      // 析构函数
      void init(ros::NodeHandle &nh);
      // void set_costmap(std::shared_ptr<costmap_2d::Costmap2D> const& costmap) {
      //   costmap_ = costmap;
      // }

      void set_traffic_route(lines_type const& lines);
      void set_traffic_route_topology(lines_type const& lines);
      void set_traffic_route_(lines_type const& lines);
      // void show_string();

      //  auto get_path(geometry_msgs::PoseStamped const& start, geometry_msgs::PoseStamped const& goal) -> nav_msgs::Path;
      // 常引用参数start和goal
      // 返回一个std::vector<geometry_msgs::PoseStamped>类型的向量
      // geometry_msgs::PoseStamped用于表示带有时间戳的姿态信息
      // 函数声明中的auto关键字用于自动推导函数的返回类型，此处推导结果为std::vector<geometry_msgs::PoseStamped>
      auto get_path(geometry_msgs::PoseStamped const& start,
                    geometry_msgs::PoseStamped const& goal) -> std::vector<geometry_msgs::PoseStamped>; 
                    // 将返回的点序用goto连接，不直接返回路径，不在这个类做地图路径搜索

      auto is_traffic_plan() -> bool;
      auto get_nearest_point_of_start_and_update(geometry_msgs::PoseStamped const& start, geometry_msgs::PoseStamped& np) -> std::vector<geometry_msgs::PoseStamped>;
      void visPath(std::vector<vector<Eigen::Vector3d>> paths);
      void visSGPath(const std::vector<vector<Eigen::Vector3d>>& paths);
      // void visSGPath(std::vector<vector<Eigen::Vector3d>> paths);
      // void vistopologyPath(std::vector<vector<Eigen::Vector3d>> paths);
      void vistopologyPath(const std::vector<vector<Eigen::Vector3d>>& paths);
      void calculate_pre_graph(lines_type const& lines, vector<double> const& distance_table);
      void HybridAstarplan(Vec3d start_pose, Vec3d goal_pose);

    private:
      void show_traffic_route();
      void show_traffic_route_();
      // void show_graph();
      
      void calculate_graph();
      void calculate_graph_();
      // auto nearest_point_of_segment(point_t const& p, point_t const& a, point_t const& b) -> point_t;
      auto nearest_point_of_segment(point_t const& p, vector<Eigen::Vector3d> const * paths, bool flag) -> setnode;
      auto distance(point_t const& a, point_t const& b) -> double;
      auto distance(geometry_msgs::PoseStamped const& a, geometry_msgs::PoseStamped const& b) -> double {
      /*这行代码创建了一个名为pa的变量，并使用point_t类型的列表初始化对其进行初始化。初始化列表包含两个元素，分别是a.pose.position.x和a.pose.position.y。
        根据初始化列表的内容，可以推断出point_t是一个包含两个元素的类型，分别表示点的x坐标和y坐标。
        这行代码将a.pose.position.x赋值给pa的第一个元素，将a.pose.position.y赋值给pa的第二个元素，实现了从a的位置信息中提取出x和y坐标，并存储在pa中。*/
        auto pa = point_t {a.pose.position.x, a.pose.position.y};
        auto pb = point_t {b.pose.position.x, b.pose.position.y};
        return distance(pa, pb);
      }
      // 计算俩条边之间的拓扑的代价，用半圆周长作为代价
      auto distance_circle(geometry_msgs::PoseStamped const& a, geometry_msgs::PoseStamped const& b) -> double {
        auto pa = point_t {a.pose.position.x, a.pose.position.y};
        auto pb = point_t {b.pose.position.x, b.pose.position.y};
        double radius = distance(pa, pb) / 2.0;
        return M_PI*radius;
      }

      void print_graph();
      // auto line_safe(point_t const& a, point_t const& b) -> bool;
      // bool line_safe(point_t const& a, setnode const& b);
      bool line_safe(point_t const& a, point_t const& b);
      auto line_safe(geometry_msgs::PoseStamped const& a, geometry_msgs::PoseStamped const& b) -> bool {
        auto pa = point_t {a.pose.position.x, a.pose.position.y};
        auto pb = point_t {b.pose.position.x, b.pose.position.y};
        return line_safe(pa, pb);
      }

      auto dijkstra() -> std::vector<int>;
      auto get_nearest_point_of_start() -> geometry_msgs::PoseStamped;
      // auto is_free_(const geometry_msgs::PoseStamped& pose) const -> bool;
      // auto pose_cost_(const geometry_msgs::PoseStamped& pose) const -> uint8_t;
      
      void visOptPath(std::vector<vector<Eigen::Vector3d>> paths);
      void show_graph();
      void show_graph_(std::vector<int> node_list);
      auto write_file(std::string& file_path, std::vector<Eigen::Vector3d> const& path, double length) -> bool;
      void visfullPath(const std::vector<Eigen::Vector3d>& poses);

    private:
      lines_type traffic_routes_;
      // 路网，交通规则

      ros::Publisher vis_pub_;
      ros::Publisher string_pub_;
      ros::Publisher res_pub_;

      // graph_type **graph_;
      double** graph_;
      // 表示预处理的图
      double** graph_bk_;
      std::vector<geometry_msgs::PoseStamped> nodes_;

      // std::shared_ptr<costmap_2d::Costmap2D> costmap_;

      int node_num_;
      // 路径节点数
      std::unique_ptr<HybridAstar> hybrid_astar_ptr;
      ros::Publisher path_vis_pub, marker_pub, optimized_path_vis_pub, path_point_vis_pub, SG_path_vis_pub, topology_path_vis_pub, full_path_vis_pub;
      std::unique_ptr<Smoother> smoother_ptr;

      ros::Publisher voronoi_pub;
      HybridAstarResult result;
      // std::unique_ptr<DynamicVoronoi> voronoiDiagram_ptr;
      visualization_msgs::Marker voronoi;
      DynamicVoronoi voronoiDiagram;
      // nav_msgs::OccupancyGrid grid_;
      // nav_msgs::OccupancyGridPtr grid_;
      nav_msgs::OccupancyGridConstPtr grid_;

      std::vector<vector<Eigen::Vector3d>> SG_paths;
      std::vector<vector<Eigen::Vector3d>> topology_paths;
      // 起(终)点到接点
      std::vector<vector<Eigen::Vector3d>> ContactS_paths;
      std::vector<vector<Eigen::Vector3d>> ContactG_paths;
      // 接点到该边起点或终点
      std::vector<vector<Eigen::Vector3d>> ContactS_paths_;
      std::vector<vector<Eigen::Vector3d>> ContactG_paths_;
      // 接点到接点(若在同一条边上)
      std::vector<vector<Eigen::Vector3d>> ContactSG_paths;

      // 创建一个映射表，由两个数字编号映射到一个向量
      std::unordered_map<std::pair<int, int>, const std::vector<Eigen::Vector3d>*, PairHash> mappingTable;
      Visualize vis;

      int connection_point_num;
      double connection_points_range;
  };

}
