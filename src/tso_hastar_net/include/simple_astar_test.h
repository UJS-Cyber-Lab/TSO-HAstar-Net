#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <tf2/utils.h>
#include "half_struct_planner.h"


namespace chibot::topologyplanning {
    using namespace planning;
    using namespace std;

    enum class StateValue : uint8_t {
        Idle = 0,
        Record,
        Record_Traffic_route,
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
            void startCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
            void goalCallback(geometry_msgs::PoseStamped::ConstPtr const& msg);
            void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

        private:
            ros::NodeHandle nh;
            std::shared_ptr<HalfStructPlanner> half_struct_planner_;
            ros::Subscriber start_sub;
            ros::Subscriber goal_sub;
            ros::Subscriber map_sub_;
            bool has_start, has_goal;
            geometry_msgs::PoseStamped start, goal;
            StateValue cur_state_;
            Visualize vis;
            boost::mutex map_mutex_;
    };

}
