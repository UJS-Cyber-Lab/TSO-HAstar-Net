#pragma once

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include "type_defs.h"

namespace planning
{
    class Visualize
    {
    private:
        ros::NodeHandle nh;
        ros::Publisher explored_nodes_pub, vehicle_boxes_pub, path_points_pub, start_pose_pub, goal_pose_pub;
        ros::Publisher primitive_astar_pub, simple_astar_pub;
        geometry_msgs::PoseArray nodes_pose, start_poses, goal_poses;
        visualization_msgs::MarkerArray vehicle_boxes, path_points;


    public:
        Visualize(/* args */) {
            //可视化生长节点，这种可视化是rviz自带的插件，设置都是在rviz中
            explored_nodes_pub = nh.advertise<geometry_msgs::PoseArray>("/visualize_nodes_pose", 10);
            //可视化起点位姿
            start_pose_pub = nh.advertise<geometry_msgs::PoseArray>("/visualize_start_pose", 10);
            //可视化终点位姿
            goal_pose_pub = nh.advertise<geometry_msgs::PoseArray>("/visualize_goal_pose", 10);
            //可视化小车轮廓，这种可视化是自定义的插件
            vehicle_boxes_pub = nh.advertise<visualization_msgs::MarkerArray>("/vehicle_boxes", 10);
            path_points_pub = nh.advertise<visualization_msgs::MarkerArray>("/path_points", 10);

            //可视化原始A*路径
            primitive_astar_pub = nh.advertise<nav_msgs::Path>("/visualize_primitive_astar", 10);
            //可视化折线A*路径
            simple_astar_pub = nh.advertise<nav_msgs::Path>("/visualize_simple_astar", 10);
        }

        void vis_clear() {
            // nodes_pose.poses.clear();
            // vehicle_boxes.markers.clear();
            // start_poses.poses.clear();
            // goal_poses.poses.clear();
        }

        void vis_nodes_clear(){nodes_pose.poses.clear();}//清理生长节点的时机要选好
        //节点生长可视化
        void publishExploredNodes(Vec3d node_pose) {
            nodes_pose.header.frame_id = "map";
            nodes_pose.header.stamp = ros::Time::now();
            geometry_msgs::Pose pose;
            pose.position.x = node_pose(0);
            pose.position.y = node_pose(1);
            pose.orientation = tf::createQuaternionMsgFromYaw(node_pose(2));
            nodes_pose.poses.push_back(pose);
            explored_nodes_pub.publish(nodes_pose);
        }
        

        //可视化起始位姿
        void publishStartPoses(Vec3d start_pose) {
            start_poses.poses.clear();
            start_poses.header.frame_id = "map";
            start_poses.header.stamp = ros::Time::now();
            geometry_msgs::Pose pose;
            pose.position.x = start_pose(0);
            pose.position.y = start_pose(1);
            pose.orientation = tf::createQuaternionMsgFromYaw(start_pose(2));
            start_poses.poses.push_back(pose);
            start_pose_pub.publish(start_poses);
        }

        //可视化终点位姿
        void publishGoalPoses(Vec3d goal_pose) {
            goal_poses.poses.clear();
            goal_poses.header.frame_id = "map";
            goal_poses.header.stamp = ros::Time::now();
            geometry_msgs::Pose pose;
            pose.position.x = goal_pose(0);
            pose.position.y = goal_pose(1);
            pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose(2));
            goal_poses.poses.push_back(pose);
            goal_pose_pub.publish(goal_poses);
        }


        // 发布车辆的边界框（车辆轮廓）
        void publishVehicleBoxes(Vec3d node_pose, int i) {
            visualization_msgs::Marker vehicle_box;
            if (i == 0)  vehicle_box.action = 3;//需要清除之前发布的所有车辆边界框
            vehicle_box.header.frame_id = "map";
            vehicle_box.header.stamp = ros::Time(0);
            vehicle_box.id = i;
            vehicle_box.type = visualization_msgs::Marker::CUBE;//立方体
            //车轮廓
            vehicle_box.scale.x = 0.72+0.2;
            vehicle_box.scale.y = 0.58+0.2;//带膨胀
            vehicle_box.scale.z = 0.01;
            vehicle_box.color.a = 0.4;
            vehicle_box.color.r = 0;
            vehicle_box.color.b = 1;
            vehicle_box.color.g = 0;

            // Vec2d center = {node_pose(0) + 1.45 * std::cos(node_pose(2)),
            //             node_pose(1) + 1.45 * std::sin(node_pose(2))};
            // vehicle_box.pose.position.x = center(0);
            vehicle_box.pose.position.x = node_pose(0);
            // vehicle_box.pose.position.y = center(1);
            vehicle_box.pose.position.y = node_pose(1);
            vehicle_box.pose.orientation = tf::createQuaternionMsgFromYaw(node_pose(2));
            vehicle_boxes.markers.push_back(vehicle_box);
            vehicle_boxes_pub.publish(vehicle_boxes);
        }


        //发布路点
        void publishPathPoint(Vec2d position, int i) {
            visualization_msgs::Marker path_point;
            if (i == 0)  path_point.action = 3;
            path_point.header.frame_id = "map";
            path_point.header.stamp = ros::Time(0);
            path_point.id = i;
            path_point.type = visualization_msgs::Marker::SPHERE;
            path_point.scale.x = 0.1;
            path_point.scale.y = 0.1;
            path_point.scale.z = 0.1;
            path_point.color.a = 1;
            path_point.color.r = 1;
            path_point.color.g = 0;
            path_point.color.b = 0;
            path_point.pose.position.x = position(0);
            path_point.pose.position.y = position(1);
            path_point.pose.position.z = 0.1;
            path_point.pose.orientation.w = 1.0;

            path_points.markers.push_back(path_point);
            path_points_pub.publish(path_points);
        }

        
        //发布原始A*路径
        void publishPrimitiveAstar(std::vector<geometry_msgs::PoseStamped> const& path) {
            nav_msgs::Path primitive_astar_path;
            primitive_astar_path.header.frame_id = "map";
            for(auto const& pose : path)
            {
                geometry_msgs::PoseStamped current_pose;
                current_pose.header.frame_id = "map";
                current_pose.pose.position.x = pose.pose.position.x;
                current_pose.pose.position.y = pose.pose.position.y;
                primitive_astar_path.poses.push_back(current_pose);
            }
            primitive_astar_pub.publish(primitive_astar_path);
        }

        //发布折线A*路径
        void publishSimpleAstar(std::vector<geometry_msgs::PoseStamped> const& path) {
            nav_msgs::Path simple_astar_path;
            simple_astar_path.header.frame_id = "map";
            for(auto const& pose : path)
            {
                geometry_msgs::PoseStamped current_pose;
                current_pose.header.frame_id = "map";
                current_pose.pose.position.x = pose.pose.position.x;
                current_pose.pose.position.y = pose.pose.position.y;
                simple_astar_path.poses.push_back(current_pose);
            }
            simple_astar_pub.publish(simple_astar_path);
        }

};

} // namespace planning







