#include "simple_astar_test.h"


namespace chibot::topologyplanning {

  SimpleAstarTest::SimpleAstarTest():
  has_start(false),
  has_goal(false),
  cur_state_(StateValue::Run){}

  void SimpleAstarTest::init() {
    map_sub_ = nh.subscribe("/map", 1, &SimpleAstarTest::mapReceived, this);
    start_sub = nh.subscribe("/initialpose", 1, &SimpleAstarTest::startCallback, this);
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &SimpleAstarTest::goalCallback, this);
  }

  void SimpleAstarTest::run() {
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        switch (cur_state_) {
            case StateValue::Idle:
                break;
            case StateValue::Pause:
                break;
            case StateValue::Record_Traffic_route:
                break;
            case StateValue::Run:
                if (has_goal) {
                    Vec3d start_pose = Eigen::Vector3d{start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation)};
                    Vec3d goal_pose = Eigen::Vector3d{goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation)};
                    half_struct_planner_->HybridAstarplan(start_pose, goal_pose);
                    has_goal = false;
                }
                break;

            default:
                ROS_ERROR("Error StateValue!");
                break;
        }
        rate.sleep();
    }
  }

  // Describe the starting pose
  void SimpleAstarTest::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
    Vec3d start_pose;
    start_pose[0] = msg.pose.pose.position.x;
    start_pose[1] = msg.pose.pose.position.y;
    start_pose[2] = tf2::getYaw(msg.pose.pose.orientation);
    start.header = msg.header;
    start.pose.position = msg.pose.pose.position;
    start.pose.orientation = msg.pose.pose.orientation;
    has_start = true;
    vis.publishStartPoses(start_pose); // Visualize the starting point
  }

  // Describe the goal pose
  void SimpleAstarTest::goalCallback(geometry_msgs::PoseStamped::ConstPtr const& msg) {
      Vec3d goal_pose;
      goal_pose[0] = msg->pose.position.x;
      goal_pose[1] = msg->pose.position.y;
      goal_pose[2] = tf2::getYaw(msg->pose.orientation);
      goal = *msg;
      has_goal = true;
      vis.publishGoalPoses(goal_pose);
  }

  // Receive map information.
  void SimpleAstarTest::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg) {
    boost::mutex::scoped_lock lock(map_mutex_);
    half_struct_planner_ = std::make_shared<HalfStructPlanner>(msg, nh);
    half_struct_planner_->init(nh);
  }


}
