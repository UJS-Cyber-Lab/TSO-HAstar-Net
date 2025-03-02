#include <csignal>
#include "simple_astar_test.h"

std::shared_ptr<chibot::topologyplanning::SimpleAstarTest> simple_astar_test_ptr;

void sigintHandler(int sig) {
  // 对象已建立
  if (simple_astar_test_ptr) {
    // simple_astar_test_ptr->stop();
    // 终止导航
    simple_astar_test_ptr.reset();
  }
  ROS_INFO("hybrid astar dubins opt shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_astar_test");
  simple_astar_test_ptr = std::make_shared<chibot::topologyplanning::SimpleAstarTest>();
  signal(SIGINT, sigintHandler);
  // Ctrl+C触发
  simple_astar_test_ptr->init();
  // 初始化
  simple_astar_test_ptr->run();
  // 运行
  return 0;
}
