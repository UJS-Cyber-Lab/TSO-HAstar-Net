#include <csignal>
#include "simple_astar_test.h"

std::shared_ptr<chibot::topologyplanning::SimpleAstarTest> simple_astar_test_ptr;

void sigintHandler(int sig) {
  if (simple_astar_test_ptr) {
    simple_astar_test_ptr.reset();
  }
  ROS_INFO("hybrid astar dubins opt shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_astar_test");
  simple_astar_test_ptr = std::make_shared<chibot::topologyplanning::SimpleAstarTest>();
  signal(SIGINT, sigintHandler);
  simple_astar_test_ptr->init();
  simple_astar_test_ptr->run();
  return 0;
}
