#include <csignal>
#include "hybrid_astar_dubins_opt.h"

std::shared_ptr<chibot::topologyplanning::HybridAstarDubinsOpt> hybrid_astar_dubins_opt_ptr;

void sigintHandler(int sig) {
  if (hybrid_astar_dubins_opt_ptr) {
    hybrid_astar_dubins_opt_ptr.reset();
  }
  ROS_INFO("hybrid astar dubins opt shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, chibot::topologyplanning::NODE_NAME);
  hybrid_astar_dubins_opt_ptr = std::make_shared<chibot::topologyplanning::HybridAstarDubinsOpt>();
  signal(SIGINT, sigintHandler);
  hybrid_astar_dubins_opt_ptr->init();
  hybrid_astar_dubins_opt_ptr->run();
  return 0;
}
