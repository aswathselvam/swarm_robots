#include <swarm_robots/swarm_master.h>
#include <swarm_robots/agent.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "swarm_master_node");
  ros::NodeHandle* nh_master = new ros::NodeHandle();
  MasterNode master_node;
  Agent agent;
  delete nh_master;
  return 0;
}