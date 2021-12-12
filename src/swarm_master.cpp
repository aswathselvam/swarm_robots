#include <vector>
#include <string>
#include <iostream>
#include "swarm_robots/agent_node.h"

using std::vector;
using std::string;
using std::cout;
int main(int argc, char **argv) {
  ros::init(argc,argv,"swarm_master");

  vector<AgentNode*> agent_nodes;
  int n = 2;
  for (int i = 0; i < n; i++) {
    AgentNode* agent_node = new AgentNode(std::to_string(i));
    agent_nodes.push_back(agent_node);
  }

  while (ros::ok())
  {
    for (int i = 0; i < n; i++) {
      agent_nodes[i]->Loop();
    }
  }
  

  return 0;
}