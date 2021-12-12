#include <vector>
#include <string>
#include <iostream>
#include "swarm_robots/agent_node.h"

using std::vector;
using std::string;
using std::cout;
int main(int argc, char **argv) {
  cout << "argc: "<< argc << "argv: " << argv <<std::endl;
  std::string ns = "0";

  AgentNode agent_node(argc, argv, ns);
  //vector<AgentNode> agent_nodes;
  for (int i = 0; i < 2; i++) {
    //AgentNode agent_node(std::to_string(i));
    //agent_nodes.push_back(agent_node);
  }

  return 0;
}