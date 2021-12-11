#include <vector>
#include <string>
#include "swarm_robots/agent_node.h"

using std::vector;
using std::string;

int main(int argc, char **argv) {
  std::string ns = "";
  AgentNode agent_node(ns);
  //vector<AgentNode> agent_nodes;
  for (int i = 0; i < 2; i++) {
    //AgentNode agent_node(std::to_string(i));
    //agent_nodes.push_back(agent_node);
  }

  return 0;
}