#include <vector>
#include "swarm_robots/agent_node.h"

using std::vector;

int main(int argc, char **argv) {

  vector<AgentNode> agent_nodes;
  for (int i = 0; i < 2; i++) {
    AgentNode agent_node = new AgentNode(std::to_string(i));
    agent_nodes.push_back(agent_node);
  }

  return 0;
}